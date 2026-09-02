// Tag -> EKF2 external vision bridge.
//
// WHY THIS EXISTS. Indoors there is no GNSS, and this airframe has no optical
// flow (SENS_EN_PMW3901/PAW3902/PAA3905/PX4FLOW are all 0 on the vehicle). With
// no horizontal aiding source EKF2 does NOT report an error and does NOT publish
// NaN -- EKF2::PublishOdometry copies the filter state unconditionally, and
// Ekf::controlFakePosFusion() fuses a fake position measurement at the last
// known position with EKF2_NOAID_NOISE (10 m) of assumed noise, purely to
// constrain drift. So vehicle_odometry keeps arriving, at full rate, finite, and
// completely disconnected from where the aircraft actually is.
//
// That is the failure this node prevents. The SMC closes on that position and
// velocity; the tag only ever reached the OUTER loop, through the landing node's
// guidance. Feeding a soft anchor to the inner loop makes e_v ~= -r_velocity
// permanently -- the estimate never confirms the commanded motion, so -m*Lambda*e_v
// (3.73 N per m/s, the dominant horizontal authority per the force-balance work)
// keeps commanding acceleration in the reference direction and keeps commanding
// it. Nothing in the stack notices: PX4 lets you into Offboard with attitude and
// angular velocity alone (mode_requirements.cpp), controller_node's only guard is
// the odometry_received_ LATCH, and the landing node's Phase 2 gate tests
// offboard_active_, not whether the estimate means anything.
//
// The fix belongs in the ESTIMATOR'S INPUT, not in the control loop. This node
// turns the AprilTag detection into an external-vision position and hands it to
// EKF2, which fuses it with the IMU and republishes a real vehicle_odometry. The
// SMC, the trajectory publishers and every gain stay exactly as they are.
//
// WHY NOT FEED THE TAG STRAIGHT TO THE CONTROLLER. relative_state_filter.h is
// already a relative position/velocity estimator, and on a static pad p_rel and
// v_rel are what the SMC wants -- so routing its state into the controller looks
// like the smaller change. It is not. That filter has NO IMU propagation: it
// propagates on the navigation velocity it is handed, and that velocity is the
// fake anchor above. Disable it and the velocity is observable only as the
// derivative of 15 Hz corner measurements, which then enters a super-twisting law
// through a 3.73 N per m/s damping gain. That is the exact noise path the whole
// omega_ref chattering investigation was about. EKF2 is a tested vision-inertial
// filter; use it rather than reimplementing IMU propagation.
//
// FRAME NOTE, and it is the reassuring one. p_ENU is resolved with R_W_B, the
// EKF's OWN attitude, and then fed back to that EKF as a position. A heading
// error therefore rotates the measurement -- but it rotates the vehicle's
// perceived motion by the same angle, so the loop is self-consistent and
// converges in a frame rotated by the heading error. Directly over the tag,
// r_plat_b_b is ~[0,0,-h] and p_ENU is ~[0,0,h] with zero XY WHATEVER the yaw is,
// so a [0,0,h] setpoint is exactly over the pad regardless. Only two things
// remain: a lateral setpoint moves in a direction offset by the heading error,
// and a DRIFTING heading is a slowly rotating frame, so a lateral hold orbits.
// Both are acceptable for a hover test; publishing EV yaw is the fix if the
// magnetometer misbehaves indoors, and is deliberately not done here (one moving
// part at a time).

#include <deque>
#include <memory>
#include <string>

#include <eigen3/Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "px4_offboard_lowlevel/camera_extrinsics.h"
#include "px4_offboard_lowlevel/control_config.h"
#include "px4_offboard_lowlevel/px4_frame_conversions.h"

namespace {

// A stamp further from now than this is not on our clock at all, rather than
// merely late. Same threshold and same reasoning as the landing node: the real
// detector's transforms carry the SIMULATOR clock (ros_gz_image copies the
// Gazebo header) while this node runs on wall time, and the two do not even run
// at the same rate -- measured at 0.67x, so a fitted offset puts measurements
// hundreds of ms in the past. On real hardware everything is wall time on the
// companion computer and this never trips.
constexpr double kImplausibleStampAge = 10.0;  // [s]

}  // namespace

class TagEvBridgeNode : public rclcpp::Node {
 public:
  TagEvBridgeNode() : Node("tag_ev_bridge") {
    // Shared with the landing node on purpose: one yaml configures both, so the
    // camera geometry cannot disagree between the estimator's input and the
    // guidance that consumes its output.
    camera_frame_id_ = this->declare_parameter<std::string>(
        "landing_parameters.camera_frame_id", "t2_cruza_vtol_0/camera_link/imager");
    platform_frame_id_ = this->declare_parameter<std::string>(
        "landing_parameters.platform_frame_id", "platform");
    r_cam_b_b_ << this->declare_parameter<double>("landing_parameters.camera_offset_body.x", 0.0),
                  this->declare_parameter<double>("landing_parameters.camera_offset_body.y", 0.0),
                  this->declare_parameter<double>("landing_parameters.camera_offset_body.z", -0.10);
    R_b_cam_ = px4_offboard::bodyFromCameraOptical(
        this->declare_parameter<double>("landing_parameters.camera_rotation_z_deg", 90.0),
        this->declare_parameter<double>("landing_parameters.camera_rotation_x_deg", 180.0));

    // Measurement noise handed to EKF2. EKF2_EV_NOISE_MD must be 0 for these to
    // be used at all, and EKF2_EVP_NOISE is then a LOWER bound -- so a value
    // below that parameter is silently raised, which is worth knowing when the
    // innovations look too confident.
    //
    // Monocular tag lateral error grows with range, so the sigma does too. These
    // defaults are deliberately loose: an over-tight EV sigma is what makes EKF2
    // gate the measurement out during motion, and a starved EV fusion drops
    // straight back to the fake-position anchor with no warning anywhere.
    sigma_xy_base_ = this->declare_parameter<double>("ev_bridge.sigma_xy_base", 0.03);
    sigma_xy_slope_ = this->declare_parameter<double>("ev_bridge.sigma_xy_per_metre", 0.02);
    sigma_z_base_ = this->declare_parameter<double>("ev_bridge.sigma_z_base", 0.05);
    sigma_z_slope_ = this->declare_parameter<double>("ev_bridge.sigma_z_per_metre", 0.03);
    max_range_ = this->declare_parameter<double>("ev_bridge.max_range", 8.0);
    attitude_history_length_ =
        this->declare_parameter<double>("ev_bridge.attitude_history_seconds", 1.0);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        std::bind(&TagEvBridgeNode::odometryCallback, this, std::placeholders::_1));

    ev_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        "/fmu/in/vehicle_visual_odometry", 10);
    // Diagnostic: the same position in ENU, before the NED conversion, so a sign
    // or axis error can be read off one topic instead of inferred from flight.
    // Carries no phase or mode condition -- see CLAUDE.md on diagnostics that go
    // quiet in the situation they exist to explain.
    ev_position_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/landing/ev_position", 10);
    // The tag's position in BODY FLU, before any world rotation. This is the
    // topic that validates R_b_cam, and it is the only one that can: it does not
    // pass through R_W_B, so it is independent of the vehicle's heading and of
    // whatever the magnetometer thinks north is. /landing/ev_position cannot do
    // the job -- a heading error and a camera-rotation error look identical there.
    //
    // Ground check, no flight needed. Park the aircraft over the tag and read it,
    // then move the AIRCRAFT (not the tag) in a known BODY direction:
    //   directly above the tag  ->  [ 0,  0, -h ]
    //   moved 0.5 m FORWARD     ->  x goes to -0.5  (the tag is now behind you)
    //   moved 0.5 m LEFT        ->  y goes to -0.5  (the tag is now to your right)
    // Wrong signs on BOTH lateral axes is the 180 deg boresight error that cost
    // this project three roadmap items. One axis wrong is a 90 deg error.
    tag_body_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/landing/tag_in_body", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Polled rather than driven by the detector, so one code path covers the TF
    // and detections sources. Publishing is gated on a NEW transform stamp
    // below, so this rate only bounds latency, it does not resample.
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(px4_offboard::kControlPeriodSeconds),
        std::bind(&TagEvBridgeNode::update, this));

    RCLCPP_INFO(this->get_logger(),
                "Tag EV bridge up: '%s' -> '%s', camera offset [%.3f %.3f %.3f] m, "
                "publishing /fmu/in/vehicle_visual_odometry.",
                platform_frame_id_.c_str(), camera_frame_id_.c_str(),
                r_cam_b_b_(0), r_cam_b_b_(1), r_cam_b_b_(2));
    RCLCPP_INFO(this->get_logger(),
                "EKF2 must be told to use it: EKF2_EV_CTRL bit 0 (horizontal position), "
                "EKF2_GPS_CTRL 0, and EKF2_EV_DELAY set to the measured camera latency.");
  }

 private:
  void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    px4_frames::eigenOdometryFromPX4Msg(msg, position, orientation, velocity, angular_velocity);

    RCLCPP_INFO_ONCE(this->get_logger(), "EV bridge got its first odometry message.");

    // Retain a short attitude history so the tag measurement is resolved with the
    // attitude at ITS OWN stamp rather than the newest available -- the same rule
    // the landing node follows (see the conventions in CLAUDE.md). At a 15 Hz
    // detector the difference is the vehicle's rotation over up to 66 ms, which
    // at 1 m of range is centimetres of position error.
    const rclcpp::Time arrival = this->now();
    attitude_history_.push_back({arrival, orientation});
    while (!attitude_history_.empty() &&
           (arrival - attitude_history_.front().time).seconds() > attitude_history_length_) {
      attitude_history_.pop_front();
    }
  }

  // Attitude interpolated (slerp) to the measurement's own stamp. False when the
  // stamp falls outside the history, which the caller answers by using the newest
  // attitude -- that costs latency compensation and nothing else.
  bool attitudeAt(const rclcpp::Time &stamp, Eigen::Quaterniond &out) const {
    if (attitude_history_.size() < 2) {
      return false;
    }
    if (stamp < attitude_history_.front().time || stamp > attitude_history_.back().time) {
      return false;
    }
    for (std::size_t i = 1; i < attitude_history_.size(); ++i) {
      const auto &a = attitude_history_[i - 1];
      const auto &b = attitude_history_[i];
      if (stamp >= a.time && stamp <= b.time) {
        const double span = (b.time - a.time).seconds();
        const double t = (span > 1e-9) ? (stamp - a.time).seconds() / span : 0.0;
        out = a.orientation.slerp(t, b.orientation);
        return true;
      }
    }
    return false;
  }

  void update() {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(camera_frame_id_, platform_frame_id_,
                                              tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "No transform '%s' -> '%s' yet: %s",
                           platform_frame_id_.c_str(), camera_frame_id_.c_str(), ex.what());
      return;
    }

    // canTransform()/lookupTransform() is NOT a visibility test: it succeeds on
    // the last detection for as long as the transform stays in the buffer, and
    // the detector publishes EMPTY TF messages when it sees nothing. Publishing
    // an EV position off a cached detection would feed EKF2 a stale absolute
    // position as though it were current, which is strictly worse than
    // publishing nothing -- so gate on the stamp actually advancing.
    const rclcpp::Time stamp(transform.header.stamp, this->get_clock()->get_clock_type());
    if (have_last_stamp_ && stamp <= last_stamp_) {
      return;
    }
    have_last_stamp_ = true;
    last_stamp_ = stamp;

    Eigen::Vector3d r_plat_cam_cam;
    r_plat_cam_cam << transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z;

    // The tag in BODY FLU. Published FIRST, and deliberately before the attitude
    // gate below, because it does not use the attitude: it is
    // R_b_cam * r_plat_cam + r_cam_b, camera geometry and nothing else. That is
    // what makes it the right signal for validating R_b_cam -- and it means the
    // extrinsic can be checked on a bench with only the camera and this node, with
    // no flight controller connected and nothing that could arm.
    const Eigen::Vector3d r_plat_b_b = R_b_cam_ * r_plat_cam_cam + r_cam_b_b_;
    geometry_msgs::msg::Vector3 body_diag;
    body_diag.x = r_plat_b_b(0);
    body_diag.y = r_plat_b_b(1);
    body_diag.z = r_plat_b_b(2);
    tag_body_pub_->publish(body_diag);

    if (attitude_history_.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Tag seen and /landing/tag_in_body is publishing, but there is no "
                           "vehicle attitude yet, so no external vision can be sent. Fine for "
                           "the extrinsic check; connect the flight controller to fly.");
      return;
    }

    const rclcpp::Time arrival = this->now();
    const double stamp_age = (arrival - stamp).seconds();
    const bool stamp_usable = std::abs(stamp_age) < kImplausibleStampAge;

    Eigen::Quaterniond orientation;
    if (!stamp_usable || !attitudeAt(stamp, orientation)) {
      orientation = attitude_history_.back().orientation;
      if (!stamp_usable && !warned_clock_) {
        warned_clock_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "Tag stamps are not on this node's clock (age %.0f s): they carry simulator "
                    "time while this node runs on wall time. Using arrival time for EV instead, "
                    "and pairing with the latest attitude. EKF2 will place the measurement at "
                    "arrival, so set EKF2_EV_DELAY to cover the real latency. On hardware "
                    "everything is wall time and this does not happen.",
                    stamp_age);
      }
    }

    const Eigen::Matrix3d R_W_B = orientation.toRotationMatrix();

    // Vehicle position relative to the tag, in world (ENU) axes. Identical
    // expression to landing_trajectory_base.h's updateTagPosition(): the tag
    // translation is resolved with the EKF attitude, NOT with the PnP rotation.
    // Two concentric markers give the solve no baseline, so its out-of-plane tilt
    // is where the planar ambiguity lives -- it can wobble degrees between frames
    // while the translation stays put, and at 2 m a degree is ~3.5 cm of injected
    // position noise. See item 3 in CLAUDE.md.
    const Eigen::Vector3d p_enu = -(R_W_B * r_plat_b_b);

    const double range = r_plat_cam_cam.norm();
    if (!p_enu.allFinite() || range > max_range_ || range < 1e-3) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Rejecting tag measurement at %.2f m range (limit %.1f m).",
                           range, max_range_);
      return;
    }

    geometry_msgs::msg::Vector3 diag;
    diag.x = p_enu(0);
    diag.y = p_enu(1);
    diag.z = p_enu(2);
    ev_position_pub_->publish(diag);

    const Eigen::Vector3d p_ned = px4_frames::rotateVectorFromToENU_NED(p_enu);

    px4_msgs::msg::VehicleOdometry msg{};
    // timestamp_sample is the ONLY field EKF2 uses to place the sample in its
    // buffer (EKF2::UpdateExtVisionSample: ev_data.time_us = ev_odom.timestamp_sample).
    // Stamp it on the ROS clock: the uXRCE-DDS client applies its timesync offset
    // to inbound messages, so PX4 receives it in PX4 time. That requires
    // UXRCE_DDS_SYNCT = 1, which the vehicle already has.
    const rclcpp::Time sample_time = stamp_usable ? stamp : arrival;
    msg.timestamp_sample = static_cast<uint64_t>(sample_time.nanoseconds() / 1000);
    msg.timestamp = static_cast<uint64_t>(arrival.nanoseconds() / 1000);

    msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    msg.position = {static_cast<float>(p_ned(0)), static_cast<float>(p_ned(1)),
                    static_cast<float>(p_ned(2))};

    // NaN is how "not provided" is spelled here, and each block is gated
    // independently on isAllFinite(). Attitude is left out because EKF2 has a far
    // better one from the IMU, and velocity because EKF2 derives it by fusing
    // these positions with the IMU -- which is the entire reason for routing
    // through EKF2 rather than feeding the controller directly.
    const float nan = std::nanf("1");
    msg.q = {nan, nan, nan, nan};
    msg.velocity = {nan, nan, nan};
    msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    msg.angular_velocity = {nan, nan, nan};
    msg.orientation_variance = {nan, nan, nan};
    msg.velocity_variance = {nan, nan, nan};

    const double sigma_xy = sigma_xy_base_ + sigma_xy_slope_ * range;
    const double sigma_z = sigma_z_base_ + sigma_z_slope_ * range;
    msg.position_variance = {static_cast<float>(sigma_xy * sigma_xy),
                             static_cast<float>(sigma_xy * sigma_xy),
                             static_cast<float>(sigma_z * sigma_z)};

    msg.reset_counter = 0;
    msg.quality = 0;

    ev_pub_->publish(msg);

    if (!published_once_) {
      published_once_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "First EV position published: ENU [%.3f %.3f %.3f] m at %.2f m range, "
                  "sigma_xy %.3f m. Check EKF2 actually accepted it -- "
                  "estimator_aid_src_ev_pos innovations, and vehicle_local_position.xy_valid.",
                  p_enu(0), p_enu(1), p_enu(2), range, sigma_xy);
    }
    ++published_count_;
    if ((arrival - last_rate_report_).seconds() >= 5.0) {
      RCLCPP_INFO(this->get_logger(), "EV bridge: %.1f Hz over the last %.1f s, range %.2f m.",
                  published_count_ / (arrival - last_rate_report_).seconds(),
                  (arrival - last_rate_report_).seconds(), range);
      published_count_ = 0;
      last_rate_report_ = arrival;
    }
  }

  struct AttitudeSample {
    rclcpp::Time time;
    Eigen::Quaterniond orientation;
  };

  std::string camera_frame_id_;
  std::string platform_frame_id_;
  Eigen::Vector3d r_cam_b_b_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_b_cam_ = Eigen::Matrix3d::Identity();

  double sigma_xy_base_ = 0.03;
  double sigma_xy_slope_ = 0.02;
  double sigma_z_base_ = 0.05;
  double sigma_z_slope_ = 0.03;
  double max_range_ = 8.0;
  double attitude_history_length_ = 1.0;

  std::deque<AttitudeSample> attitude_history_;
  rclcpp::Time last_stamp_;
  bool have_last_stamp_ = false;
  bool warned_clock_ = false;
  bool published_once_ = false;
  int published_count_ = 0;
  rclcpp::Time last_rate_report_ = this->now();

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr ev_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ev_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr tag_body_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagEvBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
