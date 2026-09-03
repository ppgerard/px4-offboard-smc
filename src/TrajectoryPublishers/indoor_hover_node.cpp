// Indoor STSMC test profile: hold station over the tag, then optional steps.
//
// This is a CONTROLLER test, not a landing. It exists because the landing rig
// cannot answer "is the control law stable on the real airframe" -- it answers
// "does the whole guidance stack land", and when that fails you cannot tell which
// half did it. Here the reference is a fixed point and a few steps, so the only
// thing under test is the loop from position error to actuator command.
//
// It replaces steps_publisher_node for this purpose because that one commands a
// FIXED [1.5, 0, 3.0], which indoors is a wall and a ceiling, and because it
// publishes an identity attitude -- see the yaw note below.
//
// The setpoint is in the EKF's local frame. With tag_ev_bridge_node feeding
// external vision, the EKF origin IS the tag, so [0, 0, h] is directly over the
// pad. Keep the lateral steps well inside the camera footprint (radius ~= 1.19*h,
// so ~1.4 m at 1.2 m of altitude): losing the tag here is not a degraded
// measurement, it is the loss of the ONLY horizontal aiding source, and EKF2
// drops silently back to fake-position fusion at EKF2_NOAID_NOISE.
//
// STEPS ARE OFF BY DEFAULT. The first flight should be a hold and nothing else.
//
// YAW. steps_publisher_node and the landing node both command an identity
// attitude, which in the ENU convention points the body x-axis East. In SITL that
// is free -- the vehicle spawns aligned with the world x-axis, so the commanded
// heading is the heading it already has. On real hardware the EKF frame is
// north-aligned by the magnetometer and the aircraft sits at whatever heading the
// room gives it, so an identity command is a yaw slew to magnetic East the
// instant offboard engages. This node latches the heading at engagement and holds
// THAT, so handover is a no-op in yaw. (It also means R_B_W is no longer the
// identity, which is exactly why the velocity-frame fix in
// px4_frame_conversions.h had to land with it.)

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include "px4_offboard_lowlevel/control_config.h"
#include "px4_offboard_lowlevel/px4_frame_conversions.h"

class IndoorHoverNode : public rclcpp::Node {
 public:
  IndoorHoverNode() : Node("indoor_hover") {
    altitude_ = this->declare_parameter<double>("hover.altitude", 1.2);
    centre_x_ = this->declare_parameter<double>("hover.centre_x", 0.0);
    centre_y_ = this->declare_parameter<double>("hover.centre_y", 0.0);
    settle_seconds_ = this->declare_parameter<double>("hover.settle_seconds", 20.0);
    enable_steps_ = this->declare_parameter<bool>("hover.enable_steps", false);
    step_xy_ = this->declare_parameter<double>("hover.step_size_xy", 0.3);
    step_z_ = this->declare_parameter<double>("hover.step_size_z", 0.2);
    dwell_seconds_ = this->declare_parameter<double>("hover.step_dwell_seconds", 10.0);
    // Rate-limited reference travel. 0 gives a true step, which is the sharper
    // system-identification input but hands the law an instantaneously infeasible
    // reference; the default is a reference the aircraft can actually follow, so
    // what you measure is tracking rather than saturation.
    slew_speed_ = this->declare_parameter<double>("hover.slew_speed", 0.25);
    // A hard bound on how far this node will ever command from the centre,
    // enforced on the published setpoint rather than trusted from the sequence
    // below. Indoors the useful failure mode is "refuses to go there", not
    // "went there".
    max_radius_ = this->declare_parameter<double>("hover.max_radius", 0.8);
    max_altitude_ = this->declare_parameter<double>("hover.max_altitude", 2.0);
    // Fly the identical profile with PX4'S OWN position controller instead of the
    // SMC: same hold point, same steps, same timing, same estimator underneath.
    // That is what makes the two comparable -- and it is the order to fly them
    // in, because if PX4 cannot hold station on this estimate then nothing
    // measured about the SMC afterwards means anything.
    //
    // The two paths are mutually exclusive by construction: here we publish
    // OffboardControlMode with position=true and a TrajectorySetpoint, while
    // offboard_controller_node publishes direct_actuator=true. Both streaming at
    // once would have PX4 acting on whichever OffboardControlMode arrived last,
    // so the launch file does not start the controller in this mode.
    use_px4_controller_ = this->declare_parameter<bool>("hover.use_px4_controller", false);
    // Where the hold point IS.
    //
    //   "engage" (default) -- the vehicle's own XY at the moment offboard is
    //                         entered. Hand over anywhere and it holds THERE.
    //   "origin"           -- the fixed hover.centre_x/y in the EKF local frame,
    //                         which indoors with the tag as EV is the pad.
    //
    // The default is "engage" because "origin" has a nasty outdoor failure mode:
    // the local origin is wherever EKF2 initialised, so handing over 30 m away
    // means the aircraft immediately sets off back to it at hover.slew_speed. In
    // SITL the two are the same point and the behaviour is identical, which is
    // exactly why this needed catching by reading rather than by testing.
    centre_mode_ = this->declare_parameter<std::string>("hover.centre_mode", "engage");
    // See the note in landing_trajectory_base.h: the "_v<N>" suffix comes from the
    // FIRMWARE's message version, is absent when that version is 0, and differs
    // between PX4 releases. A wrong name here means offboard entry is never
    // detected, so the profile never engages and the reference never latches.
    const std::string status_topic = this->declare_parameter<std::string>(
        "topics_names.status_topic", "/fmu/out/vehicle_status_v1");

    buildSequence();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        std::bind(&IndoorHoverNode::odometryCallback, this, std::placeholders::_1));
    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        status_topic, qos,
        std::bind(&IndoorHoverNode::statusCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
        "command/trajectory", 10);

    if (use_px4_controller_) {
      offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
          "/fmu/in/offboard_control_mode", 10);
      setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
          "/fmu/in/trajectory_setpoint", 10);
      offboard_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(0.33),
          std::bind(&IndoorHoverNode::publishOffboardControlMode, this));
    }

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(px4_offboard::kControlPeriodSeconds),
        std::bind(&IndoorHoverNode::update, this));

    RCLCPP_INFO(this->get_logger(),
                "Indoor hover profile: hold [%.2f %.2f %.2f] m for %.0f s, steps %s "
                "(xy %.2f m, z %.2f m, dwell %.0f s), slew %.2f m/s, "
                "bounded to %.2f m radius and %.2f m altitude.",
                centre_x_, centre_y_, altitude_, settle_seconds_,
                enable_steps_ ? "ENABLED" : "disabled",
                step_xy_, step_z_, dwell_seconds_, slew_speed_, max_radius_, max_altitude_);
  }

 private:
  struct Waypoint {
    Eigen::Vector3d offset;  // from the hold point [m]
    double dwell;            // [s]
    const char *label;
  };

  void buildSequence() {
    sequence_.push_back({{0.0, 0.0, 0.0}, settle_seconds_, "settle over the tag"});
    if (!enable_steps_) {
      return;
    }
    sequence_.push_back({{step_xy_, 0.0, 0.0}, dwell_seconds_, "+x step"});
    sequence_.push_back({{0.0, 0.0, 0.0}, dwell_seconds_, "back to centre"});
    sequence_.push_back({{0.0, step_xy_, 0.0}, dwell_seconds_, "+y step"});
    sequence_.push_back({{0.0, 0.0, 0.0}, dwell_seconds_, "back to centre"});
    sequence_.push_back({{0.0, 0.0, -step_z_}, dwell_seconds_, "descend step"});
    sequence_.push_back({{0.0, 0.0, 0.0}, dwell_seconds_, "back to centre"});
  }

  void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angular_velocity;
    px4_frames::eigenOdometryFromPX4Msg(msg, position, orientation_, velocity, angular_velocity);
    drone_position_ = position;
    if (!odometry_received_) {
      odometry_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Got first odometry at [%.2f %.2f %.2f].",
                  position(0), position(1), position(2));
    }
  }

  void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    const bool offboard =
        msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    if (offboard && !offboard_active_ && odometry_received_) {
      // Latch the heading and the reference AT ENGAGEMENT. Both matter: a stale
      // reference would step the aircraft the moment the loop closes, and an
      // identity heading would slew it to magnetic East.
      held_yaw_ = yawOf(orientation_);
      reference_ = drone_position_;
      if (centre_mode_ != "origin") {
        centre_x_ = drone_position_(0);
        centre_y_ = drone_position_(1);
      }
      sequence_index_ = 0;
      phase_start_ = this->now();
      engaged_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Offboard engaged: holding heading %.1f deg, reference seeded at "
                  "[%.2f %.2f %.2f], centre (%s) [%.2f %.2f], target altitude %.2f m.",
                  held_yaw_ * 180.0 / M_PI, reference_(0), reference_(1), reference_(2),
                  centre_mode_.c_str(), centre_x_, centre_y_, altitude_);
    }
    if (!offboard && offboard_active_) {
      RCLCPP_WARN(this->get_logger(), "Left offboard; holding the profile until re-engaged.");
      engaged_ = false;
    }
    offboard_active_ = offboard;
  }

  static double yawOf(const Eigen::Quaterniond &q) {
    const Eigen::Matrix3d R = q.toRotationMatrix();
    return std::atan2(R(1, 0), R(0, 0));
  }

  void update() {
    if (!odometry_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Waiting for odometry before publishing setpoints.");
      return;
    }

    // Before engagement, publish the vehicle's own position. That is what makes
    // offboard enterable at all (PX4 wants a setpoint stream first) and it means
    // engagement never produces a step.
    if (!engaged_) {
      reference_ = drone_position_;
      publish(reference_, Eigen::Vector3d::Zero(), yawOf(orientation_));
      return;
    }

    const Waypoint &wp = sequence_[sequence_index_];
    Eigen::Vector3d target(centre_x_ + wp.offset(0), centre_y_ + wp.offset(1),
                           altitude_ + wp.offset(2));
    target = clampTarget(target);

    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    if (slew_speed_ > 0.0) {
      const Eigen::Vector3d error = target - reference_;
      const double distance = error.norm();
      const double step = slew_speed_ * px4_offboard::kControlPeriodSeconds;
      if (distance > step) {
        const Eigen::Vector3d direction = error / distance;
        reference_ += direction * step;
        velocity = direction * slew_speed_;
      } else {
        reference_ = target;
      }
    } else {
      reference_ = target;
    }

    publish(reference_, velocity, held_yaw_);

    // Advance only once the reference has arrived, so the dwell is time spent AT
    // the setpoint rather than time spent travelling to it.
    const bool arrived = (target - reference_).norm() < 1e-3;
    if (arrived && (this->now() - phase_start_).seconds() >= wp.dwell) {
      if (sequence_index_ + 1 < sequence_.size()) {
        ++sequence_index_;
        phase_start_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Profile step %zu/%zu: %s.",
                    sequence_index_ + 1, sequence_.size(), sequence_[sequence_index_].label);
      } else if (!profile_complete_) {
        profile_complete_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Profile complete; holding the centre point. Land on the RC when ready.");
      }
    }
    if (!arrived) {
      phase_start_ = this->now();
    }
  }

  Eigen::Vector3d clampTarget(const Eigen::Vector3d &target) const {
    Eigen::Vector3d out = target;
    const Eigen::Vector2d centre(centre_x_, centre_y_);
    Eigen::Vector2d offset = out.head<2>() - centre;
    const double radius = offset.norm();
    if (radius > max_radius_) {
      offset *= max_radius_ / radius;
      out.head<2>() = centre + offset;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Target beyond hover.max_radius (%.2f m); clamped.", max_radius_);
    }
    if (out(2) > max_altitude_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Target above hover.max_altitude (%.2f m); clamped.", max_altitude_);
      out(2) = max_altitude_;
    }
    if (out(2) < 0.2) {
      out(2) = 0.2;
    }
    return out;
  }

  void publish(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity, double yaw) {
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 0;
    point.transforms.resize(1);
    point.velocities.resize(1);

    point.transforms[0].translation.x = position(0);
    point.transforms[0].translation.y = position(1);
    point.transforms[0].translation.z = position(2);

    const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    point.transforms[0].rotation.x = q.x();
    point.transforms[0].rotation.y = q.y();
    point.transforms[0].rotation.z = q.z();
    point.transforms[0].rotation.w = q.w();

    point.velocities[0].linear.x = velocity(0);
    point.velocities[0].linear.y = velocity(1);
    point.velocities[0].linear.z = velocity(2);

    if (use_px4_controller_) {
      // PX4 wants NED. r_position/r_velocity here are ENU, and the yaw is an ENU
      // heading (x-axis East); PX4's yaw is NED (x-axis North), so the two differ
      // by 90 degrees. Getting that wrong is a quarter turn, not noise.
      px4_msgs::msg::TrajectorySetpoint sp{};
      const Eigen::Vector3d p_ned = px4_frames::rotateVectorFromToENU_NED(position);
      const Eigen::Vector3d v_ned = px4_frames::rotateVectorFromToENU_NED(velocity);
      sp.position = {static_cast<float>(p_ned(0)), static_cast<float>(p_ned(1)),
                     static_cast<float>(p_ned(2))};
      sp.velocity = {static_cast<float>(v_ned(0)), static_cast<float>(v_ned(1)),
                     static_cast<float>(v_ned(2))};
      const float nan = std::nanf("1");
      sp.acceleration = {nan, nan, nan};
      sp.jerk = {nan, nan, nan};
      sp.yaw = static_cast<float>(M_PI / 2.0 - yaw);
      sp.yawspeed = nan;
      sp.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);
      setpoint_pub_->publish(sp);
    }

    // No accelerations field, deliberately. r_a enters I_a_d at full authority
    // (49.7 N per metre of reference jump through the legacy path), and a
    // constant-velocity slew has no acceleration worth feeding forward. Leaving
    // it absent is also what makes this rig, like trajectory:=tuning, free of the
    // reference chain -- so a chattering measurement taken here is comparable to
    // every still-air number in CLAUDE.md.

    publisher_->publish(point);
  }

  void publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);
    offboard_mode_pub_->publish(msg);
    RCLCPP_INFO_ONCE(this->get_logger(),
                     "Flying with PX4's OWN position controller (offboard position setpoints). "
                     "offboard_controller_node must NOT be running.");
  }

  bool use_px4_controller_ = false;
  std::string centre_mode_ = "engage";
  double altitude_ = 1.2;
  double centre_x_ = 0.0;
  double centre_y_ = 0.0;
  double settle_seconds_ = 20.0;
  bool enable_steps_ = false;
  double step_xy_ = 0.3;
  double step_z_ = 0.2;
  double dwell_seconds_ = 10.0;
  double slew_speed_ = 0.25;
  double max_radius_ = 0.8;
  double max_altitude_ = 2.0;

  std::vector<Waypoint> sequence_;
  std::size_t sequence_index_ = 0;
  rclcpp::Time phase_start_ = this->now();
  bool profile_complete_ = false;

  bool odometry_received_ = false;
  bool offboard_active_ = false;
  bool engaged_ = false;
  double held_yaw_ = 0.0;
  Eigen::Vector3d drone_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d reference_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr publisher_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr offboard_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndoorHoverNode>());
  rclcpp::shutdown();
  return 0;
}
