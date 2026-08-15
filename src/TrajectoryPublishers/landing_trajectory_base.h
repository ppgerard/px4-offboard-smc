#ifndef LANDING_TRAJECTORY_BASE_H
#define LANDING_TRAJECTORY_BASE_H

// Shared logic for AprilTag-based landing trajectory nodes: TF lookup of the
// platform tag, a simple position fusion filter (odometry prediction +
// tag correction), and the 4-phase approach/descent/commit/touchdown state
// machine.
//
// The tag supplies the translation and the in-plane yaw; the attitude used to
// resolve that translation into world axes comes from the EKF, not from the
// tag. Two concentric markers give the pose solve no baseline, so its
// out-of-plane rotation is the one output that cannot be trusted — see
// updateTagPosition().
//
// The vision loop stays closed all the way to the ground: PX4's NAV_LAND is
// never used, because it descends on PX4's own position estimate with no
// knowledge of the tag. Below the commit altitude the XY reference is frozen
// at its last good value and the aircraft descends at a fixed rate until the
// land detector reports contact, at which point this node disarms.
//
// Concrete nodes differ only in where the resulting setpoint is sent:
// through the SMC controller (landing_trajectory_node.cpp) or directly to
// PX4 (px4_offboard_landing_node.cpp). That difference is captured by the
// onSetpointPublished() hook.

#include <algorithm>
#include <chrono>
#include <deque>
#include <memory>
#include <cmath>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <eigen3/Eigen/Eigen>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include "px4_offboard_lowlevel/control_config.h"
#include "px4_offboard_lowlevel/px4_frame_conversions.h"
#include "diagnostics_publisher.h"

using namespace std::chrono_literals;

class LandingTrajectoryNodeBase : public rclcpp::Node {
public:
  explicit LandingTrajectoryNodeBase(const std::string &node_name) : Node(node_name) {

    // Vehicle-specific AprilTag/camera mounting parameters. Values below are
    // declared with the previous t2 defaults; override per-vehicle via
    // config/uav_parameters/{t2,x500}_param.yaml (landing_parameters.*).
    this->declare_parameter("landing_parameters.camera_frame_id", "t2_cruza_vtol_0/camera_link/imager");
    this->declare_parameter("landing_parameters.platform_frame_id", "platform");
    this->declare_parameter("landing_parameters.world_frame_id", "world");
    this->declare_parameter("landing_parameters.camera_offset_body.x", 0.0);
    this->declare_parameter("landing_parameters.camera_offset_body.y", 0.0);
    this->declare_parameter("landing_parameters.camera_offset_body.z", -0.03);

    camera_frame_id_ = this->get_parameter("landing_parameters.camera_frame_id").as_string();
    platform_frame_id_ = this->get_parameter("landing_parameters.platform_frame_id").as_string();
    world_frame_id_ = this->get_parameter("landing_parameters.world_frame_id").as_string();
    r_cam_b_b_ = Eigen::Vector3d(
        this->get_parameter("landing_parameters.camera_offset_body.x").as_double(),
        this->get_parameter("landing_parameters.camera_offset_body.y").as_double(),
        this->get_parameter("landing_parameters.camera_offset_body.z").as_double());

    // TF2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Defining the compatible ROS 2 predefined QoS for PX4 topics
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Subscriber to drone odometry for initialization
    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>
        ("fmu/out/vehicle_odometry", qos, std::bind(&LandingTrajectoryNodeBase::odometryCallback, this, std::placeholders::_1));
    // Subscriber to groundtruth for diagnostics
    groundtruth_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>
        ("/fmu/out/vehicle_local_position_groundtruth_v1", qos, std::bind(&LandingTrajectoryNodeBase::groundtruthCallback, this, std::placeholders::_1));
    // Touchdown detection and disarm confirmation for the terminal descent
    land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>
        ("/fmu/out/vehicle_land_detected", qos, std::bind(&LandingTrajectoryNodeBase::landDetectedCallback, this, std::placeholders::_1));
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>
        ("/fmu/out/vehicle_status_v1", qos, std::bind(&LandingTrajectoryNodeBase::vehicleStatusCallback, this, std::placeholders::_1));

    // Publishers
    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>
        ("command/trajectory", 10);
    platform_position_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>
        ("landing/platform_position", 10);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>
        ("/fmu/in/vehicle_command", 10);

    // Initialize diagnostics publisher
    diagnostics_ = std::make_unique<DiagnosticsPublisher>(this);

    // Timer: 100 Hz (0.01 s)
    timer_ = this->create_wall_timer(std::chrono::duration<double>(px4_offboard::kControlPeriodSeconds),
                                     std::bind(&LandingTrajectoryNodeBase::publishLandingTrajectory, this));

    // Initialize frequency monitoring
    last_callback_time_ = std::chrono::high_resolution_clock::now();

    // Terminal-descent timers use the node clock (see the conventions in
    // CLAUDE.md); they must share its clock type to be comparable.
    const rclcpp::Time now = this->now();
    commit_wait_start_time_ = now;
    commit_start_time_ = now;
    last_disarm_request_time_ = now;
    last_tag_update_time_ = now;

    // Initialize drone state with defaults
    drone_orientation_W_ = Eigen::Quaterniond::Identity();
    drone_angular_velocity_W_.setZero();

    // R_b_cam: rotation from the camera OPTICAL frame to body FLU (+90° about z,
    // then 180° about x), giving optical x -> body -Y, optical y -> body -X,
    // optical z -> body -Z.
    //
    // Derived from the mounting, not guessed. The camera link sits at
    // <pose>0 0 -0.10 0 1.5707 0</pose>, so Ry(90°) takes link x -> body -Z
    // (boresight down), link y -> body +Y, link z -> body +X. The detector's
    // translation is a solvePnP output and so is in the ROS optical convention
    // (x right, y down, z forward), which relates to the link frame by
    // optical_x = -link_y, optical_y = -link_z, optical_z = +link_x.
    //
    // This used to use -pi/2, which inverts both lateral axes: a 180° error
    // about the boresight. It reported the vehicle on the opposite side of the
    // pad, so the approach flew directly away from the tag and the aircraft
    // drifted off. It survived every synthetic test because fake_tag_tf.py was
    // written to mirror this same constant, so the error cancelled there and
    // only ever appeared against a real camera. Measured before the fix: the
    // estimate read [-1.93 -1.23 2.69] with the vehicle truly at [1.99 0.90 3.04].
    Eigen::AngleAxisd rot_z(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rot_x(M_PI, Eigen::Vector3d::UnitX());
    R_b_cam_ = (rot_x * rot_z).toRotationMatrix();

    // Initialize Phase 1
    phase_ = Phase::PHASE_1;
    r_position_W_.setZero();
    r_velocity_W_.setZero();
    r_acceleration_W_.setZero();
    position_W_.setZero();
    position_W_filtered_.setZero();
    position_W_raw_.setZero();

    // Log frame IDs for debugging
    RCLCPP_INFO(this->get_logger(), "Landing trajectory node initialized");
    RCLCPP_INFO(this->get_logger(), "Looking for transform from '%s' to '%s'",
                platform_frame_id_.c_str(), camera_frame_id_.c_str());
  }

  ~LandingTrajectoryNodeBase() override = default;

protected:
  enum class Phase {
    PHASE_1,
    PHASE_2,
    PHASE_3_COMMIT,
    PHASE_4_TOUCHDOWN
  };

  // Hook for subclasses that need to publish an additional setpoint
  // representation (e.g. direct PX4 TrajectorySetpoint) whenever a new
  // trajectory point has been computed. No-op by default.
  virtual void onSetpointPublished() {}

  // ---- Loop timing, gains, filters -------------------------------------------
  const double dt_ = px4_offboard::kControlPeriodSeconds;  // control period [s]
  const double K_p_ = 1.0;                    // outer-loop position gain
  const double lpf_alpha_ = 1.0;              // AprilTag pose filter [0, 1]; 1 = disabled
  const double lpf_alpha_velocity_ = 0.2;     // reference velocity filter [0, 1]
  const double lpf_alpha_odometry_vz_ = 0.05; // odometry vz filter, for the contact check

  // ---- Phase 1: approach ------------------------------------------------------
  const Eigen::Vector3d phase_1_target_{0.0, 0.0, 3.0};  // hold point above the tag [m]
  const double max_velocity_xy_ = 1.0;      // [m/s]
  const double max_velocity_z_ = 2.0;       // [m/s]
  const double max_acceleration_xy_ = 2.5;  // [m/s²]
  const double max_acceleration_z_ = 1.0;   // [m/s²]
  const double xy_error_threshold_ = 0.3;   // XY error that counts as "over the tag" [m]
  const double z_error_threshold_ = 0.3;    // ... altitude error that counts as "at the hold point" [m]
  const double settled_velocity_z_ = 0.2;   // ... and vertical speed that counts as settled [m/s]
  const double xy_error_min_time_ = 0.5;    // ... all held this long before descending [s]
  const double airborne_altitude_ = 1.0;    // above this the vehicle has certainly flown [m]
  // Longer than the descent gate: losing the tag for a moment while still far
  // from the pad should not throw away the approach, but steering at a stale
  // estimate for longer than this is a fly-away rather than a hold.
  const double phase1_tag_max_age_ = 1.0;   // aim at the tag only this fresh [s]

  // ---- Phase 2: vision-guided descent ----------------------------------------
  //
  // Every altitude from here down is the fused tag-relative altitude of the body
  // origin, so it includes the vehicle's own ground clearance: a T2 at rest on
  // the pad reads ~0.105 m, not zero.
  const Eigen::Vector3d phase_2_target_{0.0, 0.0, 0.0};  // the tag itself
  const double phase2_max_velocity_xy_ = 0.5;      // [m/s]
  const double phase2_max_velocity_z_ = 0.3;       // [m/s]
  const double phase2_max_acceleration_xy_ = 0.8;  // [m/s²]
  const double phase2_max_acceleration_z_ = 0.3;   // [m/s²]
  // Descent cone: altitude is only given up while the aircraft is inside a cone
  // that narrows as it descends, which bounds the touchdown error by design
  // instead of measuring it afterwards. A gust that pushes the aircraft
  // off-centre pauses the descent rather than racing it to the ground.
  const double cone_slope_ = 0.30;       // cone radius gained per metre of height [m/m]
  const double cone_radius_min_ = 0.05;  // cone radius at the pad [m]
  const double cone_tag_max_age_ = 0.3;  // no fresh tag, no descent [s]

  // ---- Phase 3: commit --------------------------------------------------------
  const double commit_altitude_ = 0.20;      // commit below this altitude (~0.10 m clearance) [m]
  const double commit_xy_error_max_ = 0.10;  // XY alignment required to commit [m]
  const double commit_tag_max_age_ = 0.5;    // tag measurement must be fresher than this [s]
  const double commit_wait_timeout_ = 5.0;   // commit regardless after waiting this long [s]
  const double commit_descent_rate_ = 0.4;   // fixed descent rate once committed [m/s]
  const double commit_timeout_ = 8.0;        // no touchdown by then: stop descending [s]

  // ---- Phase 4: touchdown and disarm -----------------------------------------
  const double touchdown_max_altitude_ = 0.30;  // contact only ever declared this close to the pad [m]
  const double touchdown_ref_gap_ = 0.25;       // reference this far below the vehicle = not following [m]
  const double touchdown_stall_speed_ = 0.10;   // filtered descent rate below this counts as stopped [m/s]
  const double touchdown_stall_time_ = 0.7;     // evidence to accumulate before calling contact [s]
  const double disarm_retry_period_ = 0.5;      // resend DISARM this often [s]

  // Hysteresis timer for transition Phase 1 -> Phase 2
  std::chrono::high_resolution_clock::time_point phase2_transition_start_time_;
  bool phase2_transition_flag_ = false;

  // Vehicle-specific frame IDs and camera mounting offset (loaded from ROS params)
  std::string camera_frame_id_;
  std::string platform_frame_id_;
  std::string world_frame_id_;
  Eigen::Vector3d r_cam_b_b_;  // position of camera in body frame, expressed in body frame

  // Transformation matrices (constant)
  // R_b_cam: fixed transformation from camera to body (-90° around z, then 180° around x)
  Eigen::Matrix3d R_b_cam_;

  // Transformation state (updated from TF)
  Eigen::Matrix3d R_plat_cam_;          // Rotation: platform w.r.t. camera (from TF; yaw only)
  Eigen::Vector3d r_plat_cam_cam_;      // Position: platform in camera frame (from TF)
  Eigen::Vector3d position_W_filtered_; // Filtered platform position in world frame

  // ---- Platform yaw ----------------------------------------------------------
  // The one rotational quantity the tag is kept for. Filtered hard: the platform
  // is static, so anything moving fast in this signal is noise.
  const double lpf_alpha_platform_yaw_ = 0.02;  // per tag measurement (~15 Hz)
  double platform_yaw_raw_ = 0.0;               // [rad], world frame
  double platform_yaw_filtered_ = 0.0;          // [rad], world frame
  bool platform_yaw_initialized_ = false;

  // Current phase
  Phase phase_;
  // Flag to track if trajectory has been initialized
  bool trajectory_initialized_ = false;
  // Initialization delay: stay at current position for 5 seconds before starting Phase 1
  // During this period, the drone publishes its own current position as setpoint,
  // allowing time for offboard mode activation without moving
  const double initialization_delay_seconds_ = 5.0;
  std::chrono::high_resolution_clock::time_point initialization_time_;

  // Trajectory state (setpoints)
  Eigen::Vector3d r_position_W_;        // Reference position in world frame
  Eigen::Vector3d r_velocity_W_;        // Reference velocity in world frame
  Eigen::Vector3d r_acceleration_W_;    // Reference acceleration in world frame
  Eigen::Vector3d position_W_;          // Actual position of body in world frame (Phase 2)

  // Current drone state (updated from odometry)
  Eigen::Vector3d drone_position_W_;    // Actual drone position from odometry
  Eigen::Vector3d drone_velocity_W_;    // Actual drone velocity from odometry

  // Fused position estimate (100 Hz prediction + 15 Hz correction from tag)
  Eigen::Vector3d estimated_position_W_;      // Fused estimate at 100Hz
  tf2::TimePoint last_tag_measurement_time_;  // Timestamp of last processed TF measurement
  rclcpp::Time last_tag_update_time_;         // When that measurement arrived (node clock)
  bool tag_measurement_received_ = false;
  const double fusion_alpha_ = 0.1;          // Correction gain (0.05-0.2)

  // Terminal-descent state
  Eigen::Vector2d commit_position_xy_ = Eigen::Vector2d::Zero();  // frozen XY reference
  rclcpp::Time commit_wait_start_time_;          // waiting for alignment at commit altitude
  bool commit_wait_flag_ = false;
  rclcpp::Time commit_start_time_;
  bool commit_timeout_warned_ = false;
  double contact_score_ = 0.0;                   // integrated evidence of contact [s]
  double drone_velocity_filtered_z_ = 0.0;       // low-passed vertical speed [m/s]
  rclcpp::Time last_disarm_request_time_;
  int disarm_attempts_ = 0;
  bool disarm_confirmed_ = false;

  // PX4 land detector and arming state
  bool land_detected_landed_ = false;
  bool land_detected_maybe_landed_ = false;
  bool land_detected_ground_contact_ = false;
  bool airborne_ = false;   // has left the ground at least once
  bool vehicle_status_received_ = false;
  bool vehicle_armed_ = false;
  bool offboard_active_ = false;  // this stack is flying the vehicle, not PX4

  // Frequency monitoring
  std::chrono::high_resolution_clock::time_point last_callback_time_;
  int callback_count_ = 0;

  // ROS objects
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr platform_position_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr groundtruth_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Diagnostics publisher
  std::unique_ptr<DiagnosticsPublisher> diagnostics_;

  // Current drone state with orientation (from odometry callback)
  Eigen::Quaterniond drone_orientation_W_;
  Eigen::Vector3d drone_angular_velocity_W_;
  uint64_t last_odometry_timestamp_ = 0;

  // ---- Attitude history -------------------------------------------------------
  // The tag translation is stamped at image capture, but drone_orientation_W_ is
  // whatever arrived most recently. Rotating an old translation by a new attitude
  // would put the vision latency straight back in as an attitude error — which is
  // the error this node now takes the EKF attitude to avoid. Keep a short history
  // and interpolate to the measurement's own stamp instead.
  //
  // Stamped with the arrival time on the node clock, not with the PX4 timestamp
  // in the message: that one is microseconds since flight-controller boot, and
  // would not be comparable with the ROS-clock stamps on the TF.
  struct AttitudeSample {
    rclcpp::Time time;
    Eigen::Quaterniond orientation;
  };
  std::deque<AttitudeSample> attitude_history_;
  const double attitude_history_length_ = 1.0;  // history retained [s]
  // An "age" beyond this is not a late measurement, it is a different clock.
  static constexpr double kImplausibleMeasurementAge = 3600.0;  // [s]

  // Groundtruth state for diagnostics
  Eigen::Vector3d groundtruth_position_W_;
  Eigen::Vector3d groundtruth_velocity_W_;

  // Raw tag position (unfiltered)
  Eigen::Vector3d position_W_raw_;

  // ============ Helper Functions: Saturation & Filtering ============
  void saturateXY(Eigen::Vector3d& vector, double max_norm) {
    double xy_norm = std::sqrt(vector(0) * vector(0) + vector(1) * vector(1));
    if (xy_norm > max_norm) {
      double scale = max_norm / xy_norm;
      vector(0) *= scale;
      vector(1) *= scale;
    }
  }

  void saturateZ(Eigen::Vector3d& vector, double max_val) {
    if (vector(2) > max_val) {
      vector(2) = max_val;
    } else if (vector(2) < -max_val) {
      vector(2) = -max_val;
    }
  }

  Eigen::Vector3d applyLowPassFilter(const Eigen::Vector3d& new_value,
                                     const Eigen::Vector3d& old_value,
                                     double alpha) {
    return alpha * new_value + (1.0 - alpha) * old_value;
  }

  double applyLowPassFilterScalar(double new_value, double old_value, double alpha) {
    return alpha * new_value + (1.0 - alpha) * old_value;
  }

  void monitorFrequency() {
    auto current_time = std::chrono::high_resolution_clock::now();
    callback_count_++;

    // Print frequency every 100 callbacks (should be ~1 second if running at 100Hz)
    if (callback_count_ >= 100) {
      auto duration = std::chrono::duration<double>(
          current_time - last_callback_time_).count();
      double actual_freq = callback_count_ / duration;
      if (actual_freq < 95.0) {
        RCLCPP_WARN(this->get_logger(),
                   "WARNING: Actual frequency is %.1f Hz (target 100 Hz).",
                   actual_freq);
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Actual frequency: %.1f Hz", actual_freq);
      }
      last_callback_time_ = current_time;
      callback_count_ = 0;
    }
  }

  // ============ Fused Position Estimation (100Hz prediction + 15Hz correction) ============
  void correctFusedPositionWithTag(const Eigen::Vector3d& tag_position) {
    // Correction step: apply tag measurement
    // Innovation-based correction with limited gain to smooth out jumps
    Eigen::Vector3d innovation = tag_position - estimated_position_W_;
    estimated_position_W_ += fusion_alpha_ * innovation;
  }

  // ============ Initialization Delay Handling ============
  bool handleInitializationDelay() {
    // Check if still in initialization delay phase
    auto time_since_init = std::chrono::high_resolution_clock::now() - initialization_time_;
    double init_elapsed_seconds = std::chrono::duration<double>(time_since_init).count();

    if (init_elapsed_seconds < initialization_delay_seconds_) {
      // Still in initialization delay: use drone's actual current state as setpoint
      // This allows drone to receive its own position, effectively keeping it stationary
      r_position_W_ = drone_position_W_;
      r_velocity_W_.setZero();
      r_acceleration_W_.setZero();
      publishTrajectoryPoint();
      onSetpointPublished();
      return true;  // Still in initialization delay
    }
    return false;  // Initialization delay complete
  }

  // ============ Main Control Loop ============
  void publishLandingTrajectory() {
    // Don't publish until trajectory is initialized
    if (!trajectory_initialized_) {
      return;
    }

    // Handle initialization delay phase (publish drone's own position for 5 seconds)
    if (handleInitializationDelay()) {
      publishDiagnostics();
      return;  // Still in initialization delay, setpoint already published
    }

    // Arm the contact check once the vehicle has climbed clear of the pad.
    if (estimated_position_W_(2) > airborne_altitude_) {
      airborne_ = true;
    }

    // Phase-specific trajectory generation and transitions
    switch (phase_) {
      case Phase::PHASE_1:
        updatePhase1();
        checkPhase1To2Transition();
        break;
      case Phase::PHASE_2:
        updatePhase2();
        checkPhase2To3Transition();
        break;
      case Phase::PHASE_3_COMMIT:
        updatePhase3Commit();
        checkPhase3To4Transition();
        break;
      case Phase::PHASE_4_TOUCHDOWN:
        updatePhase4Touchdown();
        break;
    }

    // The setpoint stream runs through touchdown: it is what keeps the vehicle
    // in offboard while the disarm request is acknowledged.
    publishTrajectoryPoint();
    onSetpointPublished();

    // Publish the tag-relative estimate in every phase. This used to be gated to
    // phases 2 and 3, which meant the one signal that explains what the aircraft
    // thinks it is doing went silent in exactly the phase where it now aims --
    // and a Phase 1 that never advanced looked, from outside, like a vehicle
    // with no estimate at all. Diagnostics should not have phase conditions.
    publishPlatformPosition();

    // Publish diagnostic topics
    publishDiagnostics();
  }

  void checkPhase1To2Transition() {
    // Error against the tag, in the same frame Phase 1 now flies in, so the
    // condition that releases the descent is the condition the descent needs:
    // centred over the pad, not merely settled somewhere.
    const Eigen::Vector3d error = phase_1_target_ - estimated_position_W_;
    const double xy_error = error.head<2>().norm();
    const double z_error = std::abs(error(2));
    const double climb_rate = drone_velocity_W_(2);

    // canTransform() is not a visibility test. It succeeds on the last detection
    // for as long as the transform stays in the buffer, and the detector keeps
    // publishing empty TF messages while it sees nothing, so this used to hand
    // Phase 2 an aircraft that had not seen the tag for minutes. Require a
    // measurement recent enough to steer on.
    const bool tag_visible = tagIsFresh(cone_tag_max_age_);

    // The hold has to be settled, not merely centred. Phase 2 rate-limits its
    // reference velocity to phase2_max_acceleration_z_, so handing it a climb
    // buys seconds of reference travel in the wrong direction before the
    // descent even begins: reversing +2 m/s takes ~7.7 s, and the reference
    // walks metres upward in the meantime. Alignment alone is satisfied from
    // the first instant of a climb that starts centred over the tag, which is
    // exactly the case that produced 4.9-5.7 m overshoots against a 3 m target.
    //
    // The airborne check guards the opposite end: a node started while the
    // vehicle sits disarmed on the pad reads a settled hold on every axis, and
    // would run straight through Phase 2 into the descent without having flown.
    const bool settled = xy_error < xy_error_threshold_ &&
                         z_error < z_error_threshold_ &&
                         std::abs(climb_rate) < settled_velocity_z_;

    // Descend only once this stack is the one flying. Phase 2 integrates its
    // reference open-loop and assumes the vehicle follows it; while PX4 holds
    // the aircraft — flying its own takeoff, say — that assumption is false,
    // and the reference walks away for the whole handover wait with nothing to
    // pull it back. Measured at -7.6 m below the aircraft, which the SMC then
    // dives after at engagement. Phase 1 is safe to sit in meanwhile: it
    // re-anchors its reference to the vehicle every cycle.
    if (settled && tag_visible && airborne_ && offboard_active_) {
      if (!phase2_transition_flag_) {
        phase2_transition_start_time_ = std::chrono::high_resolution_clock::now();
        phase2_transition_flag_ = true;
      }
      // Check if conditions have been true long enough
      auto duration = std::chrono::high_resolution_clock::now() - phase2_transition_start_time_;
      if (std::chrono::duration<double>(duration).count() >= xy_error_min_time_) {
        RCLCPP_INFO(this->get_logger(),
                    "Transitioning to Phase 2 (XY error=%.3f m, altitude error=%.3f m, vz=%.2f m/s)",
                    xy_error, z_error, climb_rate);
        phase_ = Phase::PHASE_2;
        phase2_transition_flag_ = false;
      }
    } else {
      // Reset if conditions are not met
      phase2_transition_flag_ = false;
    }
  }

  void checkPhase2To3Transition() {
    // An unexpected contact (mis-scaled estimate, an obstacle) short-circuits
    // the descent: there is nothing left to commit to.
    if (contactHeld()) {
      RCLCPP_WARN(this->get_logger(),
                  "Contact detected in Phase 2 at altitude=%.3f m; disarming.",
                  estimated_position_W_(2));
      enterTouchdownPhase();
      return;
    }

    // Altitude is the trigger; alignment and a fresh tag measurement are the
    // guards, so the aircraft does not commit while blown off-centre or while
    // dead-reckoning. Phase 2 holds at the commit altitude until they are met.
    if (estimated_position_W_(2) >= commit_altitude_) {
      commit_wait_flag_ = false;
      return;
    }

    if (!commit_wait_flag_) {
      commit_wait_start_time_ = this->now();
      commit_wait_flag_ = true;
    }

    const double xy_error = estimated_position_W_.head<2>().norm();
    const bool aligned = xy_error < commit_xy_error_max_;
    const bool tag_fresh = tagIsFresh(commit_tag_max_age_);
    const bool waited_long_enough =
        (this->now() - commit_wait_start_time_).seconds() >= commit_wait_timeout_;

    if (!(aligned && tag_fresh)) {
      if (!waited_long_enough) {
        return;  // hold at the commit altitude and keep trying to centre
      }
      RCLCPP_WARN(this->get_logger(),
                  "Committing after %.1f s wait: XY error=%.3f m, tag %s",
                  commit_wait_timeout_, xy_error, tag_fresh ? "fresh" : "stale");
    }

    enterCommitPhase(xy_error);
  }

  void enterCommitPhase(double xy_error) {
    // Freeze the XY reference at its last good value. Close to the pad the tag
    // is large in frame and the estimate gets noisy, while the offset the outer
    // loop has already built up against the wind is exactly what should be kept.
    commit_position_xy_ = r_position_W_.head<2>();

    // Re-anchor the altitude reference to where the vehicle actually is. Phase 2
    // integrates r_position_W_(2) open-loop with nothing tying it back to the
    // aircraft, so it can arrive here sitting above it. Inheriting that would
    // give the commit descent a gap to close before it produces any real
    // descent, and it would offset the touchdown check, which reads exactly
    // that gap as its evidence of contact.
    const double reference_drift = r_position_W_(2) - drone_position_W_(2);
    r_position_W_(2) = drone_position_W_(2);

    commit_start_time_ = this->now();
    contact_score_ = 0.0;
    commit_timeout_warned_ = false;
    phase_ = Phase::PHASE_3_COMMIT;
    RCLCPP_INFO(this->get_logger(),
                "Transitioning to Phase 3 (Commit) - altitude=%.3f m, XY error=%.3f m, descending at %.2f m/s "
                "(altitude reference re-anchored, drift was %+.3f m)",
                estimated_position_W_(2), xy_error, commit_descent_rate_, reference_drift);
  }

  void checkPhase3To4Transition() {
    if (!contactHeld()) {
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Touchdown after %.2f s of commit: estimated XY error=%.3f m, altitude=%.3f m "
                "(PX4 land detector: landed=%d ground_contact=%d)",
                (this->now() - commit_start_time_).seconds(),
                estimated_position_W_.head<2>().norm(), estimated_position_W_(2),
                land_detected_landed_, land_detected_ground_contact_);
    enterTouchdownPhase();
  }

  void enterTouchdownPhase() {
    phase_ = Phase::PHASE_4_TOUCHDOWN;
    r_velocity_W_.setZero();
    r_acceleration_W_.setZero();
    disarm_attempts_ = 0;
    disarm_confirmed_ = false;
  }

  // Touchdown evidence: within a few centimetres of the pad, the reference is
  // still commanding a descent and the vehicle has stopped going down.
  //
  // PX4's own land detector deliberately plays no part in this. Both of its
  // stages gate on a low-throttle reading taken from vehicle_thrust_setpoint
  // (MulticopterLandDetector.cpp:219, :268), and in offboard direct-actuator
  // control nothing publishes that topic — the reading sticks, and the detector
  // then latches ground contact on any descent slower than LNDMC_Z_VEL_MAX. In
  // SITL it called contact at 0.71 m in flight. Its flags are still read, but
  // only to decide whether Commander will accept a plain disarm.
  bool contactHeld() {
    // Never call a touchdown before the vehicle has actually left the ground.
    if (!airborne_) {
      contact_score_ = 0.0;
      return false;
    }

    // The primary evidence is the reference running away downwards: it sinks at
    // commit_descent_rate_, so if the vehicle is on the ground the gap between
    // where it is told to be and where it is grows steadily. That signal comes
    // from odometry and our own reference only — no tag, no PX4 state.
    const bool not_following = (drone_position_W_(2) - r_position_W_(2)) > touchdown_ref_gap_;
    const bool descent_stopped = std::abs(drone_velocity_filtered_z_) < touchdown_stall_speed_;
    const bool near_pad = estimated_position_W_(2) < touchdown_max_altitude_;

    // Integrate rather than require an unbroken window: on the ground, rotor wash
    // and airframe vibration put isolated samples outside the velocity band, and
    // a plain hysteresis timer restarts on every one of them.
    if (not_following && descent_stopped && near_pad) {
      contact_score_ = std::min(contact_score_ + dt_, touchdown_stall_time_);
    } else {
      contact_score_ = std::max(contact_score_ - dt_, 0.0);
    }
    return contact_score_ >= touchdown_stall_time_;
  }

  void updatePhase1() {
    // Phase 1: fly to phase_1_target_, which is a point above *the tag*.
    updateTagPosition();

    // The target has to be resolved against the tag, not against odometry.
    // estimated_position_W_ is the vehicle relative to the tag; drone_position_W_
    // is the vehicle relative to wherever PX4 happened to initialise its local
    // frame, which is the takeoff point. Those two agree only when the aircraft
    // launched from the pad -- true of the default synthetic harness and of
    // essentially nothing else. Fly the odometry version anywhere else and the
    // aircraft parks over its own launch point with the tag outside the camera's
    // footprint (~3 m radius at 2.5 m), so the detector sees nothing, the descent
    // gate never opens, and it hovers silently until it drifts away. That is not
    // hypothetical: measured 4.5 m off the pad while the estimate read 0.02 m,
    // because with no detections the estimate is just dead-reckoned odometry.
    // Aim at the tag only while the tag is actually being seen. tag_measurement_
    // received_ latches on the first detection and never clears, so trusting it
    // means that once the tag has been seen once the aircraft keeps steering at
    // a purely dead-reckoned estimate forever. That is a fly-away, not a hold:
    // the estimate drifts, the aircraft chases it, the tag leaves the frame, and
    // nothing pulls it back. A short grace period rides out ordinary dropouts;
    // past that, hold XY and wait for the tag rather than guess where it is.
    Eigen::Vector3d position_error;
    if (tagIsFresh(phase1_tag_max_age_)) {
      position_error = phase_1_target_ - estimated_position_W_;
    } else {
      // No usable bearing on the tag: hold XY where the aircraft is and take only
      // the altitude from the target, which is what gives the tag a chance to
      // enter the frame at all.
      position_error = Eigen::Vector3d(0.0, 0.0, phase_1_target_(2) - drone_position_W_(2));
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "%s; holding XY at the current position and keeping %.1f m altitude "
                           "to search. The approach cannot be aimed until the tag is seen.",
                           tag_measurement_received_
                               ? "Tag lost" : "No tag acquired yet",
                           phase_1_target_(2));
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Phase 1: tag-relative est [%.2f %.2f %.2f], tag age %.2f s, "
                         "XY error to hold point %.2f m",
                         estimated_position_W_(0), estimated_position_W_(1),
                         estimated_position_W_(2), tagAge(),
                         (phase_1_target_ - estimated_position_W_).head<2>().norm());

    // Desired velocity proportional to position error
    Eigen::Vector3d desired_velocity = K_p_ * position_error;

    // Saturate velocity: limit XY norm and Z separately
    saturateXY(desired_velocity, max_velocity_xy_);
    saturateZ(desired_velocity, max_velocity_z_);

    // Store previous velocity for acceleration calculation
    Eigen::Vector3d r_velocity_W_prev = r_velocity_W_;

    // Apply low-pass filter to velocity to avoid command spikes
    r_velocity_W_ = applyLowPassFilter(desired_velocity, r_velocity_W_, lpf_alpha_velocity_);

    // Calculate acceleration from velocity change
    r_acceleration_W_ = (r_velocity_W_ - r_velocity_W_prev) / dt_;

    // Saturate acceleration: limit XY norm and Z separately
    saturateXY(r_acceleration_W_, max_acceleration_xy_);
    saturateZ(r_acceleration_W_, max_acceleration_z_);

    // Integrate setpoint from actual drone position to keep trajectory anchored to reality
    r_position_W_ = drone_position_W_ + r_velocity_W_ * dt_;
  }

  // Descent rate the alignment currently earns: full rate on the axis of the
  // cone, nothing at its edge, nothing at all on a stale estimate. The cone
  // radius narrows with height, so the horizontal error at touchdown is bounded
  // by cone_radius_min_ by construction rather than measured after the fact.
  double coneDescentLimit() {
    const double height = std::max(estimated_position_W_(2), 0.0);
    const double xy_error = estimated_position_W_.head<2>().norm();
    const double cone_radius = cone_slope_ * height + cone_radius_min_;
    const double alignment = std::clamp(1.0 - xy_error / cone_radius, 0.0, 1.0);

    // Stands in for the estimator confidence a covariance would give us.
    if (!tagIsFresh(cone_tag_max_age_)) {
      // Say so. A descent that stops because the tag went stale is otherwise
      // indistinguishable from a vehicle simply hovering, and the aircraft will
      // sit there until something else intervenes. Silence here is what made a
      // vehicle parked 4.5 m off the pad look like a controller problem.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Descent paused: no tag measurement for %.1f s (need < %.1f s). "
                           "Altitude %.2f m, XY %.2f m by the last estimate.",
                           tagAge(), cone_tag_max_age_, estimated_position_W_(2), xy_error);
      return 0.0;
    }
    if (alignment <= 0.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Descent paused: %.2f m off centre, outside the %.2f m cone at "
                           "%.2f m altitude.", xy_error, cone_radius, height);
    }
    return phase2_max_velocity_z_ * alignment;
  }

  void updatePhase2() {
    // Phase 2: Use TF to get tag position and move to phase_2_target_ with controlled descent
    updateTagPosition();

    // Use actual tag position from TF for real-time vision feedback
    Eigen::Vector3d position_error = phase_2_target_ - estimated_position_W_;

    // Desired velocity proportional to position error
    Eigen::Vector3d desired_velocity = K_p_ * position_error;

    // Saturate velocity: limit XY norm and Z separately
    saturateXY(desired_velocity, phase2_max_velocity_xy_);
    saturateZ(desired_velocity, phase2_max_velocity_z_);

    // Descend on a cone, not on a clock: the further off-centre the aircraft is
    // relative to its height, the slower it is allowed to come down.
    desired_velocity(2) = std::max(desired_velocity(2), -coneDescentLimit());

    // Stop descending at the commit altitude: below it the terminal descent
    // takes over, and it only starts once the aircraft is centred.
    if (estimated_position_W_(2) < commit_altitude_) {
      desired_velocity(2) = std::max(desired_velocity(2), 0.0);
    }

    // Store previous velocity for acceleration calculation
    Eigen::Vector3d r_velocity_W_prev = r_velocity_W_;

    Eigen::Vector3d velocity_change = desired_velocity - r_velocity_W_prev;
    saturateXY(velocity_change, max_acceleration_xy_ * dt_);
    saturateZ(velocity_change, max_acceleration_z_ * dt_);
    r_velocity_W_ += velocity_change;

    r_velocity_W_ = applyLowPassFilter(r_velocity_W_, r_velocity_W_prev, lpf_alpha_velocity_);

    // Calculate acceleration from velocity change
    r_acceleration_W_ = (r_velocity_W_ - r_velocity_W_prev) / dt_;

    // Saturate acceleration: limit XY norm and Z separately
    saturateXY(r_acceleration_W_, phase2_max_acceleration_xy_);
    saturateZ(r_acceleration_W_, phase2_max_acceleration_z_);

    // Integrate setpoint from actual tag position to keep trajectory anchored to reality
    r_position_W_ = r_position_W_ + r_velocity_W_ * dt_;
  }

  // Only meaningful inside the commit phase: elsewhere commit_start_time_ is stale.
  bool commitTimedOut() const {
    return phase_ == Phase::PHASE_3_COMMIT &&
           (this->now() - commit_start_time_).seconds() > commit_timeout_;
  }

  void updatePhase3Commit() {
    // Commit: the XY reference is frozen and Z descends at a fixed rate. The tag
    // estimate keeps running for the diagnostics and the touchdown check, but it
    // no longer steers the aircraft.
    updateTagPosition();

    const bool timed_out = commitTimedOut();
    if (timed_out && !commit_timeout_warned_) {
      RCLCPP_ERROR(this->get_logger(),
                   "No touchdown %.1f s after commit (altitude estimate %.3f m); holding altitude.",
                   commit_timeout_, estimated_position_W_(2));
      commit_timeout_warned_ = true;
    }

    // Ramp into the descent rate so the acceleration feedforward stays bounded.
    const double target_velocity_z = timed_out ? 0.0 : -commit_descent_rate_;
    const double max_velocity_step = phase2_max_acceleration_z_ * dt_;
    const double velocity_step = std::clamp(target_velocity_z - r_velocity_W_(2),
                                            -max_velocity_step, max_velocity_step);
    r_velocity_W_(2) += velocity_step;
    r_velocity_W_(0) = 0.0;
    r_velocity_W_(1) = 0.0;
    r_acceleration_W_.setZero();

    r_position_W_(0) = commit_position_xy_(0);
    r_position_W_(1) = commit_position_xy_(1);
    r_position_W_(2) += r_velocity_W_(2) * dt_;
  }

  void updatePhase4Touchdown() {
    // Hold the touchdown reference and disarm. Keep asking until the vehicle
    // reports disarmed: giving up would leave it armed on the pad, which is the
    // failure this phase exists to prevent.
    r_velocity_W_.setZero();
    r_acceleration_W_.setZero();

    if (disarm_confirmed_) {
      return;
    }
    if (vehicle_status_received_ && !vehicle_armed_) {
      disarm_confirmed_ = true;
      RCLCPP_INFO(this->get_logger(), "Disarmed after %d request(s). Landing complete.",
                  disarm_attempts_);
      return;
    }
    if (disarm_attempts_ > 0 && disarm_attempts_ % 10 == 0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Vehicle still armed after %d disarm requests.", disarm_attempts_);
    }
    if (disarm_attempts_ > 0 &&
        (this->now() - last_disarm_request_time_).seconds() < disarm_retry_period_) {
      return;
    }
    sendDisarmCommand();
  }

  void sendDisarmCommand() {
    // Commander denies a plain disarm unless its own land detector reports
    // landed or maybe_landed, which it cannot do in offboard direct-actuator
    // control (see contactHeld()). Where it has not confirmed the landing, force
    // the disarm: 21196 is PX4's magic "skip the checks" value. This is only
    // reached once our own contact check has held, i.e. with the vehicle
    // stationary on the pad.
    const bool px4_confirms_landed = land_detected_landed_ || land_detected_maybe_landed_;

    px4_msgs::msg::VehicleCommand cmd{};
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.param1 = 0.0;  // 0 = disarm
    cmd.param2 = px4_confirms_landed ? 0.0 : 21196.0;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    vehicle_command_pub_->publish(cmd);
    last_disarm_request_time_ = this->now();
    disarm_attempts_++;
    RCLCPP_INFO(this->get_logger(), "%s disarm command published (attempt %d)",
                px4_confirms_landed ? "Plain" : "Forced", disarm_attempts_);
  }

  // Age of the newest tag measurement actually consumed [s]. Large when nothing
  // has arrived yet, so callers can treat "never seen" and "long lost" alike.
  double tagAge() const {
    if (!tag_measurement_received_) {
      return std::numeric_limits<double>::infinity();
    }
    return (this->now() - last_tag_update_time_).seconds();
  }

  // A transform being present in the TF buffer is not the same as the tag being
  // seen: lookupTransform() happily returns the last detection forever, and the
  // detector keeps publishing empty TF messages when it finds nothing. Freshness
  // is the only signal that separates the two.
  bool tagIsFresh(double max_age) const { return tagAge() < max_age; }

  // Vehicle attitude at `time`, interpolated from the odometry history.
  //
  // Returns false when the stamp cannot be resolved against the history, which
  // means the caller must fall back rather than use whatever is nearest. A stamp
  // a few milliseconds past the newest sample is the normal case and clamps to
  // it; a stamp that predates the whole buffer is not a stale measurement to be
  // approximated but a stamp that cannot be trusted, and clamping to the oldest
  // attitude held would quietly resolve every measurement with a second-old
  // rotation. That corrupts the estimate far more than using the latest attitude
  // would, so it is refused here and handled explicitly at the call site.
  bool attitudeAt(const rclcpp::Time& time, Eigen::Quaterniond& orientation) const {
    if (attitude_history_.empty()) {
      return false;
    }
    if (time < attitude_history_.front().time) {
      return false;
    }
    if (time >= attitude_history_.back().time) {
      orientation = attitude_history_.back().orientation;
      return true;
    }
    // Samples arrive in order, so the history is sorted: bracket `time` and slerp.
    const auto after = std::lower_bound(
        attitude_history_.begin(), attitude_history_.end(), time,
        [](const AttitudeSample& sample, const rclcpp::Time& t) { return sample.time < t; });
    const AttitudeSample& before = *(after - 1);
    const double span = (after->time - before.time).seconds();
    const double ratio = span > 0.0 ? (time - before.time).seconds() / span : 0.0;
    orientation = before.orientation.slerp(ratio, after->orientation);
    return true;
  }

  static double wrapToPi(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
  }

  // In-plane platform yaw — the one rotational quantity two concentric tags do
  // observe well, because it is a rotation within the marker plane and so is not
  // touched by the planar ambiguity that spoils the out-of-plane tilt. Recovered
  // by taking the tag orientation into world axes through the same EKF attitude
  // the position now uses, then filtered hard: the platform is static, so
  // anything fast in this signal is noise.
  //
  // Nothing steers on it yet — the published setpoint yaw is still identity. This
  // is the measurement the yaw-alignment work (roadmap item 13) will slew
  // towards, and in the meantime it is a free end-to-end check on the rest of the
  // chain: with the tag at the world origin it should read 0.
  void updatePlatformYaw(const Eigen::Matrix3d& R_W_B) {
    const Eigen::Matrix3d R_W_plat = R_W_B * R_b_cam_ * R_plat_cam_;
    platform_yaw_raw_ = std::atan2(R_W_plat(1, 0), R_W_plat(0, 0));

    if (!platform_yaw_initialized_) {
      platform_yaw_filtered_ = platform_yaw_raw_;
      platform_yaw_initialized_ = true;
      return;
    }
    // Filter the wrapped difference so the estimate does not take the long way
    // round when the raw yaw crosses +/-pi.
    platform_yaw_filtered_ = wrapToPi(
        platform_yaw_filtered_ +
        lpf_alpha_platform_yaw_ * wrapToPi(platform_yaw_raw_ - platform_yaw_filtered_));
  }

  bool updateTagPosition() {
    try {
      // Get transformation from camera_link to platform
      // Use tf2::TimePointZero to get the latest available transform
      auto transform = tf_buffer_->lookupTransform(
          camera_frame_id_,           // Target frame (camera)
          platform_frame_id_,         // Source frame (platform)
          tf2::TimePointZero         // Get latest transform (not a specific time)
      );

      // Extract translation: position of platform in camera frame
      r_plat_cam_cam_ <<
          transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z;

      // Extract rotation: quaternion -> rotation matrix
      Eigen::Quaterniond quat(
          transform.transform.rotation.w,
          transform.transform.rotation.x,
          transform.transform.rotation.y,
          transform.transform.rotation.z
      );
      R_plat_cam_ = quat.toRotationMatrix();

      // Pair the measurement with the vehicle attitude at its own timestamp.
      const rclcpp::Time measurement_stamp(transform.header.stamp,
                                           this->get_clock()->get_clock_type());
      Eigen::Quaterniond orientation_at_measurement;
      if (!attitudeAt(measurement_stamp, orientation_at_measurement)) {
        // No usable stamp, so pair with the newest attitude instead. That is the
        // naive version of this fusion: it gives up latency compensation and
        // nothing else, which is a far smaller error than resolving the tag with
        // a second-old rotation. The estimate stays usable and the descent keeps
        // working; only the item 3 refinement is lost.
        if (attitude_history_.empty()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Tag transform available but no vehicle attitude yet; "
                               "skipping the update.");
          return false;
        }
        orientation_at_measurement = attitude_history_.back().orientation;

        // Distinguish the two ways this happens, because the fixes differ. An age
        // of years means the stamp is not on our clock at all: ros_gz_image copies
        // the Gazebo header, so the detector's transforms carry simulator time
        // while this node runs on wall time. Recovering the interpolation needs
        // /clock bridged from Gazebo *and* this node on use_sim_time -- setting
        // use_sim_time alone leaves the node waiting on a clock nobody publishes,
        // which stalls every timer here and is worse than this fallback.
        const double measurement_age = (this->now() - measurement_stamp).seconds();
        if (std::abs(measurement_age) > kImplausibleMeasurementAge) {
          // Say this once, not on a timer: a clock-domain mismatch is a fixed
          // property of how the stack was launched, so repeating it only buries
          // the tf2 TF_OLD_DATA warnings that share its root cause.
          RCLCPP_WARN_ONCE(this->get_logger(),
                           "Tag stamp is not on this node's clock (apparent age %.0f s): the "
                           "transform is stamped on the simulator clock while this node uses "
                           "wall time. Pairing with the latest attitude for the rest of this "
                           "run, which costs only the latency compensation. To restore it, "
                           "bridge /clock and run this node with use_sim_time -- neither alone "
                           "is enough. If tf2 is also logging TF_OLD_DATA for the platform "
                           "frame, something else is publishing it on a second clock (usually "
                           "a leftover fake_tag_tf.py); run tools/tf_clock_check.py.",
                           measurement_age);
        } else {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "Tag measurement is %.2f s old, beyond the %.1f s attitude "
                               "history; pairing with the latest attitude.",
                               measurement_age, attitude_history_length_);
        }
      }
      const Eigen::Matrix3d R_W_B = orientation_at_measurement.toRotationMatrix();

      // Resolve the tag translation into world axes with the EKF attitude rather
      // than with the PnP rotation:
      //
      //   position_W_raw_ = -R_W_B * (R_B_C * r_plat_cam_cam_ + r_cam_b_b_)
      //
      // Rotating through R_plat_cam_ (as this did) is algebraically identical
      // whenever the PnP rotation is exact, and it carried the vehicle attitude
      // implicitly, so it needed nothing from the EKF. But it multiplies the
      // translation by the least reliable output of monocular tag pose. With two
      // concentric markers the solve has no baseline, so its out-of-plane tilt is
      // where the planar ambiguity lives: it can wobble or flip by degrees between
      // frames while the translation stays put, and at 2 m every degree is ~3.5 cm
      // of position noise injected into an otherwise clean measurement. The EKF
      // attitude is IMU-driven and an order of magnitude better. See §02 of the
      // review artifact linked from CLAUDE.md.
      const Eigen::Vector3d r_plat_b_b = R_b_cam_ * r_plat_cam_cam_ + r_cam_b_b_;
      position_W_raw_ = -(R_W_B * r_plat_b_b);

      // Apply low-pass filter to position to avoid jumps
      position_W_filtered_ = applyLowPassFilter(position_W_raw_, position_W_filtered_, lpf_alpha_);
      position_W_ = position_W_filtered_;

      // Update fused position estimate: apply correction only on NEW tag measurement
      // (not every 100Hz loop, but only when TF timestamp changes ~15Hz)
      tf2::TimePoint measurement_time = tf2_ros::fromMsg(transform.header.stamp);
      if (measurement_time > last_tag_measurement_time_) {
        last_tag_measurement_time_ = measurement_time;
        last_tag_update_time_ = this->now();
        tag_measurement_received_ = true;
        correctFusedPositionWithTag(position_W_);
        updatePlatformYaw(R_W_B);
      }

      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG(this->get_logger(), "Transform lookup failed from '%s' to '%s': %s",
                   platform_frame_id_.c_str(), camera_frame_id_.c_str(), ex.what());
      return false;
    }
  }

  void publishTrajectoryPoint() {
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint traj_point;
    traj_point.time_from_start.sec = 0;
    traj_point.time_from_start.nanosec = 0;

    // Resize vectors for single DOF
    traj_point.transforms.resize(1);
    traj_point.velocities.resize(1);
    traj_point.accelerations.resize(1);

    // Position (reference trajectory)
    traj_point.transforms[0].translation.x = r_position_W_(0);
    traj_point.transforms[0].translation.y = r_position_W_(1);
    traj_point.transforms[0].translation.z = r_position_W_(2);

    // Orientation: identity quaternion (yaw = 0)
    traj_point.transforms[0].rotation.x = 0.0;
    traj_point.transforms[0].rotation.y = 0.0;
    traj_point.transforms[0].rotation.z = 0.0;
    traj_point.transforms[0].rotation.w = 1.0;

    // Velocity
    traj_point.velocities[0].linear.x = r_velocity_W_(0);
    traj_point.velocities[0].linear.y = r_velocity_W_(1);
    traj_point.velocities[0].linear.z = r_velocity_W_(2);
    traj_point.velocities[0].angular.x = 0.0;
    traj_point.velocities[0].angular.y = 0.0;
    traj_point.velocities[0].angular.z = 0.0;

    // Acceleration
    traj_point.accelerations[0].linear.x = r_acceleration_W_(0);
    traj_point.accelerations[0].linear.y = r_acceleration_W_(1);
    traj_point.accelerations[0].linear.z = r_acceleration_W_(2);
    traj_point.accelerations[0].angular.x = 0.0;
    traj_point.accelerations[0].angular.y = 0.0;
    traj_point.accelerations[0].angular.z = 0.0;

    trajectory_publisher_->publish(traj_point);
  }

  void publishPlatformPosition() {
    // Fused tag-relative position of the vehicle (100 Hz odometry prediction,
    // corrected at the ~15 Hz measurement rate). Telemetry only: nothing
    // subscribes to it. The controller closes on command/trajectory against its
    // own odometry, and the tag reaches it solely through that setpoint -- the
    // outer loop lives in this node. (An older comment here claimed
    // controller_node consumed this via setActualPosition(); no such subscriber
    // or method exists, and describing a loop that is not wired invites someone
    // to debug the wrong side of it.)
    geometry_msgs::msg::Vector3 pos_msg;
    pos_msg.x = estimated_position_W_(0);
    pos_msg.y = estimated_position_W_(1);
    pos_msg.z = estimated_position_W_(2);
    platform_position_pub_->publish(pos_msg);
  }

  void publishDiagnostics() {
    // Publish odometry with orientation and angular velocity from odometry callback
    diagnostics_->publishOdometry(groundtruth_position_W_ - drone_position_W_, drone_velocity_W_,
                                 drone_orientation_W_, drone_angular_velocity_W_,
                                 last_odometry_timestamp_);

    // Publish groundtruth with odometry timestamp for synchronization
    diagnostics_->publishGroundtruth(groundtruth_position_W_, groundtruth_velocity_W_, last_odometry_timestamp_);

    // Publish current phase
    diagnostics_->publishPhase(static_cast<int>(phase_), last_odometry_timestamp_);

    // Publish estimated position (fused estimate)
    diagnostics_->publishEstimatedPosition(groundtruth_position_W_ - estimated_position_W_, last_odometry_timestamp_);

    // Publish raw tag position
    diagnostics_->publishPositionRaw(groundtruth_position_W_ - position_W_filtered_, last_odometry_timestamp_);

    // Publish the in-plane platform yaw taken from the tag
    diagnostics_->publishPlatformYaw(platform_yaw_filtered_, platform_yaw_raw_, last_odometry_timestamp_);
  }

  void landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
    land_detected_landed_ = msg->landed;
    land_detected_maybe_landed_ = msg->maybe_landed;
    land_detected_ground_contact_ = msg->ground_contact;
  }

  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    vehicle_status_received_ = true;
    vehicle_armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    offboard_active_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
  }

  void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg) {
    // Extract position with proper coordinate frame conversion
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;

    px4_frames::eigenOdometryFromPX4Msg(odom_msg, position, orientation, velocity, angular_velocity);

    // Always update current drone state for later use
    Eigen::Vector3d delta_position = position - drone_position_W_;
    if (trajectory_initialized_) {
        estimated_position_W_ += delta_position;
    }

    drone_position_W_ = position;
    drone_velocity_W_ = velocity;
    // Low-passed for the contact check, which must not restart on single samples.
    drone_velocity_filtered_z_ = applyLowPassFilterScalar(velocity(2), drone_velocity_filtered_z_,
                                                          lpf_alpha_odometry_vz_);
    drone_orientation_W_ = orientation;
    drone_angular_velocity_W_ = angular_velocity;
    last_odometry_timestamp_ = odom_msg->timestamp;

    // Retain a short attitude history so a tag measurement can be resolved with
    // the attitude at its own stamp rather than the newest one available.
    const rclcpp::Time arrival_time = this->now();
    attitude_history_.push_back({arrival_time, orientation});
    while (!attitude_history_.empty() &&
           (arrival_time - attitude_history_.front().time).seconds() > attitude_history_length_) {
      attitude_history_.pop_front();
    }

    // Initialize trajectory reference to current drone position on first message
    if (!trajectory_initialized_) {
      r_position_W_ = position;
      r_velocity_W_.setZero();
      r_acceleration_W_.setZero();

      // Initialize fused position estimate from odometry
      estimated_position_W_ = position;

      initialization_time_ = std::chrono::high_resolution_clock::now();
      trajectory_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Landing trajectory initialized at position [%.2f, %.2f, %.2f]",
                  r_position_W_(0), r_position_W_(1), r_position_W_(2));
      RCLCPP_INFO(this->get_logger(), "Fused position estimate initialized from odometry");
      RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds before starting Phase 1 trajectory...",
                  initialization_delay_seconds_);
    }
  }

  void groundtruthCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr gt_msg) {
    // Convert groundtruth position and velocity from NED to ENU
    Eigen::Vector3d gt_position(gt_msg->x, gt_msg->y, gt_msg->z);
    Eigen::Vector3d gt_velocity(gt_msg->vx, gt_msg->vy, gt_msg->vz);

    groundtruth_position_W_ = px4_frames::rotateVectorFromToENU_NED(gt_position);
    groundtruth_velocity_W_ = px4_frames::rotateVectorFromToENU_NED(gt_velocity);
  }
};

#endif  // LANDING_TRAJECTORY_BASE_H
