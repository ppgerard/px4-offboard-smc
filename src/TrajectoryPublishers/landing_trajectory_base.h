#ifndef LANDING_TRAJECTORY_BASE_H
#define LANDING_TRAJECTORY_BASE_H

// Shared logic for AprilTag-based landing trajectory nodes: TF lookup of the
// platform tag, a simple position fusion filter (odometry prediction +
// tag correction), and the 4-phase approach/descent/commit/touchdown state
// machine.
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
#include <memory>
#include <cmath>
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
    descent_stall_start_ = now;
    last_disarm_request_time_ = now;
    last_tag_update_time_ = now;

    // Initialize drone state with defaults
    drone_orientation_W_ = Eigen::Quaterniond::Identity();
    drone_angular_velocity_W_.setZero();

    // Initialize rotation matrices
    // R_b_cam: rotation from camera to body (-90° around z, then 180° around x)
    Eigen::AngleAxisd rot_z(-M_PI / 2.0, Eigen::Vector3d::UnitZ());
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

  // Control parameters
  const double K_p_ = 1.0;              // Position gain
  const double dt_ = px4_offboard::kControlPeriodSeconds;  // Time step (seconds)
  const double xy_error_threshold_ = 0.3; // 30cm threshold for XY error
  const double tag_visibility_min_time_ = 0.5; // 0.5s minimum tag visibility
  const double xy_error_min_time_ = 0.5;      // 0.5s minimum XY error below threshold
  // Terminal descent (Phase 2 -> commit -> touchdown). Altitudes are the fused
  // tag-relative altitude of the body origin, so they include the vehicle's own
  // ground clearance: the T2 rests on the pad at ~0.105 m in this frame.
  const double airborne_altitude_ = 1.0;        // above this the vehicle has certainly left the pad [m]
  const double commit_altitude_ = 0.20;         // commit below this altitude (~0.10 m clearance) [m]
  const double commit_xy_error_max_ = 0.10;     // XY alignment required to commit [m]
  const double commit_tag_max_age_ = 0.5;       // tag measurement must be fresher than this [s]
  const double commit_wait_timeout_ = 5.0;      // commit regardless after waiting this long [s]
  const double commit_descent_rate_ = 0.4;      // fixed descent rate once committed [m/s]
  const double commit_timeout_ = 8.0;           // no touchdown by then: stop descending [s]
  const double touchdown_max_altitude_ = 0.15;  // contact only ever declared this close to the pad [m]
  const double touchdown_min_descent_cmd_ = 0.05;  // reference must be commanding down [m/s]
  const double touchdown_stall_speed_ = 0.05;   // descent counts as stalled below this [m/s]
  const double touchdown_stall_time_ = 1.5;     // stalled this long = contact [s]
  const double disarm_retry_period_ = 0.5;      // resend DISARM this often [s]
  const int disarm_max_attempts_ = 10;
  // Phase 1 limits (separate XY and Z)
  const double max_velocity_xy_ = 1.0;     // Maximum velocity XY [m/s] (Phase 1)
  const double max_velocity_z_ = 2.0;      // Maximum velocity Z [m/s] (Phase 1)
  const double max_acceleration_xy_ = 2.5; // Maximum acceleration XY [m/s²] (Phase 1)
  const double max_acceleration_z_ = 1.0;  // Maximum acceleration Z [m/s²] (Phase 1)
  // Phase 2 limits (separate XY and Z)
  const double phase2_max_velocity_xy_ = 0.5;     // Maximum velocity XY in Phase 2 [m/s]
  const double phase2_max_velocity_z_ = 0.3;      // Maximum velocity Z in Phase 2 [m/s]
  const double phase2_max_acceleration_xy_ = 0.8; // Maximum acceleration XY in Phase 2 [m/s²]
  const double phase2_max_acceleration_z_ = 0.3;  // Maximum acceleration Z in Phase 2 [m/s²]
  const double phase2_descent_rate_z_ = 0.3;   // Constant descent rate in Z during Phase 2 [m/s]
  const double lpf_alpha_ = 1;        // Low-pass filter coefficient for AprilTag pose [0, 1]
  const double lpf_alpha_velocity_ = 0.2; // Low-pass filter coefficient for velocity [0, 1]
  // Phase 1 setpoint
  const Eigen::Vector3d phase_1_target_{0.0, 0.0, 3.0};

  // Phase 2 target (in world frame, tag at origin)
  const Eigen::Vector3d phase_2_target_{0.0, 0.0, 0.0};

  // Hysteresis timer for transition Phase 1 -> Phase 2
  std::chrono::high_resolution_clock::time_point phase2_transition_start_time_;
  bool phase2_transition_flag_ = false;

  // Vehicle-specific frame IDs and camera mounting offset (loaded from ROS params)
  std::string camera_frame_id_;
  std::string platform_frame_id_;
  std::string world_frame_id_;
  Eigen::Vector3d r_cam_b_b_;  // position of camera in body frame, expressed in body frame

  // Transformation matrices (constant)
  // R_plat_world: transforms from world to platform frame (identity since tag is at origin)
  Eigen::Matrix3d R_plat_world_ = Eigen::Matrix3d::Identity();

  // R_b_cam: fixed transformation from camera to body (-90° around z, then 180° around x)
  Eigen::Matrix3d R_b_cam_;

  // Transformation state (updated from TF)
  Eigen::Matrix3d R_plat_cam_;          // Rotation: platform w.r.t. camera (from TF)
  Eigen::Vector3d r_plat_cam_cam_;      // Position: platform in camera frame (from TF)
  Eigen::Vector3d position_W_filtered_; // Filtered platform position in world frame

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
  rclcpp::Time descent_stall_start_;             // debounce for the stalled-descent check
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

  // Groundtruth state for diagnostics
  px4_msgs::msg::VehicleLocalPosition groundtruth_msg_;
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

    // Publish platform position feedback (from TF) while the vision loop is closed
    if (phase_ == Phase::PHASE_2 || phase_ == Phase::PHASE_3_COMMIT) {
      publishPlatformPosition();
    }

    // Publish diagnostic topics
    publishDiagnostics();
  }

  void checkPhase1To2Transition() {
    // Check XY error against actual drone position (ignore Z)
    Eigen::Vector2d xy_error_2d(phase_1_target_(0) - drone_position_W_(0),
                                 phase_1_target_(1) - drone_position_W_(1));
    double xy_error = xy_error_2d.norm();

    // Check if tag is visible
    bool tag_visible = tf_buffer_->canTransform(
        camera_frame_id_, platform_frame_id_, tf2::TimePointZero, tf2::durationFromSec(0.01));

    // All three must hold to start the hysteresis timer. The airborne check
    // matters because this transition ignores Z: a node started while the
    // vehicle sits disarmed on the pad would otherwise run straight through
    // Phase 2 into the terminal descent without ever having flown.
    if (xy_error < xy_error_threshold_ && tag_visible && airborne_) {
      if (!phase2_transition_flag_) {
        phase2_transition_start_time_ = std::chrono::high_resolution_clock::now();
        phase2_transition_flag_ = true;
      }
      // Check if conditions have been true long enough
      auto duration = std::chrono::high_resolution_clock::now() - phase2_transition_start_time_;
      if (std::chrono::duration<double>(duration).count() >= xy_error_min_time_) {
        RCLCPP_INFO(this->get_logger(), "Transitioning to Phase 2 (XY error=%.3f m)", xy_error);
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
    const bool tag_fresh = tag_measurement_received_ &&
        (this->now() - last_tag_update_time_).seconds() < commit_tag_max_age_;
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
    commit_start_time_ = this->now();
    descent_stall_start_ = commit_start_time_;
    commit_timeout_warned_ = false;
    phase_ = Phase::PHASE_3_COMMIT;
    RCLCPP_INFO(this->get_logger(),
                "Transitioning to Phase 3 (Commit) - altitude=%.3f m, XY error=%.3f m, descending at %.2f m/s",
                estimated_position_W_(2), xy_error, commit_descent_rate_);
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
    const rclcpp::Time now = this->now();

    // Never call a touchdown before the vehicle has actually left the ground.
    if (!airborne_) {
      descent_stall_start_ = now;
      return false;
    }

    // Past the commit timeout the reference no longer commands a descent, so the
    // timeout itself stands in for the command: it means the same thing, that
    // the vehicle was told to go down and did not.
    const bool descent_commanded =
        r_velocity_W_(2) <= -touchdown_min_descent_cmd_ || commitTimedOut();
    const bool descent_stopped = std::abs(drone_velocity_W_(2)) < touchdown_stall_speed_;
    const bool near_pad = estimated_position_W_(2) < touchdown_max_altitude_;
    if (!(descent_commanded && descent_stopped && near_pad)) {
      descent_stall_start_ = now;
      return false;
    }
    return (now - descent_stall_start_).seconds() >= touchdown_stall_time_;
  }

  void updatePhase1() {
    // Phase 1: Move to phase_1_target_ with proportional velocity control
    // Use actual drone position from odometry for real-time feedback
    updateTagPosition();

    Eigen::Vector3d position_error = phase_1_target_ - drone_position_W_;

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
    // Hold the touchdown reference and disarm. PX4 can reject the request while
    // its own land detector is still settling, so retry a few times.
    r_velocity_W_.setZero();
    r_acceleration_W_.setZero();

    if (disarm_confirmed_) {
      return;
    }
    if (vehicle_status_received_ && !vehicle_armed_) {
      disarm_confirmed_ = true;
      RCLCPP_INFO(this->get_logger(), "Disarmed. Landing complete.");
      return;
    }
    if (disarm_attempts_ >= disarm_max_attempts_) {
      RCLCPP_ERROR_ONCE(this->get_logger(),
                        "Vehicle still armed after %d disarm requests.", disarm_max_attempts_);
      return;
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

      // Apply coordinate transformation:
      // r_b_world_world = -R_plat_world^T * R_plat_cam * (r_plat_cam_cam + R_b_cam^T * r_cam_b_b)
      Eigen::Vector3d r_plat_b_cam = r_plat_cam_cam_ + R_b_cam_.transpose() * r_cam_b_b_;
      Eigen::Vector3d r_plat_b_plat = R_plat_cam_.transpose() * r_plat_b_cam;
      position_W_raw_ = -R_plat_world_.transpose() * r_plat_b_plat;

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
    // Publish fused position estimate (100Hz prediction + 15Hz correction from tag)
    // This is the feedback that controller_node will use via setActualPosition()
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
  }

  void landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
    land_detected_landed_ = msg->landed;
    land_detected_maybe_landed_ = msg->maybe_landed;
    land_detected_ground_contact_ = msg->ground_contact;
  }

  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    vehicle_status_received_ = true;
    vehicle_armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
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
    drone_orientation_W_ = orientation;
    drone_angular_velocity_W_ = angular_velocity;
    last_odometry_timestamp_ = odom_msg->timestamp;

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
    // Store groundtruth message for diagnostics
    groundtruth_msg_ = *gt_msg;

    // Convert groundtruth position and velocity from NED to ENU
    Eigen::Vector3d gt_position(gt_msg->x, gt_msg->y, gt_msg->z);
    Eigen::Vector3d gt_velocity(gt_msg->vx, gt_msg->vy, gt_msg->vz);

    groundtruth_position_W_ = px4_frames::rotateVectorFromToENU_NED(gt_position);
    groundtruth_velocity_W_ = px4_frames::rotateVectorFromToENU_NED(gt_velocity);
  }
};

#endif  // LANDING_TRAJECTORY_BASE_H
