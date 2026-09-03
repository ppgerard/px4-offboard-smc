/****************************************************************************
 *
 *   Copyright (c) 2023, SMART Research Group, Saxion University of 
 *   Applied Sciences.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "px4_offboard_lowlevel/controller_node.h"

#include <algorithm>

namespace {
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kTiltRateLimitRadPerStep =
    90.0 * kDegToRad * px4_offboard::kControlPeriodSeconds;  // 90°/s

// Tricopter (t2) arm geometry [m]. Shared by the allocation matrix and the
// tau_z -> front-tilt computation, which must stay consistent with each other.
// Rotor arms measured from the CENTRE OF MASS -- which is what makes them moment
// arms, and the whole point: the resultant thrust passes through wherever the
// allocation believes the CG to be, so if that point is not the real CG, hovering
// costs a CONSTANT moment.
//
// These are the gz model's actual rotor positions, and they are correct only
// because model.sdf now places the composite CG on the model origin (see the
// comment on base_link's inertial pose there). PX4's own mixer already assumed
// exactly that -- CA_ROTOR0_PX 0.20, CA_ROTOR2_PX -0.4 are the rotor link
// positions -- so model, PX4 and this node now agree with each other.
//
// The old values (0.1815 / 0.4185) put this node's moment reference 18.5 mm
// forward of the origin while the model's CG sat at +8.09 mm, and PX4's sat at 0.
// Measured in still-air hover under both control laws: tau_y = +0.2275 N.m with
// the old arms, -0.2284 N.m with these arms against the OLD model, and
// -0.0319 N.m with these arms against the corrected model -- all three matching
// 24.4 N times the offset to within 15%.
constexpr double kTricopterArm1X = 0.20;
constexpr double kTricopterArm1Y = 0.215;
constexpr double kTricopterArm2X = 0.20;
constexpr double kTricopterArm2Y = 0.215;
constexpr double kTricopterArm3X = 0.40;
constexpr double kTricopterArm1Z = 0.0;
constexpr double kTricopterArm2Z = 0.0;

// The tilt travel is a PROPERTY OF THE AIRFRAME, not a constant of this code, and
// SITL and the real T2 disagree about it: the gz airframe sets
// CA_SV_TL0/TL1_MINA -7, the vehicle's own parameters say -5. It has to match in
// every place that maps an angle to a normalized servo command, or a commanded
// tilt is not the angle the joint takes -- the disagreement that once cost a
// 4 deg pitch bias.
//
// Note these bounds do NOT come from CA_SV_TL*_MINA/MAXA at runtime. In offboard
// direct-actuator mode PX4's allocator is bypassed entirely, so the value written
// to actuator_servos goes straight to the output scaling: -1 maps to
// PWM_MAIN_MIN<n> and +1 to PWM_MAIN_MAX<n>. What these must agree with is the
// PHYSICAL angle the servo reaches at those two PWM values, which on the vehicle
// is set by its per-output travel calibration (MIN 1150 / MAX 2025 on one tilt,
// 1000 / 1875 on the other). Measure it with an inclinometer on the bench; do not
// assume the CA_ parameters describe it.
double servoNormToTiltRad(double norm, double tilt_min_deg, double tilt_max_deg)
{
    const double clamped_norm = std::clamp(norm, -1.0, 1.0);
    const double tilt_deg = tilt_min_deg + 0.5 * (clamped_norm + 1.0) * (tilt_max_deg - tilt_min_deg);
    return tilt_deg * kDegToRad;
}

double tiltRadToServoNorm(double tilt_rad, double tilt_min_deg, double tilt_max_deg)
{
    const double tilt_deg = std::clamp(tilt_rad * kRadToDeg, tilt_min_deg, tilt_max_deg);
    const double norm = 2.0 * (tilt_deg - tilt_min_deg) / (tilt_max_deg - tilt_min_deg) - 1.0;
    return std::clamp(norm, -1.0, 1.0);
}
}



ControllerNode::ControllerNode()
    : Node("controller_node")
    {
        this->declare_parameter<std::string>("controller_type", "smc");
        controller_type_ = this->get_parameter("controller_type").as_string();
        if (controller_type_ == "stsmc") {
            controller_ = std::make_unique<StSmcController>();
        } else {
            if (controller_type_ != "smc") {
                RCLCPP_WARN(this->get_logger(),
                    "Unknown controller_type '%s', falling back to 'smc'.", controller_type_.c_str());
                controller_type_ = "smc";
            }
            controller_ = std::make_unique<SmcController>();
        }

        loadParams();
        compute_ControlAllocation_and_ActuatorEffect_matrices();

        // Defining the compatible ROS 2 predefined QoS for PX4 topics
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        // Subscribers
        vehicle_odometry_sub_= this->create_subscription<px4_msgs::msg::VehicleOdometry>
            (odometry_topic_, qos, std::bind(&ControllerNode::vehicle_odometryCallback, this, _1));
        vehicle_status_sub_= this->create_subscription<px4_msgs::msg::VehicleStatus>
            (status_topic_, qos, std::bind(&ControllerNode::vehicleStatusCallback, this, _1));
        command_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
            (command_pose_topic_, 10, std::bind(&ControllerNode::commandPoseCallback, this, _1));
        command_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>
            (command_traj_topic_, 10, std::bind(&ControllerNode::commandTrajectoryCallback, this, _1));
        servos_status_sub_ = this->create_subscription<px4_msgs::msg::ActuatorServos>
            (servos_status_topic_, qos, std::bind(&ControllerNode::servosStatusCallback, this, _1));

        // Publishers
        actuator_motors_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>
            (actuator_control_topic_, 10);
        actuator_servos_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorServos>
            (servos_control_topic_, 10);
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>
            (offboard_control_topic_, 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>
            (vehicle_command_topic_, 10);
        wrench_publisher_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>
            ("/landing/wrench", 10);
        f_ext_publisher_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>
            ("/landing/f_ext", 10);
        attitude_reference_publisher_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>
            ("/landing/attitude_reference", 10);
        // Defaults ON, like every other diagnostic here. The switch exists so
        // the diagnostic's OWN cost can be measured: this node publishes four
        // diagnostic topics per 100 Hz cycle, and the chattering metrics are
        // sensitive to loop jitter, so "does instrumenting it change it" is a
        // question the rig has to be able to answer about itself.
        this->declare_parameter("diagnostics.publish_sliding_surface", true);
        publish_sliding_surface_ =
            this->get_parameter("diagnostics.publish_sliding_surface").as_bool();
        if (publish_sliding_surface_) {
            sliding_surface_publisher_ =
            this->create_publisher<geometry_msgs::msg::WrenchStamped>
                ("/landing/sliding_surface", 10);
        }

        // Timers
        std::chrono::duration<double> offboard_period(0.33);
        std::chrono::duration<double> controller_period(px4_offboard::kControlPeriodSeconds);
        offboardTimer = this->create_wall_timer(offboard_period, [=]() {publishOffboardControlModeMsg();});
        controllerTimer = this->create_wall_timer(controller_period, [=]() {updateControllerOutput();});
    }

void ControllerNode::loadParams() {
    // UAV Parameters
    this->declare_parameter("uav_parameters.mass", 0.0);
    this->declare_parameter("uav_parameters.arm_length", 0.0);
    this->declare_parameter("uav_parameters.num_of_arms", 4);
    this->declare_parameter("uav_parameters.moment_constant", 0.0);
    this->declare_parameter("uav_parameters.thrust_constant", 0.0);
    this->declare_parameter("uav_parameters.gravity", 0.0);
    this->declare_parameter("uav_parameters.pitch_trim_deg", 0.0);
    this->declare_parameter("uav_parameters.PWM_MIN", 0);
    this->declare_parameter("uav_parameters.PWM_MAX", 0);
    this->declare_parameter("uav_parameters.SIM_GZ_EC_MAX", 0);
    this->declare_parameter("uav_parameters.SIM_GZ_EC_MIN", 0);
    this->declare_parameter("uav_parameters.inertia.ixx", 0.0);
    this->declare_parameter("uav_parameters.inertia.iyy", 0.0);
    this->declare_parameter("uav_parameters.inertia.izz", 0.0);
    this->declare_parameter("uav_parameters.inertia.ixy", 0.0);
    this->declare_parameter("uav_parameters.inertia.ixz", 0.0);
    this->declare_parameter("uav_parameters.inertia.iyz", 0.0);
    this->declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_2", 0.0);
    this->declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_1", 0.0);
    this->declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_0", 0.0);
    this->declare_parameter("uav_parameters.tilt_min_deg", -7.0);
    this->declare_parameter("uav_parameters.tilt_max_deg", 90.0);
    this->declare_parameter("uav_parameters.tilt_yaw_sign", 1.0);
    this->declare_parameter("uav_parameters.tilt_1_servo_index", 4);
    this->declare_parameter("uav_parameters.tilt_2_servo_index", 5);

    double _uav_mass = this->get_parameter("uav_parameters.mass").as_double();
    _arm_length = this->get_parameter("uav_parameters.arm_length").as_double();
    _num_of_arms = this->get_parameter("uav_parameters.num_of_arms").as_int();
    _moment_constant = this->get_parameter("uav_parameters.moment_constant").as_double();
    _thrust_constant = this->get_parameter("uav_parameters.thrust_constant").as_double();
    double _gravity = this->get_parameter("uav_parameters.gravity").as_double();
    // Fixed pitch trim compensating a vehicle's natural mounting tilt (e.g. the
    // t2 tiltrotor). Zero by default so airframes that don't need it (e.g.
    // x500) are unaffected.
    double _pitch_trim_rad = this->get_parameter("uav_parameters.pitch_trim_deg").as_double() * M_PI / 180.0;
    _PWM_MIN = this->get_parameter("uav_parameters.PWM_MIN").as_int();
    _PWM_MAX = this->get_parameter("uav_parameters.PWM_MAX").as_int();
    _SIM_GZ_EC_MAX = this->get_parameter("uav_parameters.SIM_GZ_EC_MAX").as_int();
    _SIM_GZ_EC_MIN = this->get_parameter("uav_parameters.SIM_GZ_EC_MIN").as_int();
    double _inertia_ixx = this->get_parameter("uav_parameters.inertia.ixx").as_double();
    double _inertia_iyy = this->get_parameter("uav_parameters.inertia.iyy").as_double();
    double _inertia_izz = this->get_parameter("uav_parameters.inertia.izz").as_double();
    double _inertia_ixy = this->get_parameter("uav_parameters.inertia.ixy").as_double();
    double _inertia_ixz = this->get_parameter("uav_parameters.inertia.ixz").as_double();
    double _inertia_iyz = this->get_parameter("uav_parameters.inertia.iyz").as_double();
    tilt_yaw_sign_ = this->get_parameter("uav_parameters.tilt_yaw_sign").as_double() < 0.0 ? -1.0 : 1.0;
    tilt_min_deg_ = this->get_parameter("uav_parameters.tilt_min_deg").as_double();
    tilt_max_deg_ = this->get_parameter("uav_parameters.tilt_max_deg").as_double();
    tilt_1_servo_index_ = this->get_parameter("uav_parameters.tilt_1_servo_index").as_int();
    tilt_2_servo_index_ = this->get_parameter("uav_parameters.tilt_2_servo_index").as_int();
    // ActuatorServos carries 8 channels; an out-of-range index would read or
    // write past the array, and on the write side it would silently leave a tilt
    // servo commanded to zero while the other tracked -- an asymmetric tilt is
    // the yaw axis on this airframe, so it is not a small error.
    if (tilt_1_servo_index_ < 0 || tilt_1_servo_index_ > 7 ||
        tilt_2_servo_index_ < 0 || tilt_2_servo_index_ > 7 ||
        tilt_1_servo_index_ == tilt_2_servo_index_) {
        RCLCPP_ERROR(this->get_logger(),
            "Invalid tilt servo indices (%d, %d): must be distinct and within [0, 7]. "
            "Falling back to 4 and 5.", tilt_1_servo_index_, tilt_2_servo_index_);
        tilt_1_servo_index_ = 4;
        tilt_2_servo_index_ = 5;
    }
    // Announce the tilt configuration at startup. Not decoration: the yaw sign
    // is invisible in flight until the aircraft spins, and a yaml that says -1
    // proves nothing if the BINARY predates the parameter. This line is the only
    // evidence that the value reached the code -- it cost a bench test to learn.
    RCLCPP_INFO(this->get_logger(),
        "Tilt config: yaw_sign %+.0f, travel [%.1f, %.1f] deg, servo indices %d and %d.",
        tilt_yaw_sign_, tilt_min_deg_, tilt_max_deg_,
        tilt_1_servo_index_, tilt_2_servo_index_);
    if (tilt_yaw_sign_ < 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "tilt_yaw_sign is NEGATIVE: differential tilt is inverted relative to the "
            "simulator's geometry. This is correct for the real T2 and wrong for gz.");
    }

    if (!(tilt_max_deg_ > tilt_min_deg_)) {
        RCLCPP_ERROR(this->get_logger(),
            "uav_parameters.tilt_max_deg (%.1f) must exceed tilt_min_deg (%.1f); "
            "falling back to [-7, 90].", tilt_max_deg_, tilt_min_deg_);
        tilt_min_deg_ = -7.0;
        tilt_max_deg_ = 90.0;
    }

    double _omega_to_pwm_coefficient_x_2 = this->get_parameter("uav_parameters.omega_to_pwm_coefficient.x_2").as_double();
    double _omega_to_pwm_coefficient_x_1 = this->get_parameter("uav_parameters.omega_to_pwm_coefficient.x_1").as_double();
    double _omega_to_pwm_coefficient_x_0 = this->get_parameter("uav_parameters.omega_to_pwm_coefficient.x_0").as_double();
    Eigen::Matrix3d _inertia_matrix;
    _inertia_matrix << _inertia_ixx, _inertia_ixy, _inertia_ixz,
                       _inertia_ixy, _inertia_iyy, _inertia_iyz,
                       _inertia_ixz, _inertia_iyz, _inertia_izz;
    _omega_to_pwm_coefficients << _omega_to_pwm_coefficient_x_2, _omega_to_pwm_coefficient_x_1, _omega_to_pwm_coefficient_x_0;
    
    // Topics Names
    this->declare_parameter("topics_names.command_pose_topic", "default");
    this->declare_parameter("topics_names.command_traj_topic", "default");
    this->declare_parameter("topics_names.odometry_topic", "default");
    this->declare_parameter("topics_names.status_topic", "default");
    this->declare_parameter("topics_names.offboard_control_topic", "default");
    this->declare_parameter("topics_names.vehicle_command_topic", "default");
    this->declare_parameter("topics_names.actuator_control_topic", "default");
    this->declare_parameter("topics_names.servos_control_topic", "default");
    this->declare_parameter("topics_names.servos_status_topic", "default");

    command_pose_topic_ = this->get_parameter("topics_names.command_pose_topic").as_string();
    command_traj_topic_ = this->get_parameter("topics_names.command_traj_topic").as_string();
    odometry_topic_ = this->get_parameter("topics_names.odometry_topic").as_string();
    status_topic_ = this->get_parameter("topics_names.status_topic").as_string();
    offboard_control_topic_ = this->get_parameter("topics_names.offboard_control_topic").as_string();
    vehicle_command_topic_ = this->get_parameter("topics_names.vehicle_command_topic").as_string();
    actuator_control_topic_ = this->get_parameter("topics_names.actuator_control_topic").as_string();
    servos_control_topic_ = this->get_parameter("topics_names.servos_control_topic").as_string();
    servos_status_topic_ = this->get_parameter("topics_names.servos_status_topic").as_string();
    
    // Load logic switches
    this->declare_parameter("sitl_mode", true);

    in_sitl_mode_ = this->get_parameter("sitl_mode").as_bool();
    
    // pass UAV parameters to the controller (shared by every control law)
    controller_->setUavMass(_uav_mass);
    controller_->setInertiaMatrix(_inertia_matrix);
    controller_->setGravity(_gravity);
    controller_->setPitchTrim(_pitch_trim_rad);

    // Shared by both control laws, so it is read here rather than inside either
    // branch below -- the stsmc path returns early. Zero disables the observer,
    // which is what makes it A/B-able against one binary.
    this->declare_parameter("control_gains.f_ext_observer_gain", 0.0);
    const double f_ext_gain = this->get_parameter("control_gains.f_ext_observer_gain").as_double();
    controller_->setExternalForceGain(f_ext_gain);
    RCLCPP_INFO(this->get_logger(), "External-force observer (item 8): %s, gain %.2f rad/s",
                f_ext_gain > 0.0 ? "ON" : "OFF", f_ext_gain);

    // Also shared by both laws: the low-pass on omega_ref. Zero disables it.
    this->declare_parameter("control_gains.omega_ref_filter_hz", 0.0);
    const double omega_ref_hz = this->get_parameter("control_gains.omega_ref_filter_hz").as_double();
    controller_->setReferenceRateFilterHz(omega_ref_hz);
    RCLCPP_INFO(this->get_logger(), "Reference-rate filter on omega_ref: %s, %.2f Hz",
                omega_ref_hz > 0.0 ? "ON" : "OFF", omega_ref_hz);

    if (controller_type_ == "stsmc") {
        // ===== STSMC CONTROLLER GAINS (fully independent from the SMC gains below) =====
        this->declare_parameter("control_gains.STA_Lambda_x", 0.0);
        this->declare_parameter("control_gains.STA_Lambda_y", 0.0);
        this->declare_parameter("control_gains.STA_Lambda_z", 0.0);
        this->declare_parameter("control_gains.STA_K1_x", 0.0);
        this->declare_parameter("control_gains.STA_K1_y", 0.0);
        this->declare_parameter("control_gains.STA_K1_z", 0.0);
        this->declare_parameter("control_gains.STA_K2_x", 0.0);
        this->declare_parameter("control_gains.STA_K2_y", 0.0);
        this->declare_parameter("control_gains.STA_K2_z", 0.0);
        this->declare_parameter("control_gains.STA_Lambda_R_x", 0.0);
        this->declare_parameter("control_gains.STA_Lambda_R_y", 0.0);
        this->declare_parameter("control_gains.STA_Lambda_R_z", 0.0);
        this->declare_parameter("control_gains.STA_K1_R_x", 0.0);
        this->declare_parameter("control_gains.STA_K1_R_y", 0.0);
        this->declare_parameter("control_gains.STA_K1_R_z", 0.0);
        this->declare_parameter("control_gains.STA_K2_R_x", 0.0);
        this->declare_parameter("control_gains.STA_K2_R_y", 0.0);
        this->declare_parameter("control_gains.STA_K2_R_z", 0.0);

        Eigen::Vector3d lambda, k1, k2, lambda_r, k1_r, k2_r;

        lambda << this->get_parameter("control_gains.STA_Lambda_x").as_double(),
                  this->get_parameter("control_gains.STA_Lambda_y").as_double(),
                  this->get_parameter("control_gains.STA_Lambda_z").as_double();

        k1 << this->get_parameter("control_gains.STA_K1_x").as_double(),
              this->get_parameter("control_gains.STA_K1_y").as_double(),
              this->get_parameter("control_gains.STA_K1_z").as_double();

        k2 << this->get_parameter("control_gains.STA_K2_x").as_double(),
              this->get_parameter("control_gains.STA_K2_y").as_double(),
              this->get_parameter("control_gains.STA_K2_z").as_double();

        lambda_r << this->get_parameter("control_gains.STA_Lambda_R_x").as_double(),
                    this->get_parameter("control_gains.STA_Lambda_R_y").as_double(),
                    this->get_parameter("control_gains.STA_Lambda_R_z").as_double();

        k1_r << this->get_parameter("control_gains.STA_K1_R_x").as_double(),
                this->get_parameter("control_gains.STA_K1_R_y").as_double(),
                this->get_parameter("control_gains.STA_K1_R_z").as_double();

        k2_r << this->get_parameter("control_gains.STA_K2_R_x").as_double(),
                this->get_parameter("control_gains.STA_K2_R_y").as_double(),
                this->get_parameter("control_gains.STA_K2_R_z").as_double();

        RCLCPP_INFO(this->get_logger(), "===== STSMC PARAMETERS LOADED =====");
        RCLCPP_INFO(this->get_logger(), "Lambda:   [%.2f, %.2f, %.2f]", lambda(0), lambda(1), lambda(2));
        RCLCPP_INFO(this->get_logger(), "K1:       [%.2f, %.2f, %.2f]", k1(0), k1(1), k1(2));
        RCLCPP_INFO(this->get_logger(), "K2:       [%.2f, %.2f, %.2f]", k2(0), k2(1), k2(2));
        RCLCPP_INFO(this->get_logger(), "Lambda_R: [%.2f, %.2f, %.2f]", lambda_r(0), lambda_r(1), lambda_r(2));
        RCLCPP_INFO(this->get_logger(), "K1_R:     [%.2f, %.2f, %.2f]", k1_r(0), k1_r(1), k1_r(2));
        RCLCPP_INFO(this->get_logger(), "K2_R:     [%.2f, %.2f, %.2f]", k2_r(0), k2_r(1), k2_r(2));
        RCLCPP_INFO(this->get_logger(), "==================================");

        auto* stsmc_controller = static_cast<StSmcController*>(controller_.get());
        stsmc_controller->setLambda(lambda);
        stsmc_controller->setLambdaR(lambda_r);
        stsmc_controller->setK1(k1);
        stsmc_controller->setK2(k2);
        stsmc_controller->setK1R(k1_r);
        stsmc_controller->setK2R(k2_r);
        return;
    }

    // ===== SMC CONTROLLER GAINS =====
    // Translational sliding mode control parameters
    this->declare_parameter("control_gains.Lambda_x", 0.0);
    this->declare_parameter("control_gains.Lambda_y", 0.0);
    this->declare_parameter("control_gains.Lambda_z", 0.0);
    this->declare_parameter("control_gains.K_s_x", 0.0);
    this->declare_parameter("control_gains.K_s_y", 0.0);
    this->declare_parameter("control_gains.K_s_z", 0.0);
    this->declare_parameter("control_gains.Phi_x", 0.0);
    this->declare_parameter("control_gains.Phi_y", 0.0);
    this->declare_parameter("control_gains.Phi_z", 0.0);
    // Rotational sliding mode control parameters
    this->declare_parameter("control_gains.Lambda_R_x", 0.0);
    this->declare_parameter("control_gains.Lambda_R_y", 0.0);
    this->declare_parameter("control_gains.Lambda_R_z", 0.0);
    this->declare_parameter("control_gains.K_s_R_x", 0.0);
    this->declare_parameter("control_gains.K_s_R_y", 0.0);
    this->declare_parameter("control_gains.K_s_R_z", 0.0);
    this->declare_parameter("control_gains.Phi_R_x", 0.0);
    this->declare_parameter("control_gains.Phi_R_y", 0.0);
    this->declare_parameter("control_gains.Phi_R_z", 0.0);

    Eigen::Vector3d lambda, k_s, phi, lambda_r, k_s_r, phi_r;

    lambda << this->get_parameter("control_gains.Lambda_x").as_double(),
              this->get_parameter("control_gains.Lambda_y").as_double(),
              this->get_parameter("control_gains.Lambda_z").as_double();

    k_s << this->get_parameter("control_gains.K_s_x").as_double(),
           this->get_parameter("control_gains.K_s_y").as_double(),
           this->get_parameter("control_gains.K_s_z").as_double();

    phi << this->get_parameter("control_gains.Phi_x").as_double(),
           this->get_parameter("control_gains.Phi_y").as_double(),
           this->get_parameter("control_gains.Phi_z").as_double();

    lambda_r << this->get_parameter("control_gains.Lambda_R_x").as_double(),
                this->get_parameter("control_gains.Lambda_R_y").as_double(),
                this->get_parameter("control_gains.Lambda_R_z").as_double();

    k_s_r << this->get_parameter("control_gains.K_s_R_x").as_double(),
             this->get_parameter("control_gains.K_s_R_y").as_double(),
             this->get_parameter("control_gains.K_s_R_z").as_double();

    phi_r << this->get_parameter("control_gains.Phi_R_x").as_double(),
             this->get_parameter("control_gains.Phi_R_y").as_double(),
             this->get_parameter("control_gains.Phi_R_z").as_double();

    // Debug: Print loaded SMC parameters
    RCLCPP_INFO(this->get_logger(), "===== SMC PARAMETERS LOADED =====");
    RCLCPP_INFO(this->get_logger(), "Lambda: [%.2f, %.2f, %.2f]", lambda(0), lambda(1), lambda(2));
    RCLCPP_INFO(this->get_logger(), "K_s:    [%.2f, %.2f, %.2f]", k_s(0), k_s(1), k_s(2));
    RCLCPP_INFO(this->get_logger(), "Phi:    [%.2f, %.2f, %.2f]", phi(0), phi(1), phi(2));
    RCLCPP_INFO(this->get_logger(), "Lambda_R: [%.2f, %.2f, %.2f]", lambda_r(0), lambda_r(1), lambda_r(2));
    RCLCPP_INFO(this->get_logger(), "K_s_R:    [%.2f, %.2f, %.2f]", k_s_r(0), k_s_r(1), k_s_r(2));
    RCLCPP_INFO(this->get_logger(), "Phi_R:    [%.2f, %.2f, %.2f]", phi_r(0), phi_r(1), phi_r(2));
    RCLCPP_INFO(this->get_logger(), "==================================");

    // Gains below are specific to the SMC control law.
    auto* smc_controller = static_cast<SmcController*>(controller_.get());
    smc_controller->setLambda(lambda);
    smc_controller->setKs(k_s);
    smc_controller->setPhi(phi);
    smc_controller->setLambdaR(lambda_r);
    smc_controller->setKsR(k_s_r);
    smc_controller->setPhiR(phi_r);
}

void ControllerNode::compute_ControlAllocation_and_ActuatorEffect_matrices(double tilt_1_rad, double tilt_2_rad) {
    Eigen::MatrixXd rotor_velocities_to_torques_and_thrust;

    if (_num_of_arms == 3) {
        // 3x3 reduced allocation for tricopter tilt-rotor: [tau_x, tau_y, thrust].
        // Tau_z is intentionally handled by a separate loop.
        rotor_velocities_to_torques_and_thrust.resize(3, 3);

        const double l_1_x = kTricopterArm1X;
        const double l_1_y = kTricopterArm1Y;
        const double l_2_x = kTricopterArm2X;
        const double l_2_y = kTricopterArm2Y;
        const double l_3_x = kTricopterArm3X;
        const double l_1_z = kTricopterArm1Z;
        const double l_2_z = kTricopterArm2Z;

        const double c_1 = std::cos(tilt_1_rad);
        const double c_2 = std::cos(tilt_2_rad);
        const double s_1 = std::sin(tilt_1_rad);
        const double s_2 = std::sin(tilt_2_rad);

        rotor_velocities_to_torques_and_thrust <<
            -c_1 * _thrust_constant * l_1_y - _moment_constant * _thrust_constant * s_1,  c_2 * _thrust_constant * l_2_y + _moment_constant * _thrust_constant * s_2, 0.0,
            -c_1 * _thrust_constant * l_1_x + s_1 * _thrust_constant * l_1_z, -c_2 * _thrust_constant * l_2_x + s_2 * _thrust_constant * l_2_z, _thrust_constant * l_3_x,
            c_1 * _thrust_constant, c_2 * _thrust_constant, _thrust_constant;
        
        // rotor_velocities_to_torques_and_thrust <<
        //     -_thrust_constant * l_1_y,  _thrust_constant * l_2_y, 0.0,
        //     -_thrust_constant * l_1_x, -_thrust_constant * l_2_x, _thrust_constant * l_3_x,
        //     _thrust_constant, _thrust_constant, _thrust_constant;


        torques_and_thrust_to_rotor_velocities_.resize(3, 3);
        torques_and_thrust_to_rotor_velocities_ =
            rotor_velocities_to_torques_and_thrust.completeOrthogonalDecomposition().pseudoInverse();
        return;
    }

    if (_num_of_arms == 4) {
        const double kDegToRad = M_PI / 180.0;
        const double kS = std::sin(45 * kDegToRad);
        rotor_velocities_to_torques_and_thrust.resize(4, 4);
        rotor_velocities_to_torques_and_thrust << -kS, kS, kS, -kS,
                                                   -kS, kS, -kS, kS,
                                                   -1, -1, 1, 1,
                                                    1, 1, 1, 1;
        Eigen::Vector4d k;
        k << _thrust_constant * _arm_length,
             _thrust_constant * _arm_length,
             _moment_constant * _thrust_constant,
             _thrust_constant;
        rotor_velocities_to_torques_and_thrust = k.asDiagonal() * rotor_velocities_to_torques_and_thrust;

        torques_and_thrust_to_rotor_velocities_.resize(4, 4);
        torques_and_thrust_to_rotor_velocities_ =
            rotor_velocities_to_torques_and_thrust.completeOrthogonalDecomposition().pseudoInverse();
        return;
    }

    RCLCPP_ERROR(this->get_logger(),
        "Unsupported uav_parameters.num_of_arms = %d (supported: 3, 4). Control allocation not computed.",
        _num_of_arms);
}

// Control allocation: wrench -> rotor speeds [rad/s]. For the tricopter this
// also computes the commanded front-tilt from the desired tau_z. Shared by both
// the hardware (PWM) and SITL mappings below; returns false and leaves *omega
// empty when the airframe is not supported.
bool ControllerNode::computeRotorVelocities(const Eigen::VectorXd &wrench, Eigen::VectorXd *omega) {
    // Every clamp below means the airframe was asked for more than it has, and
    // the delivered wrench is no longer the commanded one. Recorded so the
    // control law can hold its integral state while that is true.
    allocation_saturated_ = false;

    if (_num_of_arms == 3) {
        Eigen::Vector3d reduced_wrench;
        reduced_wrench << wrench(0), wrench(1), wrench(3);
        Eigen::Vector3d omega_sq = torques_and_thrust_to_rotor_velocities_ * reduced_wrench;

        // Safety: clamp negative values to zero
        for (int i = 0; i < omega_sq.size(); i++){
            if (omega_sq[i] <= 0){
                allocation_saturated_ = allocation_saturated_ || omega_sq[i] < 0.0;
                omega_sq[i] = 0.0;
            }
        }

        // Compute front-tilt from desired tau_z:
        // tau_z_0 = moment_constant * thrust_constant * (-omega1^2 + omega2^2 - omega3^2)
        // tilt = (tau_z_desired - tau_z_0) / (thrust_constant*(l1y*omega1^2 + l2y*omega2^2))
        const double tau_z_desired = wrench(2);
        const double tau_z_0 = _moment_constant * _thrust_constant * (-omega_sq[0] + omega_sq[1] - omega_sq[2]);
        const double denom = _thrust_constant * (kTricopterArm1Y * omega_sq[0] + kTricopterArm2Y * omega_sq[1]);
        // tilt_yaw_sign_ is +1 in the simulator and -1 on the real T2, and that is
        // MEASURED rather than assumed. The relationship it carries is whether a
        // positive commanded tilt_1 (with tilt_2 = -tilt_1) produces a positive or
        // a negative yaw moment, and it depends on which physical nacelle each
        // servo channel drives and which way that servo rotates -- neither of which
        // this code can see.
        //
        // Get it wrong and the yaw loop is POSITIVE feedback. Measured on the first
        // hardware flight (log_20, 2026-09-03): within 0.2 s of offboard entry the
        // aircraft was at -28 deg/s and accelerating, both tilt servos hit their
        // +/-6.5 deg clamps by 2 s, and it reached -292 deg/s having turned 585 deg
        // before the pilot took it back. Over the unsaturated samples the
        // correlation between commanded differential tilt and yaw ACCELERATION was
        // -0.882: the law asked for one sign and the airframe delivered the other.
        //
        // Roll and pitch stayed bounded throughout (-3.8 to +8 deg), which is what
        // rules out a mirrored motor assignment -- that would have inverted roll
        // too. The fault is specific to the tilt -> yaw path.
        //
        // The simulator cannot find this. Both gz tilt joints share <xyz>0 1 0</xyz>
        // and the model's rotor ordering matches this code's convention, so +1 is
        // self-consistent there and every SITL landing in CLAUDE.md flew with it.
        double computed_tilt_rad = 0.0;
        if (std::abs(denom) > 1e-9) {
            computed_tilt_rad = tilt_yaw_sign_ * (tau_z_desired - tau_z_0) / denom;
        }

        // Keep physical angles in radians internally; convert to normalized only when publishing.
        const double tilt_1_desired = std::clamp(computed_tilt_rad, tilt_min_deg_ * kDegToRad, -tilt_min_deg_ * kDegToRad);
        allocation_saturated_ = allocation_saturated_ || tilt_1_desired != computed_tilt_rad;

        // Rate limiting
        const double delta_desired = tilt_1_desired - tilt_1_prev_;
        const double delta = std::clamp(delta_desired, -kTiltRateLimitRadPerStep, kTiltRateLimitRadPerStep);
        allocation_saturated_ = allocation_saturated_ || delta != delta_desired;
        tilt_1_rad_ = tilt_1_prev_ + delta;
        tilt_1_prev_ = tilt_1_rad_;
        tilt_2_rad_ = -tilt_1_rad_;

        *omega = omega_sq.cwiseSqrt();
        return true;
    }

    if (_num_of_arms == 4) {
        Eigen::VectorXd omega_sq = torques_and_thrust_to_rotor_velocities_ * wrench;
        // Safety: clamp negative values to zero (prevents NaN from sqrt of negative)
        for (int i = 0; i < omega_sq.size(); i++){
            if (omega_sq[i] <= 0){
                allocation_saturated_ = allocation_saturated_ || omega_sq[i] < 0.0;
                omega_sq[i] = 0.0;
            }
        }
        *omega = omega_sq.cwiseSqrt();
        return true;
    }

    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Unsupported uav_parameters.num_of_arms = %d (supported: 3, 4). Not commanding actuators.",
        _num_of_arms);
    omega->resize(0);
    return false;
}

void ControllerNode::px4Inverse
    (Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench) {
    Eigen::VectorXd omega;
    if (!computeRotorVelocities(*wrench, &omega)) {
        throttles->resize(0);
        return;
    }

    // Map rotor speed to PWM with the identified rotor curve, then normalize.
    const Eigen::VectorXd ones_temp = Eigen::VectorXd::Ones(omega.size());
    const Eigen::VectorXd pwm =
        (_omega_to_pwm_coefficients(0) * omega.cwiseProduct(omega)) + (_omega_to_pwm_coefficients(1) * omega) +
        (_omega_to_pwm_coefficients(2) * ones_temp);
    *throttles = (pwm - (_PWM_MIN * ones_temp));
    *throttles /= (_PWM_MAX - _PWM_MIN);
}

void ControllerNode::px4InverseSITL
    (Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench) {
    Eigen::VectorXd omega;
    if (!computeRotorVelocities(*wrench, &omega)) {
        throttles->resize(0);
        return;
    }

    // Gazebo takes the rotor speed normalized over the engine-control range.
    const Eigen::VectorXd ones_temp = Eigen::VectorXd::Ones(omega.size());
    *throttles = (omega - (_SIM_GZ_EC_MIN * ones_temp));
    *throttles /= (_SIM_GZ_EC_MAX - _SIM_GZ_EC_MIN);
}

void ControllerNode::arm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void ControllerNode::disarm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void ControllerNode::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	// vehicle_command_publisher_->publish(msg);
}

void ControllerNode::publishOffboardControlModeMsg()
{
	px4_msgs::msg::OffboardControlMode offboard_msg{};
	offboard_msg.position = false;
	offboard_msg.velocity = false;
	offboard_msg.acceleration = false;
	offboard_msg.body_rate = false;
	offboard_msg.attitude = false;
	offboard_msg.thrust_and_torque = false;
	offboard_msg.direct_actuator = true;
	offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(offboard_msg);
    RCLCPP_INFO_ONCE(get_logger(),"Offboard enabled");
}

void ControllerNode::commandPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {                   // When a command is received
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    eigenTrajectoryPointFromPoseMsg(pose_msg, position, orientation);
    RCLCPP_INFO_ONCE(get_logger(),"Controller got first command message.");
    controller_->setTrajectoryPoint(position, orientation);          // Send the command to controller_ obj
}

void ControllerNode::commandTrajectoryCallback(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint &msg) {                   // When a command is received
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

    // A point with no transform carries no setpoint: hold the previous one
    // rather than commanding whatever the unpacked vectors happen to contain.
    if (!eigenTrajectoryPointFromMsg(msg, position, orientation, velocity, angular_velocity, acceleration)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                             "Ignoring trajectory point with no transform.");
        return;
    }

    controller_->setTrajectoryPoint(position, velocity, acceleration, orientation, angular_velocity);
    RCLCPP_INFO_ONCE(get_logger(),"Controller got first command message.");
}

void ControllerNode::vehicle_odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg){
        //  Debug message
        RCLCPP_INFO_ONCE(get_logger(),"Controller got first odometry message.");
        
        last_odometry_timestamp_ = odom_msg->timestamp;

        Eigen::Vector3d position;
        Eigen::Vector3d velocity; 
        Eigen::Quaterniond orientation;
        Eigen::Vector3d angular_velocity;
        
        px4_frames::eigenOdometryFromPX4Msg(odom_msg,
                                            position, orientation, velocity, angular_velocity);

        controller_->setOdometry(position, orientation, velocity, angular_velocity);
        odometry_received_ = true;
}

void ControllerNode::servosStatusCallback(const px4_msgs::msg::ActuatorServos::SharedPtr servos_msg) {
    // Which servo channels carry the tilts is airframe-dependent, and SITL and
    // the real T2 differ. PX4 assigns servo functions CONTROL SURFACES FIRST,
    // THEN TILTS (ActuatorEffectivenessTiltrotorVTOL::getEffectivenessMatrix), so
    // the index is CA_SV_CS_COUNT. The gz airframe has 4 control surfaces, giving
    // tilts on Servo 5/6 (indices 4/5); the vehicle has CA_SV_CS_COUNT 3, giving
    // tilts on Servo 4/5 (indices 3/4) -- confirmed by its per-output travel
    // calibration sitting on exactly those two channels.

    if (_num_of_arms != 3) {
        return;
    }

    // Read measured servo normalized values (these are measurements, not commands)
    const double measured_tilt_1_norm = std::clamp(static_cast<double>(servos_msg->control[tilt_1_servo_index_]), -1.0, 1.0);
    const double measured_tilt_2_norm = std::clamp(static_cast<double>(servos_msg->control[tilt_2_servo_index_]), -1.0, 1.0);
    // Convert measurements to radians
    const double measured_tilt_1_rad = servoNormToTiltRad(measured_tilt_1_norm, tilt_min_deg_, tilt_max_deg_);
    const double measured_tilt_2_rad = servoNormToTiltRad(measured_tilt_2_norm, tilt_min_deg_, tilt_max_deg_);
    // store measured tilts separately from commanded tilts
    measured_tilt_1_rad_ = measured_tilt_1_rad;
    measured_tilt_2_rad_ = measured_tilt_2_rad;

    compute_ControlAllocation_and_ActuatorEffect_matrices(measured_tilt_1_rad_, measured_tilt_2_rad_);
}

void ControllerNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg){
    const bool was_offboard =
        current_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    current_status_ = *status_msg;
    const bool is_offboard =
        current_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    // The control law runs from node start but only publishes in offboard, so
    // at engagement it would otherwise carry whatever its integral state wound
    // up to while the vehicle sat on the pad waiting to be handed control.
    if (is_offboard && !was_offboard) {
        controller_->reset();
        RCLCPP_INFO(this->get_logger(), "Offboard engaged: controller state reset.");
    }
}

void ControllerNode::publishActuatorMotorsMsg(const Eigen::VectorXd& throttles) {
    // Lockstep should be disabled from PX4 and from the model.sdf file
    // direct motor throttles control
    // Prepare msg
    px4_msgs::msg::ActuatorMotors actuator_motors_msg;
    auto safe_throttle = [](double v){
        if (!std::isfinite(v)) return 0.0f;
        return (float)std::clamp(v, 0.0, 1.0);
    };
    if (throttles.size() == 3) {
        actuator_motors_msg.control = { safe_throttle(throttles[0]), safe_throttle(throttles[1]), safe_throttle(throttles[2]), std::nanf("1"),
                                        std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                                        std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1") };
    } else {
        actuator_motors_msg.control = { safe_throttle(throttles[0]), safe_throttle(throttles[1]), safe_throttle(throttles[2]), safe_throttle(throttles[3]),
                                        std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                                        std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1") };
    }
	actuator_motors_msg.reversible_flags = 0;
	actuator_motors_msg.timestamp = this->now().nanoseconds() / 1000;
	actuator_motors_msg.timestamp_sample = actuator_motors_msg.timestamp;

	actuator_motors_publisher_->publish(actuator_motors_msg);
}

void ControllerNode::publishActuatorServosMsg(double tilt_1_rad, double tilt_2_rad) {
    px4_msgs::msg::ActuatorServos actuator_servos_msg;

    const double tilt_1_norm = tiltRadToServoNorm(tilt_1_rad, tilt_min_deg_, tilt_max_deg_);
    const double tilt_2_norm = tiltRadToServoNorm(tilt_2_rad, tilt_min_deg_, tilt_max_deg_);

    auto safe_servo = [](double v){
        if (!std::isfinite(v)) return 0.0f;
        return (float)std::clamp(v, -1.0, 1.0);
    };
    // Non-tilt channels are held at 0.0 (centred control surfaces) and the two
    // unused tail channels at NaN, exactly as before -- only WHICH channels carry
    // the tilts is now configurable. With the SITL indices (4, 5) this produces a
    // byte-identical message to the version that flew every result in CLAUDE.md,
    // which is what lets the hardware change be adopted without re-validating the
    // simulator numbers.
    actuator_servos_msg.control = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                    std::nanf("1"), std::nanf("1") };
    actuator_servos_msg.control[tilt_1_servo_index_] = safe_servo(tilt_1_norm);
    actuator_servos_msg.control[tilt_2_servo_index_] = safe_servo(tilt_2_norm);
    actuator_servos_msg.timestamp = this->now().nanoseconds() / 1000;
    actuator_servos_publisher_->publish(actuator_servos_msg);
}
void ControllerNode::publishWrenchMsg(const Eigen::VectorXd& wrench, uint64_t timestamp){
    if (wrench.size() < 4) {
        return;
    }

    geometry_msgs::msg::WrenchStamped msg;

    msg.header.stamp = rclcpp::Time(timestamp * 1000);
    msg.header.frame_id = "world";

    msg.wrench.torque.x = wrench(0);
    msg.wrench.torque.y = wrench(1);
    msg.wrench.torque.z = wrench(2);

    msg.wrench.force.x = 0.0;
    msg.wrench.force.y = 0.0;
    msg.wrench.force.z = wrench(3);

    wrench_publisher_->publish(msg);

    // Same stamp, same frame: the disturbance the law was working against when
    // it produced the wrench above.
    geometry_msgs::msg::WrenchStamped f_ext_msg;
    f_ext_msg.header = msg.header;
    const Eigen::Vector3d f_ext = controller_->externalForceEstimate();
    f_ext_msg.wrench.force.x = f_ext(0);
    f_ext_msg.wrench.force.y = f_ext(1);
    f_ext_msg.wrench.force.z = f_ext(2);
    f_ext_publisher_->publish(f_ext_msg);

    // Same stamp again: the desired-attitude derivative and the force command it
    // was taken from. Diagnostic only -- see referenceAngularVelocity().
    geometry_msgs::msg::WrenchStamped att_msg;
    att_msg.header = msg.header;
    const Eigen::Vector3d omega_ref = controller_->referenceAngularVelocity();
    const Eigen::Vector3d i_a_d = controller_->desiredAcceleration();
    att_msg.wrench.torque.x = omega_ref(0);
    att_msg.wrench.torque.y = omega_ref(1);
    att_msg.wrench.torque.z = omega_ref(2);
    att_msg.wrench.force.x = i_a_d(0);
    att_msg.wrench.force.y = i_a_d(1);
    att_msg.wrench.force.z = i_a_d(2);
    attitude_reference_publisher_->publish(att_msg);

    // Same stamp again: the two sliding surfaces, which say which chattering
    // mechanism the law is actually operating in. torque = s_R [rad/s],
    // force = s [m/s]. See ControllerBase::slidingSurface().
    if (!publish_sliding_surface_) {
        return;
    }
    geometry_msgs::msg::WrenchStamped s_msg;
    s_msg.header = msg.header;
    const Eigen::Vector3d s_trans = controller_->slidingSurface();
    const Eigen::Vector3d s_rot = controller_->rotationalSlidingSurface();
    s_msg.wrench.torque.x = s_rot(0);
    s_msg.wrench.torque.y = s_rot(1);
    s_msg.wrench.torque.z = s_rot(2);
    s_msg.wrench.force.x = s_trans(0);
    s_msg.wrench.force.y = s_trans(1);
    s_msg.wrench.force.z = s_trans(2);
    sliding_surface_publisher_->publish(s_msg);
}

void ControllerNode::updateControllerOutput() {
    // Nothing to track until the vehicle state is known: evaluating the control
    // law before the first odometry message would act on the initial state
    // rather than on the vehicle's.
    if (!odometry_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Waiting for the first odometry message on '%s' before running the controller.",
            odometry_topic_.c_str());
        return;
    }

    //  calculate controller output
    Eigen::VectorXd controller_output;
    Eigen::Quaterniond desired_quaternion;
    controller_->calculateControllerOutput(&controller_output, &desired_quaternion);
    publishWrenchMsg(controller_output, last_odometry_timestamp_);
    
    // Debug: Log the controller output (only once per second to avoid spam)
    // if (iteration_count_++ % 100 == 0) {
    //     RCLCPP_INFO(this->get_logger(), "Controller output [tau_x, tau_y, tau_z, thrust]: [%.3f, %.3f, %.3f, %.3f]",
    //                 controller_output(0), controller_output(1), controller_output(2), controller_output(3));
    //     RCLCPP_INFO(this->get_logger(), "Vehicle nav_state: %d (Offboard=%d)", 
    //                 current_status_.nav_state, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    // }
    
    Eigen::VectorXd throttles;
    if (in_sitl_mode_) px4InverseSITL(&throttles, &controller_output);
    else px4Inverse(&throttles, &controller_output);

    if (throttles.size() == 0) {
        return;  // unsupported airframe, already reported by computeRotorVelocities()
    }

    // Anti-windup: tell the law whether the wrench it just asked for survived
    // allocation. Where it did not, the loop is open at the actuators, and
    // integrating an error they were never able to answer is what winds the
    // super-twisting state up. Takes effect on the next cycle, 10 ms later.
    const bool throttle_saturated = !throttles.allFinite() ||
        (throttles.array() < 0.0).any() || (throttles.array() > 1.0).any();
    controller_->setActuatorsSaturated(allocation_saturated_ || throttle_saturated);

    // Publish the controller output
    if (current_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        publishActuatorMotorsMsg(throttles);
        if (_num_of_arms == 3) {
            publishActuatorServosMsg(tilt_1_rad_, tilt_2_rad_);
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ControllerNode>());

    rclcpp::shutdown();

    return 0;
}