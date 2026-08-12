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
constexpr double kTiltMinDeg = -7.0;
constexpr double kTiltMaxDeg = 90.0;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kTiltRateLimitRadPerStep = 90.0 * kDegToRad * 0.01;  // 90°/s at 100Hz

double servoNormToTiltRad(double norm)
{
    const double clamped_norm = std::clamp(norm, -1.0, 1.0);
    const double tilt_deg = kTiltMinDeg + 0.5 * (clamped_norm + 1.0) * (kTiltMaxDeg - kTiltMinDeg);
    return tilt_deg * kDegToRad;
}

double tiltRadToServoNorm(double tilt_rad)
{
    const double tilt_deg = std::clamp(tilt_rad * kRadToDeg, kTiltMinDeg, kTiltMaxDeg);
    const double norm = 2.0 * (tilt_deg - kTiltMinDeg) / (kTiltMaxDeg - kTiltMinDeg) - 1.0;
    return std::clamp(norm, -1.0, 1.0);
}
}



ControllerNode::ControllerNode() 
    : Node("controller_node")
    {
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
        // platform_position_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>
        //     ("landing/platform_position", 10, std::bind(&ControllerNode::platformPositionCallback, this, _1));

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

        // Timers
        std::chrono::duration<double> offboard_period(0.33);        
        std::chrono::duration<double> controller_period(0.01);        
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
    this->declare_parameter("uav_parameters.max_rotor_speed", 0);
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

    double _uav_mass = this->get_parameter("uav_parameters.mass").as_double();
    _arm_length = this->get_parameter("uav_parameters.arm_length").as_double();
    _num_of_arms = this->get_parameter("uav_parameters.num_of_arms").as_int();
    _moment_constant = this->get_parameter("uav_parameters.moment_constant").as_double();
    _thrust_constant = this->get_parameter("uav_parameters.thrust_constant").as_double();
    _max_rotor_speed = this->get_parameter("uav_parameters.max_rotor_speed").as_int();
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
    this->declare_parameter("topics_names.battery_status_topic", "default");
    this->declare_parameter("topics_names.actuator_status_topic", "default");
    this->declare_parameter("topics_names.offboard_control_topic", "default");
    this->declare_parameter("topics_names.vehicle_command_topic", "default");
    this->declare_parameter("topics_names.actuator_control_topic", "default");
    this->declare_parameter("topics_names.servos_control_topic", "default");
    this->declare_parameter("topics_names.servos_status_topic", "default");

    command_pose_topic_ = this->get_parameter("topics_names.command_pose_topic").as_string();
    command_traj_topic_ = this->get_parameter("topics_names.command_traj_topic").as_string();
    odometry_topic_ = this->get_parameter("topics_names.odometry_topic").as_string();
    status_topic_ = this->get_parameter("topics_names.status_topic").as_string();
    battery_status_topic_ = this->get_parameter("topics_names.battery_status_topic").as_string();
    actuator_status_topic_ = this->get_parameter("topics_names.actuator_status_topic").as_string();
    offboard_control_topic_ = this->get_parameter("topics_names.offboard_control_topic").as_string();
    vehicle_command_topic_ = this->get_parameter("topics_names.vehicle_command_topic").as_string();
    actuator_control_topic_ = this->get_parameter("topics_names.actuator_control_topic").as_string();
    servos_control_topic_ = this->get_parameter("topics_names.servos_control_topic").as_string();
    servos_status_topic_ = this->get_parameter("topics_names.servos_status_topic").as_string();
    
    // Load logic switches
    this->declare_parameter("sitl_mode", true);

    in_sitl_mode_ = this->get_parameter("sitl_mode").as_bool();
    
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
    
    // ===== OLD LEE CONTROLLER GAINS (KEPT FOR SMC TUNING) =====
    this->declare_parameter("control_gains.K_p_x", 0.0);
    this->declare_parameter("control_gains.K_p_y", 0.0);
    this->declare_parameter("control_gains.K_p_z", 0.0);
    this->declare_parameter("control_gains.K_v_x", 0.0);
    this->declare_parameter("control_gains.K_v_y", 0.0);
    this->declare_parameter("control_gains.K_v_z", 0.0);
    this->declare_parameter("control_gains.K_R_x", 0.0);
    this->declare_parameter("control_gains.K_R_y", 0.0);
    this->declare_parameter("control_gains.K_R_z", 0.0);
    this->declare_parameter("control_gains.K_w_x", 0.0);
    this->declare_parameter("control_gains.K_w_y", 0.0);
    this->declare_parameter("control_gains.K_w_z", 0.0);

    position_gain_ << this->get_parameter("control_gains.K_p_x").as_double(),
                      this->get_parameter("control_gains.K_p_y").as_double(),
                      this->get_parameter("control_gains.K_p_z").as_double();

    velocity_gain_ << this->get_parameter("control_gains.K_v_x").as_double(),
                      this->get_parameter("control_gains.K_v_y").as_double(),
                      this->get_parameter("control_gains.K_v_z").as_double();

    attitude_gain_ << this->get_parameter("control_gains.K_R_x").as_double(),
                      this->get_parameter("control_gains.K_R_y").as_double(),
                      this->get_parameter("control_gains.K_R_z").as_double();

    ang_vel_gain_ << this->get_parameter("control_gains.K_w_x").as_double(),
                     this->get_parameter("control_gains.K_w_y").as_double(),
                     this->get_parameter("control_gains.K_w_z").as_double();
    // ===== OLD LEE CONTROLLER GAINS (KEPT FOR SMC TUNING) =====

    // pass UAV parameters and SMC controller gains to the controller
    controller_.setUavMass(_uav_mass);
    controller_.setInertiaMatrix(_inertia_matrix);
    controller_.setGravity(_gravity);
    controller_.setPitchTrim(_pitch_trim_rad);
    controller_.setLambda(lambda);
    controller_.setKs(k_s);
    controller_.setPhi(phi);
    controller_.setLambdaR(lambda_r);
    controller_.setKsR(k_s_r);
    controller_.setPhiR(phi_r);

    controller_.setKPositionGain(position_gain_);
    controller_.setKVelocityGain(velocity_gain_);
    controller_.setKAttitudeGain(attitude_gain_);
    controller_.setKAngularRateGain(ang_vel_gain_);
}

void ControllerNode::compute_ControlAllocation_and_ActuatorEffect_matrices(double tilt_1_rad, double tilt_2_rad) {
    Eigen::MatrixXd rotor_velocities_to_torques_and_thrust;

    if (_num_of_arms == 3) {
        // 3x3 reduced allocation for tricopter tilt-rotor: [tau_x, tau_y, thrust].
        // Tau_z is intentionally handled by a separate loop.
        rotor_velocities_to_torques_and_thrust.resize(3, 3);

        double l_1_x = 0.1815;
        double l_1_y = 0.22;
        double l_2_x = 0.1815;
        double l_2_y = 0.22;
        double l_3_x = 0.4185;
        double l_1_z = 0.0;
        double l_2_z = 0.0;

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

    std::cout << "[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n";
}

void ControllerNode::px4Inverse
    (Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench) {
    Eigen::VectorXd omega;
    Eigen::VectorXd pwm;
    Eigen::VectorXd ones_temp;
    if (_num_of_arms == 3){
        omega.resize(3);
        omega.setZero();
        pwm.resize(3);
        pwm.setZero();
        throttles->resize(3);
        throttles->setZero();
        ones_temp.resize(3);
        ones_temp = Eigen::VectorXd::Ones(3,1);
    }
    else if (_num_of_arms == 4){
        omega.resize(4);
        omega.setZero();
        pwm.resize(4);
        pwm.setZero();
        throttles->resize(4);
        throttles->setZero();
        ones_temp.resize(4);
        ones_temp = Eigen::VectorXd::Ones(4,1);
    }
    else {
        std::cout<<("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n");
    }
    // Control allocation: Wrench to rotor squared speeds (omega holds omega^2 before sqrt)
    if (_num_of_arms == 3) {
        Eigen::Vector3d reduced_wrench;
        reduced_wrench << (*wrench)(0), (*wrench)(1), (*wrench)(3);
        Eigen::Vector3d omega_sq = torques_and_thrust_to_rotor_velocities_ * reduced_wrench;

        // Safety: clamp negative values to zero
        for (int i = 0; i < omega_sq.size(); i++){
            if (omega_sq[i] <= 0){
                omega_sq[i] = 0.0;
            }
        }

        // Compute front-tilt from desired tau_z using user formula
        // tau_z_0 = moment_constant * _thrust_constant *(-omega1^2 + omega2^2 - omega3^2)
        // tilt = (tau_z_desired - tau_z_0) / (thrust_constant*(l1y*omega1^2 + l2y*omega2^2))

        double tau_z_desired = (*wrench)(2);
        double tau_z_0 = _moment_constant * _thrust_constant * (-omega_sq[0] + omega_sq[1] - omega_sq[2]);

        // arm y lever arms (match values in compute_ControlAllocation_and_ActuatorEffect_matrices)
        const double l1y = 0.22;
        const double l2y = 0.22;

        double denom = _thrust_constant * (l1y * omega_sq[0] + l2y * omega_sq[1]);
        double computed_tilt_rad = 0.0;
        if (std::abs(denom) > 1e-9) {
            computed_tilt_rad = (tau_z_desired - tau_z_0) / denom;
        }

        // Keep physical angles in radians internally; convert to normalized only when publishing.
        double tilt_1_desired = std::clamp(computed_tilt_rad, kTiltMinDeg * kDegToRad, -kTiltMinDeg * kDegToRad);
        
        // Apply rate limiting with static variables
        static double tilt_1_prev = 0.0;
        double delta = std::clamp(tilt_1_desired - tilt_1_prev, -kTiltRateLimitRadPerStep, kTiltRateLimitRadPerStep);
        tilt_1_rad_ = tilt_1_prev + delta;
        tilt_1_prev = tilt_1_rad_;
        tilt_2_rad_ = -tilt_1_rad_;

        // finalize omega (take sqrt)
        omega = omega_sq.cwiseSqrt();
    } else {
        omega = torques_and_thrust_to_rotor_velocities_ * (*wrench);
        for (int i = 0; i < omega.size(); i++){
            if (omega[i] <= 0){
                omega[i] = 0.0;
            }
        }
        // convert omega^2 to omega (rad/s) for PWM mapping
        omega = omega.cwiseSqrt();
    }
    pwm = (_omega_to_pwm_coefficients(0) * omega.cwiseProduct(omega)) + (_omega_to_pwm_coefficients(1) * omega) +
          (_omega_to_pwm_coefficients(2) * ones_temp);
    *throttles = (pwm - (_PWM_MIN * ones_temp));
    *throttles /= (_PWM_MAX - _PWM_MIN);
}

void ControllerNode::px4InverseSITL
    (Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench) {
    Eigen::VectorXd omega;
    Eigen::VectorXd ones_temp;
    if (_num_of_arms == 3){
        omega.resize(3);
        omega.setZero();
        throttles->resize(3);
        throttles->setZero();
        ones_temp.resize(3);
        ones_temp = Eigen::VectorXd::Ones(3,1);
    }
    else if (_num_of_arms == 6){
        omega.resize(6);
        omega.setZero();
        throttles->resize(6);
        throttles->setZero();
        ones_temp.resize(6);
        ones_temp = Eigen::VectorXd::Ones(6,1);
    }
    else if (_num_of_arms == 4){
        omega.resize(4);
        omega.setZero();
        throttles->resize(4);
        throttles->setZero();
        ones_temp.resize(4);
        ones_temp = Eigen::VectorXd::Ones(4,1);
    }
    else if (_num_of_arms == 44){
        omega.resize(8);
        omega.setZero();
        throttles->resize(8);
        throttles->setZero();
        ones_temp.resize(8);
        ones_temp = Eigen::VectorXd::Ones(8,1);
    }
    else {
        std::cout<<("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n");
    }
    // Control allocation: Wrench to Rotational velocities (omega)
    if (_num_of_arms == 3) {
        Eigen::Vector3d reduced_wrench;
        reduced_wrench << (*wrench)(0), (*wrench)(1), (*wrench)(3);
        Eigen::Vector3d omega_sq = torques_and_thrust_to_rotor_velocities_ * reduced_wrench;

        // Safety: clamp negative values to zero
        for (int i = 0; i < omega_sq.size(); i++){
            if (omega_sq[i] <= 0){
                omega_sq[i] = 0.0;
            }
        }

        // Compute front-tilt using same law as in px4Inverse

        double tau_z_desired = (*wrench)(2);
        double tau_z_0 = _moment_constant * _thrust_constant * (-omega_sq[0] + omega_sq[1] - omega_sq[2]);
        const double l1y = 0.22;
        const double l2y = 0.22;
        double denom = _thrust_constant * (l1y * omega_sq[0] + l2y * omega_sq[1]);
        double computed_tilt_rad = 0.0;
        if (std::abs(denom) > 1e-9) {
            computed_tilt_rad = (tau_z_desired - tau_z_0) / denom;
        }
        
        double tilt_1_desired = std::clamp(computed_tilt_rad, kTiltMinDeg * kDegToRad, -kTiltMinDeg * kDegToRad);
        
        // Apply rate limiting with static variables
        static double tilt_1_prev = 0.0;
        double delta = std::clamp(tilt_1_desired - tilt_1_prev, -kTiltRateLimitRadPerStep, kTiltRateLimitRadPerStep);
        tilt_1_rad_ = tilt_1_prev + delta;
        tilt_1_prev = tilt_1_rad_;
        tilt_2_rad_ = -tilt_1_rad_;

        omega = omega_sq.cwiseSqrt();
    } else {
        omega = torques_and_thrust_to_rotor_velocities_ * (*wrench);
        // Safety: Clamp negative rotor velocities to zero (prevents NaN from sqrt of negative)
        for (int i = 0; i < omega.size(); i++){
            if (omega[i] <= 0){
                omega[i] = 0.0;
            }
        }
        // convert omega^2 to omega (rad/s) for SITL mapping
        omega = omega.cwiseSqrt();
    }
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
    // initialize vectors
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    eigenTrajectoryPointFromPoseMsg(pose_msg, position, orientation);
    RCLCPP_INFO_ONCE(get_logger(),"Controller got first command message.");
    controller_.setTrajectoryPoint(position, orientation);          // Send the command to controller_ obj
}

void ControllerNode::commandTrajectoryCallback(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint &msg) {                   // When a command is received
    // initialize vectors
    Eigen::Vector3d position;
    Eigen::Vector3d velocity; 
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d acceleration;
    eigenTrajectoryPointFromMsg(msg, position, orientation, velocity, angular_velocity, acceleration);
    controller_.setTrajectoryPoint(position, velocity, acceleration, orientation, angular_velocity);
    RCLCPP_INFO_ONCE(get_logger(),"Controller got first command message.");
}

// void ControllerNode::platformPositionCallback(const geometry_msgs::msg::Vector3::SharedPtr pos_msg) {
//     // Receive platform position feedback from landing_trajectory_node (Phase 2 only)
//     // This is the estimated position of the platform in world frame, obtained from TF
//     Eigen::Vector3d platform_position(pos_msg->x, pos_msg->y, pos_msg->z);
//     controller_.setActualPosition(platform_position);
//     RCLCPP_DEBUG(get_logger(), "Received platform position feedback: [%.3f, %.3f, %.3f]",
//                  platform_position(0), platform_position(1), platform_position(2));
// }

void ControllerNode::vehicle_odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg){
        //  Debug message
        RCLCPP_INFO_ONCE(get_logger(),"Controller got first odometry message.");
        
        last_odometry_timestamp_ = odom_msg->timestamp;

        Eigen::Vector3d position;
        Eigen::Vector3d velocity; 
        Eigen::Quaterniond orientation;
        Eigen::Vector3d angular_velocity;
        
        eigenOdometryFromPX4Msg(odom_msg,
                                position, orientation, velocity, angular_velocity);

        controller_.setOdometry(position, orientation, velocity, angular_velocity);
}

void ControllerNode::servosStatusCallback(const px4_msgs::msg::ActuatorServos::SharedPtr servos_msg) {
    constexpr int kTilt1ServoIndex = 4;  // Servo 5 (1-based)
    constexpr int kTilt2ServoIndex = 5;  // Servo 6 (1-based)

    if (_num_of_arms != 3) {
        return;
    }

    // Read measured servo normalized values (these are measurements, not commands)
    const double measured_tilt_1_norm = std::clamp(static_cast<double>(servos_msg->control[kTilt1ServoIndex]), -1.0, 1.0);
    const double measured_tilt_2_norm = std::clamp(static_cast<double>(servos_msg->control[kTilt2ServoIndex]), -1.0, 1.0);
    // Convert measurements to radians
    const double measured_tilt_1_rad = servoNormToTiltRad(measured_tilt_1_norm);
    const double measured_tilt_2_rad = servoNormToTiltRad(measured_tilt_2_norm);
    // store measured tilts separately from commanded tilts
    measured_tilt_1_rad_ = measured_tilt_1_rad;
    measured_tilt_2_rad_ = measured_tilt_2_rad;

    compute_ControlAllocation_and_ActuatorEffect_matrices(measured_tilt_1_rad_, measured_tilt_2_rad_);
}

void ControllerNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg){
    current_status_ = *status_msg;
    // if (current_status_.arming_state ==2){
    //     RCLCPP_INFO_ONCE(get_logger(),"ARMED - vehicle_status_msg.");
    // }
    // else {
    //     RCLCPP_INFO(get_logger(),"NOT ARMED - vehicle_status_msg.");
    // }
    // if (current_status_.nav_state == 14){
    //     RCLCPP_INFO_ONCE(get_logger(),"OFFBOARD - vehicle_status_msg.");
    // }
    // else {
    //     RCLCPP_INFO(get_logger(),"NOT OFFBOARD - vehicle_status_msg.");
    // }
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
	actuator_motors_msg.timestamp = this->get_clock()->make_shared()->now().nanoseconds() / 1000;
	actuator_motors_msg.timestamp_sample = actuator_motors_msg.timestamp;

	actuator_motors_publisher_->publish(actuator_motors_msg);
}

void ControllerNode::publishActuatorServosMsg(double tilt_1_rad, double tilt_2_rad) {
    px4_msgs::msg::ActuatorServos actuator_servos_msg;

    const double tilt_1_norm = tiltRadToServoNorm(tilt_1_rad);
    const double tilt_2_norm = tiltRadToServoNorm(tilt_2_rad);

    auto safe_servo = [](double v){
        if (!std::isfinite(v)) return 0.0f;
        return (float)std::clamp(v, -1.0, 1.0);
    };
    actuator_servos_msg.control = { 0.0f, 0.0f, 0.0f, 0.0f,
                                    safe_servo(tilt_1_norm),
                                    safe_servo(tilt_2_norm),
                                    std::nanf("1"), std::nanf("1") };
    actuator_servos_msg.timestamp = this->get_clock()->make_shared()->now().nanoseconds() / 1000;
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
}

void ControllerNode::updateControllerOutput() {
    //  calculate controller output
    Eigen::VectorXd controller_output;
    Eigen::Quaterniond desired_quaternion;
    controller_.calculateControllerOutput(&controller_output, &desired_quaternion);
    publishWrenchMsg(controller_output, last_odometry_timestamp_);
    
    // Debug: Log the controller output (only once per second to avoid spam)
    static int iteration_count = 0;
    if (iteration_count++ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "Controller output [tau_x, tau_y, tau_z, thrust]: [%.3f, %.3f, %.3f, %.3f]",
                    controller_output(0), controller_output(1), controller_output(2), controller_output(3));
        RCLCPP_INFO(this->get_logger(), "Vehicle nav_state: %d (Offboard=%d)", 
                    current_status_.nav_state, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    }
    
    Eigen::VectorXd throttles;
    if (in_sitl_mode_) px4InverseSITL(&throttles, &controller_output);
    else px4Inverse(&throttles, &controller_output);

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