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

#ifndef CONTROLLER_CONTROLLER_NODE_H
#define CONTROLLER_CONTROLLER_NODE_H

#include "rclcpp/rclcpp.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include <string>
#include <memory>
#include <vector>

#include "px4_offboard_lowlevel/controller_base.h"
#include "px4_offboard_lowlevel/px4_frame_conversions.h"
#include "px4_offboard_lowlevel/smc_controller.h"
#include "px4_offboard_lowlevel/st_smc_controller.h"

#include <chrono>
using namespace std::chrono_literals;

using std::placeholders::_1;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode();    
    //virtual ~controller_node();
    void updateControllerOutput();

private:

    std::unique_ptr<ControllerBase> controller_;
    std::string controller_type_;

    // Timers
    rclcpp::TimerBase::SharedPtr controllerTimer;
    rclcpp::TimerBase::SharedPtr offboardTimer;
    uint64_t last_odometry_timestamp_ = 0;
    bool odometry_received_ = false;

    // subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr command_pose_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr command_trajectory_sub_;
    rclcpp::Subscription<px4_msgs::msg::ActuatorServos>::SharedPtr servos_status_sub_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr actuator_servos_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
    // The external-force observer's estimate, in the world frame. A wind vector
    // in newtons: the one signal that separates "the disturbance is large" from
    // "the loop is not using what it already knows".
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr f_ext_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr attitude_reference_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr sliding_surface_publisher_;
    bool publish_sliding_surface_ = true;
    
    // Topic names
    std::string command_pose_topic_;
    std::string command_traj_topic_;
    std::string odometry_topic_;
    std::string status_topic_;
    std::string servos_status_topic_;
    std::string offboard_control_topic_;
    std::string vehicle_command_topic_;
    std::string actuator_control_topic_;
    std::string servos_control_topic_;

    // UAV Parameters
    double _arm_length;
    int _num_of_arms;
    double _moment_constant;
    double _thrust_constant;
    Eigen::Vector3d _omega_to_pwm_coefficients;
    int _PWM_MIN;
    int _PWM_MAX;
    int _SIM_GZ_EC_MAX;
    int _SIM_GZ_EC_MIN;
    // Tilt servo travel and channel assignment. Parameters rather than constants
    // because SITL and the real T2 differ on BOTH: the gz airframe has 4 control
    // surfaces and -7 deg of negative tilt travel, the vehicle has 3 and -5 deg.
    // PX4 assigns servo functions control-surfaces-first, so the tilt index is
    // CA_SV_CS_COUNT.
    double tilt_min_deg_ = -7.0;
    double tilt_max_deg_ = 90.0;
    int tilt_1_servo_index_ = 4;
    int tilt_2_servo_index_ = 5;
    // Sign of the differential-tilt -> yaw-torque relationship. +1 is the
    // simulator's geometry; the real T2 measured -1 in flight. See the note at
    // the tau_z -> tilt computation in controller_node.cpp.
    double tilt_yaw_sign_ = 1.0;
    // Sign of each rotor's yaw-torque contribution in body FLU, i.e. its spin
    // direction. Default is the simulator's aircraft; the real T2's tail differs.
    double rotor_yaw_sign_[3] = {-1.0, 1.0, -1.0};
    // commanded tilt angles (radians) computed by the controller and published
    double tilt_1_rad_ = 0.0;
    double tilt_2_rad_ = 0.0;
    // previous commanded front-tilt, used by the tilt rate limiter
    double tilt_1_prev_ = 0.0;
    // measured tilt angles (radians) read from servos feedback, used for allocation
    double measured_tilt_1_rad_ = 0.0;
    double measured_tilt_2_rad_ = 0.0;
    Eigen::MatrixXd torques_and_thrust_to_rotor_velocities_;
    // Set by computeRotorVelocities() when the commanded wrench had to be
    // clamped to something the airframe can produce; feeds the control law's
    // anti-windup together with the throttle range check.
    bool allocation_saturated_ = false;

    // Logic switches
    bool in_sitl_mode_;

    // Throttled logging counter for updateControllerOutput()
    int iteration_count_ = 0;
    
    px4_msgs::msg::VehicleStatus current_status_;
    bool connected_ = false;

    void loadParams();
    void arm();
    void disarm();

    // CallBacks
    void commandPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    void commandTrajectoryCallback(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint &msg);
    void vehicle_odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg);
    void servosStatusCallback(const px4_msgs::msg::ActuatorServos::SharedPtr servos_msg);

    void publishActuatorMotorsMsg(const Eigen::VectorXd& throttles);
    void publishActuatorServosMsg(double tilt_1_rad, double tilt_2_rad);
    void publishOffboardControlModeMsg();
    void publish_vehicle_command(uint16_t command, float param1 =0.0, float param2 = 0.0);   
    void publishWrenchMsg(const Eigen::VectorXd& wrench, uint64_t timestamp);

    void compute_ControlAllocation_and_ActuatorEffect_matrices(double tilt_1_rad = 0.0,
                                                               double tilt_2_rad = 0.0);
    bool computeRotorVelocities(const Eigen::VectorXd &wrench, Eigen::VectorXd *omega);
    void px4Inverse (Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench);
    void px4InverseSITL (Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench);

    // Unpacks a trajectory point. Returns false and writes nothing when the
    // message carries no transform, which is the caller's cue to skip it: the
    // outputs would otherwise be left holding whatever they held before.
    inline bool eigenTrajectoryPointFromMsg(
        const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& msg,
        Eigen::Vector3d& position_W, Eigen::Quaterniond& orientation_W_B,
        Eigen::Vector3d& velocity_W, Eigen::Vector3d& angular_velocity_W,
        Eigen::Vector3d& acceleration_W) {

        if (msg.transforms.empty()) {
            return false;
        }

        position_W << msg.transforms[0].translation.x,
            msg.transforms[0].translation.y,
            msg.transforms[0].translation.z;
        Eigen::Quaterniond quaternion(msg.transforms[0].rotation.w,
                                    msg.transforms[0].rotation.x,
                                    msg.transforms[0].rotation.y,
                                    msg.transforms[0].rotation.z);
        orientation_W_B = quaternion;
        if (msg.velocities.size() > 0) {
            velocity_W << msg.velocities[0].linear.x,
                msg.velocities[0].linear.y,
                msg.velocities[0].linear.z;
            angular_velocity_W << msg.velocities[0].angular.x,
                msg.velocities[0].angular.y,
                msg.velocities[0].angular.z;
        } else {
            velocity_W.setZero();
            angular_velocity_W.setZero();
        }
        if (msg.accelerations.size() > 0) {
            acceleration_W << msg.accelerations[0].linear.x,
                msg.accelerations[0].linear.y,
                msg.accelerations[0].linear.z;
        } else {
            acceleration_W.setZero();
        }
        return true;
    }

    inline void eigenTrajectoryPointFromPoseMsg(
        const geometry_msgs::msg::PoseStamped::SharedPtr& msg, Eigen::Vector3d& position_W, Eigen::Quaterniond& orientation_W_B) {

        position_W << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        Eigen::Quaterniond quaternion(msg->pose.orientation.w,
                                    msg->pose.orientation.x,
                                    msg->pose.orientation.y,
                                    msg->pose.orientation.z);
        orientation_W_B = quaternion;
    }
};


#endif //px4_offboard_lowlevel_CONTROLLER_NODE_H
