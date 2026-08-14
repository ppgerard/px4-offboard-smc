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

#include "../include/px4_offboard_lowlevel/smc_controller.h"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <eigen3/Eigen/Geometry>


SmcController::SmcController(){

}

void SmcController::calculateControllerOutput(
        Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion) {
    assert(controller_torque_thrust);

    controller_torque_thrust->resize(4);

    // Trajectory tracking.
    double thrust;
    Eigen::Matrix3d R_d_w;

    Eigen::Vector3d omega_ref = Eigen::Vector3d::Zero();

    // For tuning only :
    // r_position_W_(0) = position_W_(0);
    // r_position_W_(1) = position_W_(1);
    // r_velocity_W_(0) = velocity_W_(0);
    // r_velocity_W_(1) = velocity_W_(1);
    // r_acceleration_W_ = Eigen::Vector3d::Zero();
    // r_yaw = 0.5;
    // r_acceleration_W_ << 0, 0, 0;


    // Compute translational tracking errors.
    const Eigen::Vector3d e_p =
                position_W_ - r_position_W_;
    
    const Eigen::Vector3d e_v = 
                velocity_W_ - r_velocity_W_;
        
    const Eigen::Vector3d s = 
                e_v + Lambda.cwiseProduct(e_p);
    
    Eigen::Vector3d sat_vec = s.cwiseQuotient(phi);
    sat_vec = sat_vec.cwiseMax(-1.0).cwiseMin(1.0);

    // const Eigen::Vector3d I_a_d = 
    //             -position_gain_.cwiseProduct(e_p)
    //             -velocity_gain_.cwiseProduct(e_v)
    //             +_uav_mass * _gravity * Eigen::Vector3d::UnitZ() + _uav_mass * r_acceleration_W_;

    const Eigen::Vector3d I_a_d = 
                + _uav_mass * _gravity * Eigen::Vector3d::UnitZ() 
                + _uav_mass * r_acceleration_W_
                - _uav_mass * Lambda.cwiseProduct(e_v)
                - K_s.cwiseProduct(sat_vec);

    thrust = projectedThrust(I_a_d);
    Eigen::Vector3d B_z_d;
    B_z_d = I_a_d;
    B_z_d.normalize();

    // Calculate Desired Rotational Matrix
    const Eigen::Vector3d B_x_d(std::cos(r_yaw), std::sin(r_yaw), 0.0);
    Eigen::Vector3d B_y_d = B_z_d.cross(B_x_d);
    B_y_d.normalize();
    R_d_w.col(0) = B_y_d.cross(B_z_d);
    R_d_w.col(1) = B_y_d;
    R_d_w.col(2) = B_z_d;

    // Apply the vehicle's fixed pitch trim (positive = nose up) to the desired
    // attitude, compensating a natural mounting tilt (e.g. the t2 tiltrotor).
    // Zero for airframes that don't set uav_parameters.pitch_trim_deg.
    if (_pitch_trim_rad != 0.0) {
        const Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(_pitch_trim_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
        R_d_w = R_d_w * R_pitch;
    }

    if (!first_iteration_)
    {
        Eigen::Matrix3d R_d_dot =
            (R_d_w - R_d_prev_) / dt_;

        Eigen::Matrix3d omega_hat =
            0.5 * (
                R_d_w.transpose() * R_d_dot
                - R_d_dot.transpose() * R_d_w
            );

        omega_ref <<
            omega_hat(2,1),
            omega_hat(0,2),
            omega_hat(1,0);
    }
    else
    {
        first_iteration_ = false;
    }

    R_d_prev_ = R_d_w;

    
    Eigen::Quaterniond q_temp(R_d_w);
    *desired_quaternion = q_temp;
    
    // Attitude tracking.
    Eigen::Vector3d tau;

    const Eigen::Matrix3d e_R_matrix =
            0.5 * (R_d_w.transpose() * R_B_W_ - R_B_W_.transpose() * R_d_w)   ;
    Eigen::Vector3d e_R;
    e_R << e_R_matrix(2, 1), e_R_matrix(0, 2), e_R_matrix(1, 0);
    // const Eigen::Vector3d omega_ref =
    //         r_yaw_rate * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d e_omega = angular_velocity_B_ - R_B_W_.transpose() * R_d_w * omega_ref;
    
    Eigen::Matrix3d Q = R_B_W_.transpose() * R_d_w;
    Eigen::Matrix3d E = 0.5 * ( Q.trace() * Eigen::Matrix3d::Identity() - Q);
    Eigen::Vector3d e_R_dot = E * e_omega;

    // Eigen::Vector3d e_R_dot = 0.5 * (Eigen::Matrix3d::Identity() + R_B_W_.transpose() * R_d_w) * e_omega;

    // RCLCPP_INFO_STREAM(rclcpp::get_logger("controller"), "e_omega: [" << e_omega.transpose() << "]");
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("controller"), "e_R: [" << e_R.transpose() << "]");
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("controller"), "e_R_dot: [" << e_R_dot.transpose() << "]");
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("controller"), "e_p: [" << e_p.transpose() << "]");
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("controller"), "e_v: [" << e_v.transpose() << "]");
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("controller"), "b_3: [" << R_B_W_.col(2).transpose() << "]");
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("controller"), "d_3: [" << B_z_d.transpose() << "]");

    Eigen::Vector3d s_R = e_omega + Lambda_R.cwiseProduct(e_R);
    Eigen::Vector3d sat_vec_R = s_R.cwiseQuotient(phi_R);
    sat_vec_R = sat_vec_R.cwiseMax(-1.0).cwiseMin(1.0);
    
    // SMC Controller
    // Missing the desired angular acceleration term.
    tau =
        angular_velocity_B_.cross(_inertia_matrix * angular_velocity_B_)
        - _inertia_matrix * angular_velocity_B_.cross(R_B_W_.transpose() * R_d_w * omega_ref)
        - _inertia_matrix * Lambda_R.cwiseProduct(e_R_dot)
        - K_s_R.cwiseProduct(sat_vec_R);
    
    // Lee's Geometric Controller
    // tau = -attitude_gain_.cwiseProduct(e_R)
    //        - angular_rate_gain_.cwiseProduct(e_omega)
    //        + angular_velocity_B_.cross(_inertia_matrix * angular_velocity_B_);


    // Output the wrench
    *controller_torque_thrust << tau, thrust;
}
