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

#include "../include/px4_offboard_lowlevel/st_smc_controller.h"
#include "../include/px4_offboard_lowlevel/sta_reaching_law.h"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <eigen3/Eigen/Geometry>

StSmcController::StSmcController(){

}

void StSmcController::calculateControllerOutput(
        Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion) {
    assert(controller_torque_thrust);

    controller_torque_thrust->resize(4);

    // Trajectory tracking.
    double thrust;
    Eigen::Matrix3d R_d_w;

    Eigen::Vector3d omega_ref = Eigen::Vector3d::Zero();

    // Compute translational tracking errors.
    const Eigen::Vector3d e_p =
                position_W_ - r_position_W_;

    const Eigen::Vector3d e_v =
                velocity_W_ - r_velocity_W_;

    const Eigen::Vector3d s =
                e_v + Lambda.cwiseProduct(e_p);

    // Composite control (§05): the observer cancels the slow, large part of the
    // disturbance so the super-twisting law is left with only the fast residual.
    //
    // Unlike the plain smc law, the STA is not helpless against a steady wind --
    // w_ is an integral state and that is exactly what it is for. The argument
    // here is the other one from §05: with the disturbance estimated, the
    // reaching gains only have to bound the ESTIMATION ERROR rather than the
    // disturbance itself, which is the standard route to smaller K1/K2 and less
    // chattering. w_ is also bounded (kStaThrustHoverFraction) and only builds
    // through the sliding surface, so it is slower to a step of wind than a
    // feedforward that does not wait for error to appear.
    updateExternalForceEstimate();

    s_last_ = s;   // diagnostic only

    // Super-twisting reaching law: u = -K1*sqrt(|s|)*sign(s) - K3*s + w,
    //                               w_dot = -K2*sign(s) - K4*s
    // K3/K4 zero and the explicit scheme reproduce the classical law exactly;
    // see sta_reaching_law.h for both options and why each defaults off.
    //
    // The bounded, saturation-aware integration of w_ happens inside the step,
    // so the bound and the anti-windup are identical for every variant.
    px4_offboard::StaGains gains;
    gains.k1 = K1;
    gains.k2 = K2;
    const Eigen::Vector3d w_limit = Eigen::Vector3d::Constant(
        px4_offboard::kStaThrustHoverFraction * _uav_mass * _gravity);
    // Control effectiveness of the translational surface: s_dot = u/m + d.
    const Eigen::Vector3d b_translational =
        Eigen::Vector3d::Constant(1.0 / std::max(_uav_mass, 1e-6));
    const Eigen::Vector3d u_sta =
        px4_offboard::staReachingStep(s, w_, gains, b_translational, w_limit,
                                      actuators_saturated_, dt_,
                                      false);

    const Eigen::Vector3d I_a_d =
                + _uav_mass * _gravity * Eigen::Vector3d::UnitZ()
                + _uav_mass * r_acceleration_W_
                - _uav_mass * Lambda.cwiseProduct(e_v)
                + u_sta
                - f_ext_hat_;

    thrust = projectedThrust(I_a_d);
    noteAppliedThrust(thrust);
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

    // See setReferenceRateFilterHz(): omega_ref is a 100 Hz derivative of an
    // attitude built from the FULL force command, so it carries the feedback's
    // high-frequency content and not just the reference's. Off by default.
    omega_ref = filterReferenceRate(omega_ref);

    omega_ref_last_ = omega_ref;   // diagnostic only
    i_a_d_last_ = I_a_d;           // diagnostic only


    Eigen::Quaterniond q_temp(R_d_w);
    *desired_quaternion = q_temp;

    // Attitude tracking.
    Eigen::Vector3d tau;

    const Eigen::Matrix3d e_R_matrix =
            0.5 * (R_d_w.transpose() * R_B_W_ - R_B_W_.transpose() * R_d_w)   ;
    Eigen::Vector3d e_R;
    e_R << e_R_matrix(2, 1), e_R_matrix(0, 2), e_R_matrix(1, 0);
    const Eigen::Vector3d e_omega = angular_velocity_B_ - R_B_W_.transpose() * R_d_w * omega_ref;

    Eigen::Matrix3d Q = R_B_W_.transpose() * R_d_w;
    Eigen::Matrix3d E = 0.5 * ( Q.trace() * Eigen::Matrix3d::Identity() - Q);
    Eigen::Vector3d e_R_dot = E * e_omega;

    Eigen::Vector3d s_R = e_omega + Lambda_R.cwiseProduct(e_R);

    s_R_last_ = s_R;   // diagnostic only

    // Super-twisting reaching law for the rotational sliding surface. Same
    // shared step as the translational branch, so the two cannot drift apart.
    px4_offboard::StaGains gains_R;
    gains_R.k1 = K1_R;
    gains_R.k2 = K2_R;
    // Bounded by the angular acceleration it may command rather than by a raw
    // torque: a torque limit would be airframe-specific, this is not.
    const Eigen::Vector3d w_R_limit =
        _inertia_matrix.diagonal() * px4_offboard::kStaMaxAngularAcceleration;
    // Control effectiveness of the rotational surface: s_R_dot ~= tau/I.
    Eigen::Vector3d b_rotational;
    for (int i = 0; i < 3; ++i) {
        b_rotational(i) = 1.0 / std::max(_inertia_matrix(i, i), 1e-9);
    }
    const Eigen::Vector3d u_sta_R =
        px4_offboard::staReachingStep(s_R, w_R_, gains_R, b_rotational, w_R_limit,
                                      actuators_saturated_, dt_,
                                      false);

    tau =
        angular_velocity_B_.cross(_inertia_matrix * angular_velocity_B_)
        - _inertia_matrix * angular_velocity_B_.cross(R_B_W_.transpose() * R_d_w * omega_ref)
        - _inertia_matrix * Lambda_R.cwiseProduct(e_R_dot)
        + u_sta_R;

    // Output the wrench
    *controller_torque_thrust << tau, thrust;
}
