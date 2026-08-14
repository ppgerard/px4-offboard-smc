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

#ifndef CONTROLLER_CONTROLLER_BASE_H
#define CONTROLLER_CONTROLLER_BASE_H

#include <algorithm>

#include <eigen3/Eigen/Eigen>

#include "px4_offboard_lowlevel/control_config.h"

namespace px4_offboard {

// Floor on the commanded thrust, as a fraction of hover thrust. The projection
// in projectedThrust() falls to zero as the body z-axis turns away from the
// desired force; following it there would cut the rotors, and with them the
// attitude authority needed to recover, at the one moment the aircraft is
// furthest from where it should be. Only reachable beyond ~84 deg of lag.
inline constexpr double kMinThrustHoverFraction = 0.10;

// Bound on the translational super-twisting integral state, as a fraction of
// hover thrust. The STA's stability argument assumes a bounded disturbance
// derivative, but nothing in the recursion enforces the matching bound on the
// state itself. Sized to cover the disturbances it exists to absorb — thrust
// model error, battery sag, steady wind — and not much more.
inline constexpr double kStaThrustHoverFraction = 0.40;

// Bound on the rotational super-twisting integral state, expressed as the
// angular acceleration it is allowed to command. A torque limit would be
// airframe-specific; an acceleration limit scaled by the inertia is not.
inline constexpr double kStaMaxAngularAcceleration = 2.0;  // [rad/s^2]

}  // namespace px4_offboard

// Common interface for the position/attitude control laws (SMC, STSMC, ...).
// Holds the vehicle parameters and current/reference state shared by every
// control law; each law implements calculateControllerOutput() on top of it.
class ControllerBase {
public:
    virtual ~ControllerBase() = default;

    virtual void calculateControllerOutput(Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion) = 0;

    // Clears everything carried between control cycles. Call on every offboard
    // entry: the control law runs on a free-running timer from node start and
    // only its output is gated on offboard, so without this whatever the state
    // wound up to while the vehicle sat on the pad becomes the initial
    // condition at engagement. Laws with their own state extend this.
    virtual void reset() {
        R_d_prev_ = Eigen::Matrix3d::Identity();
        first_iteration_ = true;
        actuators_saturated_ = false;
    }

    // Whether the previous commanded wrench reached the actuators intact. False
    // means the loop is open downstream of the law, which is precisely when
    // integrating winds up state the law can never work off again.
    void setActuatorsSaturated(bool saturated) {
        actuators_saturated_ = saturated;
    }

    // Setters
    void setOdometry(const Eigen::Vector3d &position_W, const Eigen::Quaterniond &orientation_B_W,
        const Eigen::Vector3d &velocity_B, const Eigen::Vector3d &angular_velocity_B){
        R_B_W_ = orientation_B_W.toRotationMatrix();
        position_W_ = position_W;
        velocity_W_ = R_B_W_ * velocity_B;
        angular_velocity_B_ = angular_velocity_B;
    }

    void setTrajectoryPoint(const Eigen::Vector3d &position_W, const Eigen::Vector3d &velocity_W, const Eigen::Vector3d &acceleration_W
                    , const Eigen::Quaterniond &orientation_W, const Eigen::Vector3d &angular_velocity_B){
        r_position_W_ = position_W;
        r_velocity_W_ = velocity_W;
        r_acceleration_W_ = acceleration_W;
        r_R_B_W_ = orientation_W.toRotationMatrix();
        r_yaw = r_R_B_W_.eulerAngles(0, 1, 2)(2);
        r_yaw_rate = angular_velocity_B(2);
    }

    void setTrajectoryPoint(const Eigen::Vector3d &position_W, const Eigen::Quaterniond &orientation_W){
        r_position_W_ = position_W;
        r_velocity_W_.setZero();
        r_acceleration_W_.setZero();
        r_R_B_W_ = orientation_W.toRotationMatrix();
        r_yaw = r_R_B_W_.eulerAngles(0, 1, 2)(2);
        r_yaw_rate = 0.0;
    }

    void setUavMass(double uavMass) {
        _uav_mass = uavMass;
    }

    void setInertiaMatrix(const Eigen::Matrix3d &inertiaMatrix) {
        _inertia_matrix = inertiaMatrix;
    }

    void setGravity(double gravity) {
        _gravity = gravity;
    }

    // Fixed pitch trim applied to the desired attitude, to compensate a
    // vehicle's natural mounting tilt (e.g. the t2 tiltrotor). Leave at the
    // default 0.0 for airframes that don't need it (e.g. x500).
    void setPitchTrim(double pitchTrimRad) {
        _pitch_trim_rad = pitchTrimRad;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
    // Thrust to command for a desired force I_a_d: the component of it the
    // rotors can actually deliver, which is its projection on the current body
    // z-axis. Commanding the norm instead over-thrusts whenever the attitude
    // lags the desired force direction — 6% at 20 deg of lag, 13% at 30 deg —
    // and the lag is largest during a gust, so the error arrives as altitude
    // bumps correlated with lateral disturbance. Floored, see the constant.
    double projectedThrust(const Eigen::Vector3d &I_a_d) const {
        return std::max(I_a_d.dot(R_B_W_.col(2)),
                        px4_offboard::kMinThrustHoverFraction * _uav_mass * _gravity);
    }

    // Super-twisting integral update w += increment, with the two bounds the
    // plain recursion leaves out: |w| limited per axis, and no integration
    // while the actuators are saturated (conditional integration).
    //
    // Freezing outright would trap the state — wound up, and unable to unwind
    // for as long as the saturation lasts — so an increment that shrinks |w| is
    // still applied. Only growth is held back.
    static void integrateStaState(Eigen::Vector3d &w, const Eigen::Vector3d &increment,
                                  const Eigen::Vector3d &limit, bool saturated) {
        for (int i = 0; i < 3; ++i) {
            const bool winds_up = increment(i) * w(i) > 0.0;
            if (saturated && winds_up) {
                continue;
            }
            w(i) = std::clamp(w(i) + increment(i), -limit(i), limit(i));
        }
    }

    // Control loop period, used for the numerical derivatives in the laws.
    double dt_ = px4_offboard::kControlPeriodSeconds;

    // Set by the node when the commanded wrench did not survive allocation:
    // a rotor clamped at zero, a throttle outside [0, 1], a tilt on its stop.
    bool actuators_saturated_ = false;

    // Previous desired attitude and first-call guard, used to differentiate the
    // desired attitude into a reference angular velocity. Per-instance so that
    // several controllers can coexist (e.g. in tests) without sharing state.
    Eigen::Matrix3d R_d_prev_ = Eigen::Matrix3d::Identity();
    bool first_iteration_ = true;

    // UAV Parameter
    double _uav_mass = 0.0;
    Eigen::Matrix3d _inertia_matrix = Eigen::Matrix3d::Zero();
    double _gravity = 0.0;
    double _pitch_trim_rad = 0.0;

    // Current states. Initialised so that a control law evaluated before the
    // first odometry message operates on a defined state rather than on
    // whatever happened to be in memory.
    Eigen::Vector3d position_W_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_W_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_B_W_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d angular_velocity_B_ = Eigen::Vector3d::Zero();
    // References
    Eigen::Vector3d r_position_W_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_velocity_W_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_acceleration_W_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d r_R_B_W_ = Eigen::Matrix3d::Identity();
    double r_yaw = 0.0;
    double r_yaw_rate = 0.0;
};

#endif //CONTROLLER_CONTROLLER_BASE_H
