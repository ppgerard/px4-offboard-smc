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

// Bound on the estimated external force, as a fraction of hover thrust. The
// observer is a stable first-order filter and cannot wind up the way an
// integrator can, but it does attribute EVERY unmodelled force to the
// disturbance -- allocation shortfall and thrust-model error included -- so a
// bound keeps a bad thrust model from being fed back as a large steady lean.
// 30% of weight is 7.3 N here, well above the 2.4 N the review's own table
// predicts for the worst horizontal case (8 m/s, edge-on).
inline constexpr double kFextMaxHoverFraction = 0.30;

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
        f_ext_hat_.setZero();
        momentum_integral_.setZero();
        last_applied_thrust_ = 0.0;
        f_ext_initialised_ = false;
        omega_ref_filtered_.setZero();
        omega_ref_last_.setZero();
        i_a_d_last_.setZero();
    }

    // Whether the previous commanded wrench reached the actuators intact. False
    // means the loop is open downstream of the law, which is precisely when
    // integrating winds up state the law can never work off again.
    void setActuatorsSaturated(bool saturated) {
        actuators_saturated_ = saturated;
    }

    // Bandwidth of the external-force observer [rad/s]. Zero disables it, which
    // is what makes this A/B-able against the same binary. The estimate is a
    // first-order lag of the true disturbance at this bandwidth, so it is set by
    // what the disturbance does rather than by what the loop can take: the
    // apriltag world gusts on an 8 s period (0.8 rad/s) about a constant mean.
    void setExternalForceGain(double gain) {
        f_ext_observer_gain_ = std::max(0.0, gain);
    }

    // The estimated external force in the world frame [N]. Published as a
    // diagnostic: it is a wind vector in newtons, and it is the one signal that
    // says whether a poor hold is the disturbance being large or the loop
    // failing to use what it already knows.
    const Eigen::Vector3d &externalForceEstimate() const {
        return f_ext_hat_;
    }

    // Diagnostics for the reference-differentiation path. omega_ref is a 100 Hz
    // finite difference of the desired attitude, and the desired attitude is the
    // DIRECTION of I_a_d -- so every noisy term in the force command reaches the
    // attitude torque through this derivative, with a gain of I*Lambda_R/(m*g*dt).
    // Nothing published either side of it until now.
    const Eigen::Vector3d &referenceAngularVelocity() const { return omega_ref_last_; }

    // Low-pass on omega_ref. The desired attitude is the DIRECTION of I_a_d, so
    // every feedback term -- including the super-twisting output, whose time
    // derivative is unbounded as s -> 0 -- is differentiated at 100 Hz and
    // injected into tau with gain I*Lambda_R. Measured on stsmc in a still-air
    // hover: omega_ref carries 8.4x the high-frequency energy of the ACTUAL body
    // rate, and that path alone is 38% of tau_y's HF power.
    //
    // A reference angular velocity is only real below a few Hz on this mission
    // (static platform, slow approach, yaw pinned to the platform). Above that it
    // is the differentiator's noise. 0.0 disables the filter, so this A/Bs
    // against one binary.
    void setReferenceRateFilterHz(double hz) { omega_ref_filter_hz_ = std::max(0.0, hz); }

    Eigen::Vector3d filterReferenceRate(const Eigen::Vector3d &omega_ref) {
        if (omega_ref_filter_hz_ <= 0.0) {
            omega_ref_filtered_ = omega_ref;
            return omega_ref;
        }
        const double time_constant = 1.0 / (2.0 * M_PI * omega_ref_filter_hz_);
        const double alpha = dt_ / (time_constant + dt_);
        omega_ref_filtered_ += alpha * (omega_ref - omega_ref_filtered_);
        return omega_ref_filtered_;
    }
    const Eigen::Vector3d &desiredAcceleration() const { return i_a_d_last_; }

    // The two sliding surfaces the law last evaluated. These decide which
    // chattering mechanism is even ACTIVE, and neither can be inferred from the
    // wrench: smc's sat() is exactly linear while |s| < Phi, so a run spent
    // inside the boundary layer has no switching to blame at all, and the
    // super-twisting law's incremental gain K1/(2*sqrt(|s|)) is only large where
    // |s| is small. Diagnostic only, and unconditional -- a diagnostic that
    // carries a condition goes silent in the case being debugged.
    const Eigen::Vector3d &slidingSurface() const { return s_last_; }
    const Eigen::Vector3d &rotationalSlidingSurface() const { return s_R_last_; }

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

    // Momentum-based external-force observer (§05). Call once per control
    // cycle, before the law builds I_a_d, and pair it with noteAppliedThrust()
    // at the end of the cycle.
    //
    // The plant is   m*v_dot = R*e3*T  -  m*g*e3  +  f_ext,   so with
    //
    //     I = integral of (F_applied + f_hat) dt,
    //     f_hat = K_o * ( m*v - m*v_0 - I ),
    //
    // differentiating gives f_hat_dot = K_o * (f_ext - f_hat): a first-order
    // lag of the true disturbance, with no acceleration measurement anywhere.
    //
    // That matters here for a specific reason. The obvious alternative
    // estimates f_ext from measured acceleration, and PX4's velocity carries a
    // BIAS -- 0.20 m/s of reported climb against 0.013 m/s of actual motion has
    // been measured in this stack, and +0.135 m/s is reported with the aircraft
    // stationary on the pad. A constant velocity bias cancels exactly in the
    // momentum difference (m*v - m*v_0), because it is present in both terms.
    // Only a DRIFTING bias reaches the estimate, and slowly.
    //
    // The estimate is deliberately not frozen while the actuators are
    // saturated, unlike the STA state above: this is a filter, not an
    // integrator, so it decays back on its own once the model and the vehicle
    // agree again, and freezing it would hold a stale wind vector through
    // exactly the gust that saturated the allocation. kFextMaxHoverFraction is
    // what bounds the damage instead.
    void updateExternalForceEstimate() {
        if (f_ext_observer_gain_ <= 0.0) {
            f_ext_hat_.setZero();
            return;
        }
        const Eigen::Vector3d momentum = _uav_mass * velocity_W_;
        if (!f_ext_initialised_) {
            // Anchor on the momentum at engagement so the estimate starts at
            // zero rather than at whatever the vehicle was already doing.
            momentum_reference_ = momentum;
            momentum_integral_.setZero();
            f_ext_hat_.setZero();
            f_ext_initialised_ = true;
            return;
        }

        f_ext_hat_ = f_ext_observer_gain_ * (momentum - momentum_reference_ - momentum_integral_);
        const double limit = px4_offboard::kFextMaxHoverFraction * _uav_mass * _gravity;
        for (int i = 0; i < 3; ++i) {
            const double unclamped = f_ext_hat_(i);
            f_ext_hat_(i) = std::clamp(unclamped, -limit, limit);
            // Back-calculate the integral whenever the clamp bites, so the
            // internal residual cannot grow behind a held output. Without this
            // the momentum term keeps accumulating while the estimate sits on
            // the bound, and the observer then needs that whole excess unwound
            // before it can come off it again -- a one-way door of exactly the
            // kind this filter is meant not to have. Reachable in normal flight
            // only ON THE GROUND, where the pad pushes back with a force the
            // bound is deliberately below.
            if (f_ext_hat_(i) != unclamped) {
                momentum_integral_(i) =
                    momentum(i) - momentum_reference_(i) - f_ext_hat_(i) / f_ext_observer_gain_;
            }
        }

        // Advance the integral with the force the vehicle actually had applied
        // over the cycle just gone: the thrust it was commanded, along the
        // attitude it actually held, plus weight.
        const Eigen::Vector3d applied_force =
            R_B_W_.col(2) * last_applied_thrust_
            - _uav_mass * _gravity * Eigen::Vector3d::UnitZ();
        momentum_integral_ += (applied_force + f_ext_hat_) * dt_;
    }

    // The thrust this cycle commanded, kept for the observer's next update.
    void noteAppliedThrust(double thrust) {
        last_applied_thrust_ = thrust;
    }

    // Control loop period, used for the numerical derivatives in the laws.
    double dt_ = px4_offboard::kControlPeriodSeconds;

    // External-force observer state. f_ext_hat_ is in the WORLD frame [N].
    Eigen::Vector3d f_ext_hat_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d momentum_integral_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d momentum_reference_ = Eigen::Vector3d::Zero();
    double f_ext_observer_gain_ = 0.0;
    double last_applied_thrust_ = 0.0;
    bool f_ext_initialised_ = false;

    // Set by the node when the commanded wrench did not survive allocation:
    // a rotor clamped at zero, a throttle outside [0, 1], a tilt on its stop.
    bool actuators_saturated_ = false;

    // Previous desired attitude and first-call guard, used to differentiate the
    // desired attitude into a reference angular velocity. Per-instance so that
    // several controllers can coexist (e.g. in tests) without sharing state.
    Eigen::Matrix3d R_d_prev_ = Eigen::Matrix3d::Identity();
    bool first_iteration_ = true;

    // Diagnostic copies of the last omega_ref and I_a_d. Written by the control
    // laws, read by the node; they take part in no computation.
    Eigen::Vector3d omega_ref_last_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d i_a_d_last_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d s_last_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d s_R_last_ = Eigen::Vector3d::Zero();

    // Reference-rate filter: bandwidth [Hz], 0 disables, plus its state.
    double omega_ref_filter_hz_ = 0.0;
    Eigen::Vector3d omega_ref_filtered_ = Eigen::Vector3d::Zero();

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
