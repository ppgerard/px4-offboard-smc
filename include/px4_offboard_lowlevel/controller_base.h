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
    // ---- Solution B: online control-EFFECTIVENESS estimate --------------------
    //
    // The horizontal force actually delivered is  F_net(theta) = T*sin(theta) -
    // F_aero(theta, V_air), and on this airframe the second term grows almost as
    // fast as the first: measured net force is ~1 N at EVERY lean from 7 to 20
    // deg (see the self-inflicted-disturbance section in CLAUDE.md). So the
    // effective input gain
    //
    //     b_eff(theta) = d/dtheta [ T*sin(theta) - F_aero ] = T*cos(theta) - g
    //
    // collapses toward zero, which breaks the assumption every sliding-mode
    // proof makes (b bounded away from zero) and is why no reaching gain,
    // surface slope, integral bound or adaptation law moved this failure.
    //
    // g = d|F_aero|/dtheta is estimated ONLINE by recursive least squares on the
    // pairs (lean, |f_ext_hat_xy|) the aircraft already produces -- no aero
    // model, no wind estimator, no new sensor. The lean is then limited to where
    // b_eff still exceeds a floor, which is a DERIVED, airspeed-dependent tilt
    // limit rather than a guessed constant (and explains why a fixed 10 deg cap
    // helped at 7.5 m/s and hurt at 5 m/s).
    //
    // aero_rls_forget <= 0 disables the whole path, bit-exactly.
    // Estimate the EFFECTIVE INPUT GAIN online:
    //
    //     b_hat = d(m * a_achieved) / d(F_commanded)      [dimensionless, ideal 1]
    //
    // This is strictly better than regressing f_ext on lean, because one slope
    // absorbs EVERY corruption of the input path at once:
    //   * ct error   -- a multiplicative input-gain error IS this quantity
    //   * the aero feedback, dd/du, which is why b_eff collapses here
    //   * thrust-model error, battery sag, allocation shortfall
    //
    // Both signals already exist: the commanded horizontal force is I_a_d, and
    // the achieved acceleration is the filtered derivative of velocity_W_. The
    // achieved acceleration is PROJECTED on the commanded force direction, so
    // the regression measures delivery along the axis actually being commanded.
    //
    // b_hat << 1 means "commanding more buys nothing" -- the condition every
    // sliding-mode proof assumes away. The lean is then bounded to where the
    // command still pays, which is a derived, airspeed-dependent limit.
    void updateInputGain(const Eigen::Vector3d &force_cmd_W, double dt) {
        if (aero_rls_forget_ <= 0.0 || dt <= 1e-6) {
            return;
        }
        // filtered achieved acceleration
        const Eigen::Vector3d accel = (velocity_W_ - velocity_prev_) / dt;
        velocity_prev_ = velocity_W_;
        accel_filt_ += 0.1 * (accel - accel_filt_);

        const double f_mag = force_cmd_W.head<2>().norm();
        if (f_mag < 0.5) {                 // no excitation, do not adapt
            return;
        }
        const Eigen::Vector2d dir = force_cmd_W.head<2>() / f_mag;
        const double y = _uav_mass * accel_filt_.head<2>().dot(dir);

        // RLS with forgetting on  y = b*f_mag + c
        const double x0 = f_mag, x1 = 1.0;
        const double denom = aero_rls_forget_
            + (x0 * (aero_P_[0] * x0 + aero_P_[1] * x1)
             + x1 * (aero_P_[2] * x0 + aero_P_[3] * x1));
        if (!(denom > 1e-9)) {
            return;
        }
        const double k0 = (aero_P_[0] * x0 + aero_P_[1] * x1) / denom;
        const double k1 = (aero_P_[2] * x0 + aero_P_[3] * x1) / denom;
        const double err = y - (aero_g_ * f_mag + aero_a_);
        aero_g_ += k0 * err;               // aero_g_ is now b_hat
        aero_a_ += k1 * err;
        const double p00 = aero_P_[0], p01 = aero_P_[1], p10 = aero_P_[2], p11 = aero_P_[3];
        aero_P_[0] = (p00 - k0 * (x0 * p00 + x1 * p10)) / aero_rls_forget_;
        aero_P_[1] = (p01 - k0 * (x0 * p01 + x1 * p11)) / aero_rls_forget_;
        aero_P_[2] = (p10 - k1 * (x0 * p00 + x1 * p10)) / aero_rls_forget_;
        aero_P_[3] = (p11 - k1 * (x0 * p01 + x1 * p11)) / aero_rls_forget_;
        aero_g_ = std::clamp(aero_g_, 0.0, 2.0);

        // Bound the lean by how much of the command is actually delivered. At
        // b_hat = 1 the full envelope is available; as b_hat -> 0 commanding more
        // lean only manufactures more disturbance, so pull the bound in.
        const double target = 0.12 + 0.50 * aero_g_;        // [rad], 6.9 deg .. 35 deg
        aero_tilt_max_rad_ += 0.01 * (target - aero_tilt_max_rad_);
    }

    double inputGainEstimate() const { return aero_g_; }
    double aeroTiltMaxRad() const { return aero_tilt_max_rad_; }

    void setAeroRlsForget(double f) { aero_rls_forget_ = (f > 0.0 && f <= 1.0) ? f : 0.0; }

    // Commanded-lean limit [deg]; 0 disables. See limitTilt().
    void setTiltMaxDeg(double deg) {
        tilt_max_rad_ = (deg > 0.0) ? deg * M_PI / 180.0 : 0.0;
    }

    // The force the ALLOCATOR actually delivered along body x last cycle. The
    // attitude must be chosen BEFORE the allocation runs, so the two cannot be
    // solved together without iterating the whole loop; feeding the achieved
    // value back one cycle later closes it instead. At 100 Hz that is 10 ms
    // against a translational loop at ~1.5 rad/s -- about 0.9 deg of phase.
    //
    // ACHIEVED, not demanded, is the point: if the tilt saturated or the servo
    // rate-limited, the attitude picks up exactly the remainder, and the two
    // actuators cannot both claim the same newton. Zero unless the QP allocator
    // is on, so the laws are bit-exact by default.
    void setServedBodyX(double f) { served_body_x_ = f; }
    double servedBodyX() const { return served_body_x_; }

    // ---- The CLOSED-FORM lean/tilt split (supersedes the feedback form above) ----
    //
    // Enable with a positive tilt authority in degrees; 0.0 leaves both laws
    // bit-exact. When on, setServedBodyX is not used at all.
    void setTiltSplitMaxDeg(double deg) { tilt_split_max_rad_ = deg * M_PI / 180.0; }
    // The BACKWARD bound, which is a different number from the forward one. The
    // nacelles tilt from hover toward forward flight, so the servo range is
    // [-7, +90] deg: a headwind is served on the generous side and a tailwind on
    // the 7 deg side. Promising the allocator a share it cannot reach would put
    // the deficit back on the attitude a cycle late, which is the loop this
    // whole construction exists to avoid. Defaults to symmetric.
    void setTiltSplitMinDeg(double deg) { tilt_split_min_rad_ = deg * M_PI / 180.0; }
    // Angle RESERVED for yaw, and it is not optional. Yaw on this airframe IS
    // differential tilt, so the two nacelles are never at the same angle: when
    // one is on its stop the other is short by the full differential, and the
    // COMMON-MODE angle -- the only part that makes body-x force -- is the mean
    // of the two, not the stop.
    //
    // Measured offline against the allocator at a 15 deg stop: the rotor-drag
    // term alone holds 2.37 / -2.39 deg at hover (km 0.018), so the pair sits at
    // 15.00 / 12.50 and the mean is 13.75. A split sized on 15 deg promised
    // 4.21 N and the allocator delivered 4.09 -- and the 0.12 N difference goes
    // nowhere, because the attitude was already told to serve the remainder.
    // Sizing the share on what the PAIR can hold is what keeps the split honest.
    void setTiltSplitYawReserveDeg(double deg) {
        tilt_split_yaw_reserve_rad_ = std::fabs(deg) * M_PI / 180.0;
    }
    // The body-x force the split ASSIGNED to the tilt this cycle. The allocator
    // is handed this as its F_x demand, so both actuators are working from one
    // decision made in one cycle.
    double commandedBodyX() const { return commanded_body_x_; }

    // Divide the horizontal force demand between LEAN and TILT, in closed form.
    //
    // In the heading-aligned frame the demand is (fx, fy, fz), and only ONE of
    // those three is contestable. Both nacelles rotate about body y, so the tilt
    // makes body-x force and nothing else: fy has to come from roll and fz from
    // thrust whatever happens. fx is therefore the entire free choice, and the
    // split is one scalar.
    //
    //     X     = clamp(fx, +-X_max),   X_max = (2/3) fz sin(t_max)
    //     attitude gets  (fx - X, fy, fz)
    //
    // Serving fx with tilt FIRST is the minimum-lean solution: the same force at
    // less angle of attack. X_max carries the 2/3 because only the two front
    // rotors tilt, which is the same factor test/qp_allocator_test.cpp asserts
    // against the allocator's own wrench.
    //
    // Nothing here reads back what the allocator did. The earlier form fed the
    // achieved value forward one cycle (see desiredForceBody), which made the
    // split a discrete loop with eigenvalue -alpha in the F_x weight: marginal
    // at alpha ~ 1, and the measured result was bimodal -- 5.5/6.5/6.7 cm when
    // it did not trip, 147-165 cm when it did. A rule computed from the demand
    // has no eigenvalue to place.
    //
    // The small-angle step is self-consistent where it matters: body x equals
    // heading x exactly when the tilt covers all of fx, because the residual
    // pitch is then zero. The approximation is tightest in the regime it is for.
    Eigen::Vector3d splitTiltShare(const Eigen::Vector3d& I_a_d, double yaw) {
        commanded_body_x_ = 0.0;
        if (tilt_split_max_rad_ <= 0.0) { return I_a_d; }
        const double c = std::cos(yaw), sn = std::sin(yaw);
        const double fx =  c * I_a_d.x() + sn * I_a_d.y();
        const double fy = -sn * I_a_d.x() + c * I_a_d.y();
        const double fz =  I_a_d.z();
        // A non-positive vertical demand is a descent the tilt cannot help with,
        // and it would invert the sign of X_max. Leave the demand alone.
        if (fz <= 0.0) { return I_a_d; }
        // Both bounds shrink toward zero by the yaw reserve, never past it.
        const double t_fwd = std::max(0.0, tilt_split_max_rad_ - tilt_split_yaw_reserve_rad_);
        const double t_bwd = (tilt_split_min_rad_ < 0.0)
                                 ? std::min(0.0, tilt_split_min_rad_ + tilt_split_yaw_reserve_rad_)
                                 : -t_fwd;
        const double x_max = (2.0 / 3.0) * fz * std::sin(t_fwd);
        const double x_min = (2.0 / 3.0) * fz * std::sin(t_bwd);
        const double X = std::max(x_min, std::min(fx, x_max));
        commanded_body_x_ = X;
        const double rx = fx - X;
        return Eigen::Vector3d(c * rx - sn * fy, sn * rx + c * fy, fz);
    }
    // Desired force in BODY axes, for the allocator's F_x / F_z rows.
    // Body-frame force demand handed to the allocator: the REMAINDER after the
    // tilt's last contribution, not the full demand.
    //
    // This makes the split a stable feedback loop rather than a static division.
    // With alpha the fraction the allocator actually serves,
    //
    //     S_{n+1} = alpha * (F - S_n)      fixed point S* = alpha*F/(1+alpha)
    //                                      eigenvalue -alpha
    //
    // so it CONVERGES for alpha < 1 and is marginal at alpha ~ 1. The F_x weight
    // in AllocWeights is exactly that gain: at w_Fx = 1.0 the allocator served
    // nearly everything asked, alpha ~ 1, and the result was the marginal
    // oscillation measured 11 Sep (bimodal -- 5.5/6.5/6.7 cm when it did not
    // trip, 147-165 cm when it did). At w_Fx = 0.5 the loop self-limits.
    //
    // Feeding the FULL demand instead was tried and is worse: algebraically the
    // total force is the same either way, but it puts the MAXIMUM on the tilt
    // rather than letting the split settle, and it landed 0/3 at the same tilt
    // authority the 3x3 path flies.
    Eigen::Vector3d desiredForceBody() const { return R_B_W_.transpose() * i_a_d_last_; }


    // Bound on the external-force estimate, as a fraction of hover thrust.
    void setFextMaxFraction(double f) {
        if (f > 0.0) { fext_max_fraction_ = f; }
    }

    // SEPARATE horizontal bound. The clamp exists to stop the observer running
    // away when the PAD PUSHES BACK, and that reaction is VERTICAL -- but the
    // bound was applied per-axis with one value, so covering the wind meant
    // also letting the ground reaction in. Measured 9 Sep at 7.5 m/s / 90 deg:
    // the true aero force is 8.28 N against a 7.32 N per-axis bound, so f_ext_y
    // sits ON the clamp and the feedforward is 0.96 N short. That deficit is
    // 0.386 m/s^2, which over the 2.02 s commit descent is 0.79 m -- against a
    // MEASURED commit drift of 0.68 m. The landing is lost to a saturated
    // estimator, not to a reaching gain.
    // 0 (default) means "same as the vertical bound", i.e. bit-exact the old law.
    void setFextMaxFractionXy(double f) {
        if (f > 0.0) { fext_max_fraction_xy_ = f; }
    }

    void setActuatorsSaturated(bool saturated) {
        actuators_saturated_ = saturated;
        actuators_saturated_trans_ = saturated;
    }

    // Split form. The tilt servos are the YAW axis on this tricopter, so their
    // clamp and rate limiter say nothing about whether the TRANSLATIONAL wrench
    // reached the rotors -- yet both used to feed one flag that froze both
    // super-twisting integrals. A steady lateral wind is exactly the case where
    // the tilt works hardest AND the translational integral is most needed, so
    // the coupling starves the loop precisely when it matters. Pass the
    // translational flag without the tilt terms to break it.
    void setActuatorsSaturated(bool rotational, bool translational) {
        actuators_saturated_ = rotational;
        actuators_saturated_trans_ = translational;
    }

    bool actuatorsSaturatedTranslational() const {
        return actuators_saturated_trans_;
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
    //
    // velocity_W is already in world axes: px4_frame_conversions.h resolves the
    // message's velocity_frame. It used to arrive here as a body-frame quantity
    // and be rotated by R_B_W_, which is correct only at zero heading -- the one
    // condition SITL always satisfies and hardware never does. See the note on
    // eigenOdometryFromPX4Msg().
    void setOdometry(const Eigen::Vector3d &position_W, const Eigen::Quaterniond &orientation_B_W,
        const Eigen::Vector3d &velocity_W, const Eigen::Vector3d &angular_velocity_B){
        R_B_W_ = orientation_B_W.toRotationMatrix();
        position_W_ = position_W;
        velocity_W_ = velocity_W;
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
    // Limit the commanded LEAN, the way PX4's MPC_TILTMAX_AIR does. Neither SMC
    // law had any such limit, and on this airframe that is a positive feedback
    // loop rather than merely an aggressive one:
    //
    //   MEASURED 9 Sep, 7.5 m/s at 90 deg, from the BodyAero plugin's OWN
    //   applied wrench (gz /t2/body_aero, no inference):
    //     PX4    leans  7.1 deg -> plugin applies 3.02 N median -> lands 1.01 cm
    //     STSMC  leans 19.5 deg -> plugin applies 7.97 N median (20.8 max) -> departs
    //     SMC    leans 16-19 deg -> 5.8-8.4 N median -> departs
    //
    // The T2 carries 0.547 m2 of wing in that plugin, so lean changes the angle
    // of attack and sideslip and the aircraft MAKES its own disturbance. More
    // lean buys more force to fight, which is why no reaching gain, surface
    // slope, integral bound or reference scheme changed anything: they all act
    // by leaning harder. There are two equilibria and the limit is what keeps
    // the aircraft in the benign one.
    //
    // 0 disables it and is bit-exact the previous law.
    Eigen::Vector3d limitTilt(const Eigen::Vector3d &I_a_d) const {
        const double bound = (aero_rls_forget_ > 0.0)
            ? ((tilt_max_rad_ > 0.0) ? std::min(tilt_max_rad_, aero_tilt_max_rad_)
                                     : aero_tilt_max_rad_)
            : tilt_max_rad_;
        if (bound <= 0.0) {
            return I_a_d;
        }
        const double vertical = std::max(I_a_d.z(), 0.1 * _uav_mass * _gravity);
        const double horizontal_max = vertical * std::tan(bound);
        Eigen::Vector3d limited = I_a_d;
        const double horizontal = limited.head<2>().norm();
        if (horizontal > horizontal_max && horizontal > 1e-9) {
            limited.head<2>() *= horizontal_max / horizontal;
        }
        return limited;
    }

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
        // MEASURED PINNED IN FLIGHT (9 Sep). At 7.5 m/s / 90 deg the estimate sits
        // at 7.317/7.378/7.401 N for p50/p95/max against a 7.32 N bound -- saturated
        // for the whole flight, not 'only on the ground' as this file assumed. A
        // saturated observer feeds forward a CONSTANT force (16.7 deg of lean) and
        // stops responding to the wind at all, which is why the commanded lean was
        // invariant at ~19 deg across every gain, Lambda, integral bound and
        // reference scheme tried.
        const double limit_z = fext_max_fraction_ * _uav_mass * _gravity;
        const double limit_xy = (fext_max_fraction_xy_ > 0.0)
            ? fext_max_fraction_xy_ * _uav_mass * _gravity : limit_z;
        for (int i = 0; i < 3; ++i) {
            const double limit = (i < 2) ? limit_xy : limit_z;
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
    double fext_max_fraction_ = px4_offboard::kFextMaxHoverFraction;
    double fext_max_fraction_xy_ = 0.0;   // 0 = use fext_max_fraction_
    double tilt_max_rad_ = 0.0;           // commanded lean limit; 0 = unlimited
    double served_body_x_ = 0.0;          // body-x force the allocator delivered
    double tilt_split_max_rad_ = 0.0;     // >0 enables the closed-form lean/tilt split
    double tilt_split_min_rad_ = 0.0;     // backward bound; 0 = symmetric
    double tilt_split_yaw_reserve_rad_ = 0.0;  // angle kept for differential (yaw)
    double commanded_body_x_ = 0.0;       // body-x the split ASSIGNED to the tilt
    Eigen::Vector3d i_a_d_full_ = Eigen::Vector3d::Zero();  // demand BEFORE the tilt subtraction
    double aero_rls_forget_ = 0.0;          // 0 disables solution B
    double aero_a_ = 0.0, aero_g_ = 0.0;    // f_aero ~ a + g*lean
    double aero_P_[4] = {10.0, 0.0, 0.0, 10.0};
    Eigen::Vector3d velocity_prev_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_filt_ = Eigen::Vector3d::Zero();
    double aero_tilt_max_rad_ = 0.35;       // derived authority bound [rad]
    bool actuators_saturated_ = false;
    bool actuators_saturated_trans_ = false;

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
