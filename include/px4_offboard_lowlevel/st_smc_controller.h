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

#ifndef CONTROLLER_ST_SMC_CONTROLLER_H
#define CONTROLLER_ST_SMC_CONTROLLER_H

#include "px4_offboard_lowlevel/controller_base.h"
#include "px4_offboard_lowlevel/sta_reaching_law.h"

// Classic (2nd-order) Super-Twisting Sliding Mode Controller.
//
// Uses the same translational/rotational sliding surfaces as SmcController:
//   s   = e_v     + Lambda   .* e_p
//   s_R = e_omega + Lambda_R .* e_R
// but replaces the boundary-layer reaching law with the super-twisting
// algorithm, which is continuous (no chattering) and drives both s and its
// derivative to zero in finite time:
//   u = -K1 .* sqrt(|s|) .* sign(s) + w
//   w_dot = -K2 .* sign(s)
class StSmcController : public ControllerBase {
public:
    void setGeometricRot(bool on) { geometric_rot_ = on; }
    void setGeometricGains(const Eigen::Vector3d &kr,
                           const Eigen::Vector3d &kw,
                           const Eigen::Vector3d &ki) {
        K_R_geo_ = kr; K_W_geo_ = kw; K_I_geo_ = ki;
    }
    bool geometricRot() const { return geometric_rot_; }

    StSmcController();
    void calculateControllerOutput(Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion) override;

    // The auxiliary states are the whole memory of this law: carrying them
    // across an offboard entry would engage the aircraft with an integral term
    // wound up against a disturbance it was never flying.
    void reset() override {
        ControllerBase::reset();
        w_.setZero();
        w_R_.setZero();
        e_R_int_.setZero();
        adaptive_.sigma = 0.0;
        adaptive_multiplier_ = 1.0;
    }

    // Sliding surface slopes
    void setLambda(const Eigen::Vector3d &lambda) {
        Lambda = lambda;
    }

    void setLambdaR(const Eigen::Vector3d &lambda_r) {
        Lambda_R = lambda_r;
    }

    // Super-twisting gains, translational
    void setK1(const Eigen::Vector3d &k1) {
        K1 = k1;
    }

    void setK2(const Eigen::Vector3d &k2) {
        K2 = k2;
    }

    // Generalised STA linear terms (roadmap item 10), TRANSLATIONAL. Zero is the
    // classical law, bit-exact. Their purpose here is NOT quiet: the sqrt branch
    // produces very little FORCE at small |s| (0.7*sqrt(0.1) = 0.22 N), and
    // against a disturbance that grows with displacement the law needs real
    // authority early. -K3*s is a linear term that supplies it.
    void setK3(const Eigen::Vector3d &k3) {
        K3 = k3;
    }

    void setK4(const Eigen::Vector3d &k4) {
        K4 = k4;
    }

    // Uniform / fixed-time STA: the |s|^{3/2} coefficient. Zero = classical.
    void setBeta(const Eigen::Vector3d &b) { Beta = b; }
    // Super-twisting gains, rotational
    void setK1R(const Eigen::Vector3d &k1_r) {
        K1_R = k1_r;
    }

    void setK2R(const Eigen::Vector3d &k2_r) {
        K2_R = k2_r;
    }

    // Barrier-function adaptive gain on the TRANSLATIONAL XY pair only.
    // l_max <= 1.0 (the shipped default) disables it and leaves K1/K2
    // bit-for-bit as loaded, so it A/Bs against one binary.
    // Saturation of the position term inside the sliding surface [m]. 0 = off.
    void setStaEpMax(double m) {
        sta_ep_max_ = (m > 0.0) ? m : 0.0;
    }

    // Translational STA integral bound, as a fraction of hover thrust.
    void setStaWLimitFraction(double f) {
        if (f > 0.0) { sta_w_limit_fraction_ = f; }
    }

    void setAdaptiveGain(const px4_offboard::BarrierGain &g) {
        adaptive_ = g;
    }

    // Last multiplier applied; 1.0 means fixed-gain. Diagnostic only.
    double adaptiveMultiplier() const {
        return adaptive_multiplier_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    // Sliding surface slopes
    Eigen::Vector3d Lambda;
    Eigen::Vector3d Lambda_R;

    // Super-twisting gains
    Eigen::Vector3d K1;
    Eigen::Vector3d K2;
    Eigen::Vector3d K3 = Eigen::Vector3d::Zero();
    Eigen::Vector3d K4 = Eigen::Vector3d::Zero();
    Eigen::Vector3d Beta = Eigen::Vector3d::Zero();
    Eigen::Vector3d K1_R;
    Eigen::Vector3d K2_R;

    // ---- Geometric PID alternative for the ROTATIONAL loop -------------------
    // The attitude loop is measurably NOT the limiting factor (thrust-direction
    // error 0.2 deg p50, 0% motor saturation), so this is not a performance fix.
    // It exists because the rotational STA runs at HALF authority for chattering
    // reasons -- STA_K1_R was halved 0.5 -> 0.25 to get 4/4 problematic down to
    // 0/4, and quartering it lands 0/3 tail-on. A non-switching law carries no
    // such trade. Gains are sized from the SAME bandwidth Lambda_R already sets
    // (kR = J*wn^2, kW = 2*zeta*J*wn) so the comparison is of reaching laws, not
    // of tunings.
    //
    // PID, not PD: K2_R's job is cancelling the steady aerodynamic moment, and
    // removing it costs tail-on landings. The integral carries the same bound
    // and anti-windup as the STA's w_R_, so neither arm gets freer integral
    // action than the other.
    //
    // geometric_rot_ = false is BIT-EXACT the super-twisting law.
    bool geometric_rot_ = false;
    Eigen::Vector3d K_R_geo_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d K_W_geo_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d K_I_geo_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d e_R_int_ = Eigen::Vector3d::Zero();

    // Super-twisting auxiliary (integral) states, persistent across calls
    Eigen::Vector3d w_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_R_ = Eigen::Vector3d::Zero();

    // Adaptive-gain state (the filtered |s_xy|) and its last output.
    double sta_w_limit_fraction_ = px4_offboard::kStaThrustHoverFraction;
    double sta_ep_max_ = 0.0;   // [m] surface position saturation; 0 = off
    px4_offboard::BarrierGain adaptive_;
    double adaptive_multiplier_ = 1.0;
};

#endif //CONTROLLER_ST_SMC_CONTROLLER_H
