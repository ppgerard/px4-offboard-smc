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
    StSmcController();
    void calculateControllerOutput(Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion) override;

    // The auxiliary states are the whole memory of this law: carrying them
    // across an offboard entry would engage the aircraft with an integral term
    // wound up against a disturbance it was never flying.
    void reset() override {
        ControllerBase::reset();
        w_.setZero();
        w_R_.setZero();
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

    // Super-twisting gains, rotational
    void setK1R(const Eigen::Vector3d &k1_r) {
        K1_R = k1_r;
    }

    void setK2R(const Eigen::Vector3d &k2_r) {
        K2_R = k2_r;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    // Sliding surface slopes
    Eigen::Vector3d Lambda;
    Eigen::Vector3d Lambda_R;

    // Super-twisting gains
    Eigen::Vector3d K1;
    Eigen::Vector3d K2;
    Eigen::Vector3d K1_R;
    Eigen::Vector3d K2_R;

    // Super-twisting auxiliary (integral) states, persistent across calls
    Eigen::Vector3d w_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_R_ = Eigen::Vector3d::Zero();
};

#endif //CONTROLLER_ST_SMC_CONTROLLER_H
