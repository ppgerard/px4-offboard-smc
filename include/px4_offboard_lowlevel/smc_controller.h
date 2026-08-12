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

#ifndef CONTROLLER_SMC_CONTROLLER_H
#define CONTROLLER_SMC_CONTROLLER_H

#include "px4_offboard_lowlevel/controller_base.h"

class SmcController : public ControllerBase {
public:
    SmcController();
    void calculateControllerOutput(Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion) override;

    // SMC Controller Parameter Setters
    void setLambda(const Eigen::Vector3d &lambda) {
        Lambda = lambda;
    }

    void setKs(const Eigen::Vector3d &ks) {
        K_s = ks;
    }

    void setPhi(const Eigen::Vector3d &phi_val) {
        phi = phi_val;
    }

    void setLambdaR(const Eigen::Vector3d &lambda_r) {
        Lambda_R = lambda_r;
    }

    void setKsR(const Eigen::Vector3d &ks_r) {
        K_s_R = ks_r;
    }

    void setPhiR(const Eigen::Vector3d &phi_r) {
        phi_R = phi_r;
    }

    // void setActualPosition(const Eigen::Vector3d &position_W) {
    //     position_W_ = position_W;
    // }

    // ===== OLD LEE CONTROLLER GAINS (COMMENTED OUT - NO LONGER USED) =====
    void setKPositionGain(const Eigen::Vector3d &PositionGain){
        position_gain_ = PositionGain;
    }
    
    void setKVelocityGain(const Eigen::Vector3d &VelocityGain){
        velocity_gain_ = VelocityGain;
    }
    
    void setKAttitudeGain(const Eigen::Vector3d &AttitudeGain){
        attitude_gain_ = AttitudeGain;
    }
    
    void setKAngularRateGain(const Eigen::Vector3d &AngularRateGain){
        angular_rate_gain_ = AngularRateGain;
    }
    // ===== OLD LEE CONTROLLER GAINS (COMMENTED OUT - NO LONGER USED) =====
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    // SMC Controller Gains
    Eigen::Vector3d Lambda;
    Eigen::Vector3d K_s;
    Eigen::Vector3d phi;
    Eigen::Vector3d Lambda_R;
    Eigen::Vector3d K_s_R;
    Eigen::Vector3d phi_R;

    // ===== OLD LEE CONTROLLER GAINS (COMMENTED OUT - NO LONGER USED) =====
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    Eigen::Vector3d angular_rate_gain_;
    // ===== OLD LEE CONTROLLER GAINS (COMMENTED OUT - NO LONGER USED) =====
};

#endif //CONTROLLER_SMC_CONTROLLER_H
