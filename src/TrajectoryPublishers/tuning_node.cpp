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

// Inspired by https://github.com/Jaeyoung-Lim/mavros_controllers

#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class StepsPublisherNode : public rclcpp::Node {
public:
  StepsPublisherNode() : Node("steps_publisher") {
    publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>("command/trajectory", 10);

    timer_ = this->create_wall_timer(0.01s, std::bind(&StepsPublisherNode::publishTuningSteps, this));
  }

private:
  void publishTuningSteps() {
    
    double x_step = 1.5;  // m
    double y_step = 0.0;  // m
    double z_step = 3.0;  // m
    double pitch_step = 0.0; // rad
    double roll_step = 0.0;  // rad
    double yaw_step = 0.0;   // rad

    // Convert Euler angles to quaternion
    tf2::Quaternion q;
    q.setRPY(roll_step, pitch_step, yaw_step);

    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint traj_point;
    traj_point.time_from_start.sec = 0;
    traj_point.time_from_start.nanosec = 0;

    // Resize vectors for single DOF
    traj_point.transforms.resize(1);
    traj_point.velocities.resize(1);
    // traj_point.accelerations.resize(1);

    // Position
    traj_point.transforms[0].translation.x = x_step;
    traj_point.transforms[0].translation.y = y_step;
    traj_point.transforms[0].translation.z = z_step;

    // Orientation
    traj_point.transforms[0].rotation.x = q.x();
    traj_point.transforms[0].rotation.y = q.y();
    traj_point.transforms[0].rotation.z = q.z();
    traj_point.transforms[0].rotation.w = q.w();

    // Velocity
    traj_point.velocities[0].linear.x = 0.0;
    traj_point.velocities[0].linear.y = 0.0;
    traj_point.velocities[0].linear.z = 0.0;
    // traj_point.velocities[0].angular.x = 0.0;
    // traj_point.velocities[0].angular.y = 0.0;
    // traj_point.velocities[0].angular.z = 0.0;

    // Acceleration
    // traj_point.accelerations[0].linear.x = 0.0;
    // traj_point.accelerations[0].linear.y = 0.0;
    // traj_point.accelerations[0].linear.z = 0.0;
    // traj_point.accelerations[0].angular.x = 0.0;
    // traj_point.accelerations[0].angular.y = 0.0;
    // traj_point.accelerations[0].angular.z = 0.0;

    publisher_->publish(traj_point);

  }

  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StepsPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
