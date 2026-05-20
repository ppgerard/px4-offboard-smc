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

using namespace std::chrono_literals;

class CirclePublisherNode : public rclcpp::Node {
public:
  CirclePublisherNode() : Node("circle_publisher") {
    publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>("command/trajectory", 10);

    timer_ = this->create_wall_timer(0.01s, std::bind(&CirclePublisherNode::publishCircleTrajectory, this));
  }

private:
  void publishCircleTrajectory() {
    static double angle = 0.0;
    
    // Trajectory parameters
    double radius = 2.0;        // meters
    double altitude = 2.0;      // meters
    double angle_rate = 0.3;    // rad/s (angular velocity)
    double dt = 0.01;           // seconds (100 Hz publishing rate)
    
    // Derived quantities (dependent on above)
    double angle_increment = angle_rate * dt;  // rad per step
    double linear_speed = radius * angle_rate;  // m/s
    double centripetal_accel = angle_rate * angle_rate;  // m/s² magnitude

    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint traj_point;
    traj_point.time_from_start.sec = 0;
    traj_point.time_from_start.nanosec = 0;

    // Resize vectors for single DOF
    traj_point.transforms.resize(1);
    traj_point.velocities.resize(1);
    traj_point.accelerations.resize(1);

    // Position: circular path in x-y plane
    traj_point.transforms[0].translation.x = radius * cos(angle);
    traj_point.transforms[0].translation.y = radius * sin(angle);
    traj_point.transforms[0].translation.z = altitude;

    // Orientation: neutral (identity quaternion)
    traj_point.transforms[0].rotation.x = 0.0;
    traj_point.transforms[0].rotation.y = 0.0;
    traj_point.transforms[0].rotation.z = 0.0;
    traj_point.transforms[0].rotation.w = 1.0;

    // Velocity: tangent to circle (perpendicular to radius)
    traj_point.velocities[0].linear.x = -linear_speed * sin(angle);
    traj_point.velocities[0].linear.y = linear_speed * cos(angle);
    traj_point.velocities[0].linear.z = 0.0;
    traj_point.velocities[0].angular.x = 0.0;
    traj_point.velocities[0].angular.y = 0.0;
    traj_point.velocities[0].angular.z = 0.0;

    // Acceleration: centripetal (points toward center, perpendicular to velocity)
    traj_point.accelerations[0].linear.x = -radius * centripetal_accel * cos(angle);
    traj_point.accelerations[0].linear.y = -radius * centripetal_accel * sin(angle);
    traj_point.accelerations[0].linear.z = 0.0;
    traj_point.accelerations[0].angular.x = 0.0;
    traj_point.accelerations[0].angular.y = 0.0;
    traj_point.accelerations[0].angular.z = 0.0;

    publisher_->publish(traj_point);

    angle += angle_increment;  // Update angle based on rate
  }

  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CirclePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
