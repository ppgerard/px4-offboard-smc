#ifndef DIAGNOSTICS_PUBLISHER_H
#define DIAGNOSTICS_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include <eigen3/Eigen/Eigen>
#include <memory>

class DiagnosticsPublisher {
public:
  DiagnosticsPublisher(rclcpp::Node* node) : node_(node) {
    // Publishers for diagnostic topics
    odometry_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/landing/odometry", 10);
    groundtruth_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/landing/groundtruth", 10);
    phase_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/landing/phase", 10);
    estimated_position_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/landing/estimated_position", 10);
    position_raw_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/landing/position_raw", 10);
    platform_yaw_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/landing/platform_yaw", 10);
  }

  // Publish current odometry state
  void publishOdometry(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                      const Eigen::Quaterniond& orientation, const Eigen::Vector3d& angular_velocity,
                      uint64_t timestamp) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = rclcpp::Time(timestamp * 1000);
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "base_link";

    // Position
    odom_msg.pose.pose.position.x = position(0);
    odom_msg.pose.pose.position.y = position(1);
    odom_msg.pose.pose.position.z = position(2);

    // Orientation
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();

    // Velocity
    odom_msg.twist.twist.linear.x = velocity(0);
    odom_msg.twist.twist.linear.y = velocity(1);
    odom_msg.twist.twist.linear.z = velocity(2);

    // Angular velocity
    odom_msg.twist.twist.angular.x = angular_velocity(0);
    odom_msg.twist.twist.angular.y = angular_velocity(1);
    odom_msg.twist.twist.angular.z = angular_velocity(2);

    odometry_pub_->publish(odom_msg);
  }

  // Publish groundtruth from VehicleLocalPosition
  void publishGroundtruth(
        const Eigen::Vector3d& position_W,
        const Eigen::Vector3d& velocity_W,
        uint64_t timestamp)
    {
        nav_msgs::msg::Odometry gt_msg;

        gt_msg.header.stamp = rclcpp::Time(timestamp * 1000);
        gt_msg.header.frame_id = "world";
        gt_msg.child_frame_id = "base_link";

        // Position ENU
        gt_msg.pose.pose.position.x = position_W(0);
        gt_msg.pose.pose.position.y = position_W(1);
        gt_msg.pose.pose.position.z = position_W(2);

        // Orientation
        gt_msg.pose.pose.orientation.x = 0.0;
        gt_msg.pose.pose.orientation.y = 0.0;
        gt_msg.pose.pose.orientation.z = 0.0;
        gt_msg.pose.pose.orientation.w = 1.0;

        // Velocity ENU
        gt_msg.twist.twist.linear.x = velocity_W(0);
        gt_msg.twist.twist.linear.y = velocity_W(1);
        gt_msg.twist.twist.linear.z = velocity_W(2);

        // Angular velocity
        gt_msg.twist.twist.angular.x = 0.0;
        gt_msg.twist.twist.angular.y = 0.0;
        gt_msg.twist.twist.angular.z = 0.0;

        groundtruth_pub_->publish(gt_msg);
 }

  // Publish current phase (1, 2, or 3)
  void publishPhase(int phase, uint64_t timestamp) {
    geometry_msgs::msg::Vector3Stamped phase_msg;
    phase_msg.header.stamp = rclcpp::Time(timestamp * 1000);
    phase_msg.header.frame_id = "world";
    phase_msg.vector.x = static_cast<double>(phase);
    phase_msg.vector.y = 0.0;
    phase_msg.vector.z = 0.0;
    phase_pub_->publish(phase_msg);
  }

  // Publish estimated position (fused estimate)
  void publishEstimatedPosition(const Eigen::Vector3d& estimated_position, uint64_t timestamp) {
    geometry_msgs::msg::Vector3Stamped pos_msg;
    pos_msg.header.stamp = rclcpp::Time(timestamp * 1000);
    pos_msg.header.frame_id = "world";
    pos_msg.vector.x = estimated_position(0);
    pos_msg.vector.y = estimated_position(1);
    pos_msg.vector.z = estimated_position(2);
    estimated_position_pub_->publish(pos_msg);
  }

  // Publish raw (unfiltered) tag position
  void publishPositionRaw(const Eigen::Vector3d& position_raw, uint64_t timestamp) {
    geometry_msgs::msg::Vector3Stamped pos_msg;
    pos_msg.header.stamp = rclcpp::Time(timestamp * 1000);
    pos_msg.header.frame_id = "world";
    pos_msg.vector.x = position_raw(0);
    pos_msg.vector.y = position_raw(1);
    pos_msg.vector.z = position_raw(2);
    position_raw_pub_->publish(pos_msg);
  }

  // Publish the in-plane platform yaw taken from the tag: filtered and raw, so
  // the pair shows how much of the raw signal is noise. Radians, world frame.
  void publishPlatformYaw(double yaw_filtered, double yaw_raw, uint64_t timestamp) {
    geometry_msgs::msg::Vector3Stamped yaw_msg;
    yaw_msg.header.stamp = rclcpp::Time(timestamp * 1000);
    yaw_msg.header.frame_id = "world";
    yaw_msg.vector.x = yaw_filtered;
    yaw_msg.vector.y = yaw_raw;
    yaw_msg.vector.z = 0.0;
    platform_yaw_pub_->publish(yaw_msg);
  }

private:
  rclcpp::Node* node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr groundtruth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr phase_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimated_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr platform_yaw_pub_;
};

#endif  // DIAGNOSTICS_PUBLISHER_H
