#ifndef DIAGNOSTICS_PUBLISHER_H
#define DIAGNOSTICS_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include <eigen3/Eigen/Eigen>
#include <cmath>
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
    filter_position_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/landing/filter_position", 10);
    filter_sigma_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/landing/filter_sigma", 10);
    filter_nis_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/landing/filter_nis", 10);
    filter_residual_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/landing/filter_residual", 10);
    camera_bias_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/landing/filter_camera_bias", 10);
    tag_health_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/landing/tag_health", 10);
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

  // Publish the in-plane platform yaw taken from the tag: the complementary
  // filter's filtered and raw pair, plus the EKF's psi_pf state. Radians, world
  // frame. All three should read 0 with the tag at the world origin.
  void publishPlatformYaw(double yaw_filtered, double yaw_raw, double yaw_filter_state,
                          uint64_t timestamp) {
    geometry_msgs::msg::Vector3Stamped yaw_msg;
    yaw_msg.header.stamp = rclcpp::Time(timestamp * 1000);
    yaw_msg.header.frame_id = "world";
    yaw_msg.vector.x = yaw_filtered;
    yaw_msg.vector.y = yaw_raw;
    yaw_msg.vector.z = yaw_filter_state;
    platform_yaw_pub_->publish(yaw_msg);
  }

  // ---- Relative-state EKF ----------------------------------------------------
  // Error against groundtruth, in the same sense as the topics above (truth minus
  // estimate), so the two estimators can be scored with one tool.
  void publishFilterPosition(const Eigen::Vector3d& error, uint64_t timestamp) {
    publishVector(filter_position_pub_, error(0), error(1), error(2), timestamp);
  }

  // The uncertainty the filter claims: position sigma per axis, and the platform
  // yaw sigma in z. This is the signal the descent gate and the failsafe tiers
  // are meant to be built on, so it is telemetry from the first flight.
  void publishFilterSigma(const Eigen::Vector3d& sigma, double yaw_sigma, uint64_t timestamp) {
    publishVector(filter_sigma_pub_, sigma(0), sigma(1), yaw_sigma, timestamp);
  }

  // x: normalised innovation squared of the last gated measurement (NIS/dof, so
  // 1 means the covariance is telling the truth). y: its degrees of freedom.
  // z: the fraction of measurements the gate has rejected so far.
  void publishFilterNIS(double normalised_nis, int dof, double reject_fraction,
                        uint64_t timestamp) {
    publishVector(filter_nis_pub_, normalised_nis, static_cast<double>(dof), reject_fraction,
                  timestamp);
  }

  // The reprojection residual, in pixels: what the detector reported minus where
  // the filter thought the corners would be. x is its RMS, y is what S says that
  // should be, z is the corner count. This is the measurement that turns
  // pixel_sigma from an assumption into a number -- and the x/y ratio is the same
  // consistency statement as NIS, in units anyone can picture.
  void publishFilterResidual(double residual_rms, double predicted_rms, int corners,
                             uint64_t timestamp) {
    publishVector(filter_residual_pub_, residual_rms, predicted_rms,
                  static_cast<double>(corners), timestamp);
  }

  // The camera mounting error the filter has learned [m], body axes. A calibration
  // output rather than telemetry: whatever z settles to is a correction to
  // camera_offset_body.z, and it is the number a bench measurement should match.
  void publishCameraBias(const Eigen::Vector3d& bias, double sigma_z, uint64_t timestamp) {
    publishVector(camera_bias_pub_, bias(0), bias(2), sigma_z, timestamp);
  }

  // The tag-loss ladder (item 5 / §07): which tier, how stale the last ACCEPTED
  // measurement is, and how uncertain the estimate has become as a result.
  // Losing the tag used to produce no signal at all -- the descent simply stopped
  // and the aircraft hovered, which from outside is indistinguishable from a
  // vehicle that is merely holding station. x is the tier (0 coast, 1 hold,
  // 2 reacquire, 3 abort), y the age in seconds, z the XY sigma in metres.
  void publishTagHealth(int tier, double age_seconds, double sigma_xy, uint64_t timestamp) {
    // An age of infinity is the correct answer before the first detection and an
    // unplottable one; -1 says "never seen" without putting a non-finite value on
    // a topic every consumer would then have to guard.
    publishVector(tag_health_pub_, static_cast<double>(tier),
                  std::isfinite(age_seconds) ? age_seconds : -1.0, sigma_xy, timestamp);
  }

private:
  void publishVector(const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr& pub,
                     double x, double y, double z, uint64_t timestamp) {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = rclcpp::Time(timestamp * 1000);
    msg.header.frame_id = "world";
    msg.vector.x = x;
    msg.vector.y = y;
    msg.vector.z = z;
    pub->publish(msg);
  }

  rclcpp::Node* node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr groundtruth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr phase_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimated_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr platform_yaw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr filter_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr filter_sigma_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr filter_nis_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr filter_residual_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr camera_bias_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr tag_health_pub_;
};

#endif  // DIAGNOSTICS_PUBLISHER_H
