// AprilTag landing trajectory node that bypasses the SMC controller and
// publishes PX4 OffboardControlMode/TrajectorySetpoint directly, for
// baseline/comparison testing against the SMC-controlled landing path.
// Shared tag-fusion/phase logic lives in landing_trajectory_base.h.

#include <limits>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include "landing_trajectory_base.h"

class Px4OffboardLandingNode : public LandingTrajectoryNodeBase {
public:
  Px4OffboardLandingNode() : LandingTrajectoryNodeBase("px4_offboard_trajectory_publisher") {
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>
        ("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>
        ("/fmu/in/trajectory_setpoint", 10);

    offboard_timer_ = this->create_wall_timer(0.33s, std::bind(&Px4OffboardLandingNode::publishOffboardControlModeMsg, this));
  }

protected:
  void onSetpointPublished() override {
    publishTrajectorySetpoint();
  }

private:
  void publishOffboardControlModeMsg() {
    px4_msgs::msg::OffboardControlMode offboard_msg{};
    offboard_msg.position = true;
    offboard_msg.velocity = true;
    offboard_msg.acceleration = true;
    offboard_msg.body_rate = false;
    offboard_msg.attitude = false;
    offboard_msg.thrust_and_torque = false;
    offboard_msg.direct_actuator = false;
    offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(offboard_msg);
    RCLCPP_INFO_ONCE(get_logger(), "Offboard enabled");
  }

  void publishTrajectorySetpoint() {
    // Publish the current trajectory setpoint (position, velocity, acceleration) to PX4 in NED frame
    px4_msgs::msg::TrajectorySetpoint setpoint{};

    Eigen::Vector3d position_NED = px4_frames::rotateVectorFromToENU_NED(r_position_W_);
    Eigen::Vector3d velocity_NED = px4_frames::rotateVectorFromToENU_NED(r_velocity_W_);
    Eigen::Vector3d acceleration_NED = px4_frames::rotateVectorFromToENU_NED(r_acceleration_W_);

    setpoint.position[0] = position_NED(0);
    setpoint.position[1] = position_NED(1);
    setpoint.position[2] = position_NED(2);

    setpoint.velocity[0] = velocity_NED(0);
    setpoint.velocity[1] = velocity_NED(1);
    setpoint.velocity[2] = velocity_NED(2);

    setpoint.acceleration[0] = acceleration_NED(0);
    setpoint.acceleration[1] = acceleration_NED(1);
    setpoint.acceleration[2] = acceleration_NED(2);

    // No yaw setpoint
    setpoint.yaw = std::numeric_limits<float>::quiet_NaN();
    setpoint.yawspeed = std::numeric_limits<float>::quiet_NaN();

    setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    trajectory_setpoint_publisher_->publish(setpoint);
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::TimerBase::SharedPtr offboard_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Px4OffboardLandingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
