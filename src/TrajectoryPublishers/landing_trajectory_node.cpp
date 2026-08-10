// AprilTag landing trajectory node: publishes MultiDOFJointTrajectoryPoint
// setpoints on "command/trajectory" for the SMC offboard_controller_node to
// track. Shared tag-fusion/phase logic lives in landing_trajectory_base.h.

#include "landing_trajectory_base.h"

class LandingTrajectoryPublisherNode : public LandingTrajectoryNodeBase {
public:
  LandingTrajectoryPublisherNode() : LandingTrajectoryNodeBase("landing_trajectory_publisher") {}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LandingTrajectoryPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
