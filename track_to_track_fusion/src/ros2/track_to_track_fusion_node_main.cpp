#include "ros2/fusion_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<track_to_track_fusion::FusionNode>("fusion_node", options));
  rclcpp::shutdown();

  return 0;
}
