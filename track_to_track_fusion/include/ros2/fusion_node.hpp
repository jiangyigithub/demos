#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// #include <ros/ros.h>
// #include <tf/transform_listener.h>
#pragma GCC diagnostic pop

#include "rclcpp/rclcpp.hpp"
#include <perception_kit_msgs/msg/objects.hpp>
#include "track_to_track_fusion/fusion_interface.h"

using PerceptionKitObjects = perception_kit_msgs::msg::Objects;

namespace track_to_track_fusion
{
class FusionNode : public rclcpp::Node
{
public:
  struct Input
  {
    std::string name;
    std::string topic_name;
  };

  struct OutputConfiguration
  {
    std::string topic_name{};
    double min_existence_probability{ 0.0 };
  };

  FusionNode(const std::string& node_name, const rclcpp::NodeOptions& options);

  ~FusionNode();

  void spin();

private:
  void publishObjects();

  void onObjectsCallback(const PerceptionKitObjects::ConstSharedPtr& objects, const Input& input);

  FusionNode& operator=(const FusionNode&) = delete;
  FusionNode(const FusionNode&) = delete;

  PerceptionKitObject::_position_type
  toWeightsFrame(PerceptionKitObject::_position_type const& position_in_operation_frame) const;

  void declareParameters();
  void advertiseOutputs();
  void subscribeToInputs();

private:
  // ros::NodeHandle node_handle_;
  rclcpp::Publisher<PerceptionKitObjects>::SharedPtr fusion_publisher_;
  std::map<std::string, rclcpp::Subscription<PerceptionKitObjects>::SharedPtr> subscribers_;

  FusionInterface fusion_interface_;

  OutputConfiguration output_configuration_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Input> inputs_;

  std::string operation_frame_{};
  std::string weights_frame_{};

  // tf::TransformListener transform_listener_;  // @todo: tf2
};
}  // namespace track_to_track_fusion
