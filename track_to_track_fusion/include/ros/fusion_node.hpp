#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#pragma GCC diagnostic pop

#include <perception_kit_msgs/Objects.h>
#include "track_to_track_fusion/fusion_interface.h"

namespace track_to_track_fusion
{
class FusionNode final
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

  FusionNode();

  ~FusionNode();

  void spin();

private:
  void publishObjects(const ros::TimerEvent&);

  void onObjectsCallback(const perception_kit_msgs::ObjectsConstPtr& objects, Input const* input);

  FusionNode& operator=(const FusionNode&) = delete;
  FusionNode(const FusionNode&) = delete;

  perception_kit_msgs::Object::_position_type
  toWeightsFrame(perception_kit_msgs::Object::_position_type const& position_in_operation_frame) const;

  void advertiseOutputs();
  void subscribeToInputs();

private:
  ros::NodeHandle node_handle_;
  ros::Publisher fusion_publisher_;
  std::map<std::string, ros::Subscriber> subscribers_;

  FusionInterface fusion_interface_;

  OutputConfiguration output_configuration_;

  ros::Timer timer_;

  std::vector<Input> inputs_;

  std::string operation_frame_{};
  std::string weights_frame_{};

  tf::TransformListener transform_listener_;  // @todo: tf2
};
}  // namespace track_to_track_fusion
