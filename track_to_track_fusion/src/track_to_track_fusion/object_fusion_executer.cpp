#include "track_to_track_fusion/object_fusion_executer.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef ROS2
#include "rclcpp/rclcpp.hpp"
#define ROS_ERROR_STREAM_ONCE(log_message)                                                                             \
  RCLCPP_ERROR_ONCE(rclcpp::get_logger("track_to_track_fusion.classification"), log_message)
#include <iostream>
#else
#include <ros/ros.h>
#endif
#pragma GCC diagnostic pop

namespace track_to_track_fusion
{
ObjectFusionExecuter::ObjectFusionExecuter(
    std::map<ObjectFusion::Feature, ObjectFusion::ConstPtr>&& feature_fusion_elements, Weights::ConstPtr weights,
    OperationToWeightsFrameTransformFunction operation_to_weights_frame)
  : feature_fusion_elements_(std::move(feature_fusion_elements))
  , weights_(std::move(weights))
  , operation_to_weights_frame_(operation_to_weights_frame)
{
}

void ObjectFusionExecuter::fuse(CostCell const& cell, PerceptionKitObject& fused_object) const
{
  auto const& object_a = cell.row_object().object();
  auto const& object_b = cell.col_object().object();
  auto const& trace_a = cell.row_object().trace();
  auto const& trace_b = cell.col_object().trace();

  auto const pos_a_in_weights_frame = operation_to_weights_frame_(object_a.position);
  auto const pos_b_in_weights_frame = operation_to_weights_frame_(object_b.position);

  for (auto const& feature : ObjectFusion::AllFeatures())
  {
    if (feature_fusion_elements_.count(feature))
    {
      auto const weight_a = getWeight(trace_a, feature, pos_a_in_weights_frame);
      auto const weight_b = getWeight(trace_b, feature, pos_b_in_weights_frame);
      feature_fusion_elements_.at(feature)->fuse(object_a, object_b, weight_a, weight_b, fused_object);
    }
    else
    {
      ROS_ERROR_STREAM_ONCE("No fusion methon set for feature ");
    }
  }
}

float ObjectFusionExecuter::getWeight(ObjectWithTrace::Trace const& trace, ObjectFusion::Feature const& feature,
                                      PerceptionKitObject::_position_type const& position_in_weights_frame) const
{
  return weights_ ? weights_->getWeight(trace, feature, position_in_weights_frame) : 1.0f;
}
}  // namespace track_to_track_fusion
