#pragma once

#include "objects_processors/objects_processors.h"
#include "objects_prediction/constant_velocity.h"
#include "track_to_track_fusion/weights.h"

#include <forward_list>
#include <list>
#include <ostream>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef ROS2
#include "rclcpp/rclcpp.hpp"
#else
#include <ros/ros.h>
#endif
#pragma GCC diagnostic pop

#include "track_to_track_fusion/object_fusion_executer.h"
#include "cost/cost_functions.h"
#include "track_to_track_fusion/fusion_preprocessor_interface.h"
#include "track_to_track_fusion/fusion_postprocessor_interface.h"

namespace track_to_track_fusion
{
class FusionInterface : public FusionPreprocessorInterface, public FusionPostprocessorInterface
{
public:
  void init(std::set<std::string> const& sensor_modalities, ObjectFusionExecuter::ConstPtr fusion_executer,
            cost_calculation::CostCalculation::ConstPtr cost_function);

  virtual PerceptionKitObjects fuse(RosTime const& fusion_time);

  void setPredictor(::objects_prediction::ObjectsPredictor::ConstPtr predictor);

private:
  void predict(FusionInterface::NonCopyableObjectsContainer& data_container,
               RosTime const& requested_prediction_time) const;

  objects_prediction::ObjectsPredictor::ConstPtr predictor_{ nullptr };

  ObjectFusionExecuter::ConstPtr fusion_{ nullptr };

  cost_calculation::CostCalculation::ConstPtr cost_function_{ nullptr };
};
}  // namespace track_to_track_fusion
