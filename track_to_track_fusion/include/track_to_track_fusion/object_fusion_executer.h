#pragma once

#include "track_to_track_fusion/feature/feature_fusion_interfaces.h"
#include "track_to_track_fusion/weights.h"

namespace track_to_track_fusion
{
class ObjectFusionExecuter
{
public:
  using ConstPtr = std::unique_ptr<ObjectFusionExecuter const>;

  using OperationToWeightsFrameTransformFunction =
      std::function<PerceptionKitObject::_position_type(PerceptionKitObject::_position_type const&)>;

  ObjectFusionExecuter(std::map<ObjectFusion::Feature, ObjectFusion::ConstPtr>&& fusers, Weights::ConstPtr weights,
                       OperationToWeightsFrameTransformFunction operation_to_weights_frame =
                           [](PerceptionKitObject::_position_type const& pos) { return pos; });

  void fuse(CostCell const& cell, PerceptionKitObject& fused_object) const;

private:
  float getWeight(ObjectWithTrace::Trace const& trace, ObjectFusion::Feature const& feature,
                  PerceptionKitObject::_position_type const& position_in_weights_frame) const;

  std::map<ObjectFusion::Feature, ObjectFusion::ConstPtr> const feature_fusion_elements_;
  Weights::ConstPtr weights_{ nullptr };

  OperationToWeightsFrameTransformFunction operation_to_weights_frame_;
};

}  // namespace track_to_track_fusion