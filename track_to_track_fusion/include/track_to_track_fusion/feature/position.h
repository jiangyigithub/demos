#pragma once
#include "track_to_track_fusion/feature/feature_fusion_interfaces.h"

namespace track_to_track_fusion
{
class PoseFusionWeightedNormalDistribution : public PoseFusion
{
private:
  // clang-format off
  virtual void fuse(PerceptionKitObject const& object_a,
                    PerceptionKitObject const& object_b, 
                    float const& weight_a,
                    float const& weight_b,
                    PerceptionKitObject::_position_type& pos,
                    PerceptionKitObject::_covariance_type& covariance) const override;
  // clang-format on
};

}  // namespace track_to_track_fusion