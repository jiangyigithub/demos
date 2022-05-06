#pragma once

#include "track_to_track_fusion/feature/feature_fusion_interfaces.h"

namespace track_to_track_fusion
{
class DimensionFusionWeightedNormalDistribution : public DimensionsFusion
{
private:
  // clang-format off
  virtual void fuse(PerceptionKitObject const& object_a,
                    PerceptionKitObject const& object_b, 
                    float const& weight_a,
                    float const& weight_b,
                    PerceptionKitObject::_length_type& length,
                    PerceptionKitObject::_width_type& width,
                    PerceptionKitObject::_height_type& height,
                    PerceptionKitObject::_length_variance_type& length_variance,
                    PerceptionKitObject::_width_variance_type& width_variance,
                    PerceptionKitObject::_height_variance_type& height_variance) const override;
  // clang-format on
};

}  // namespace track_to_track_fusion