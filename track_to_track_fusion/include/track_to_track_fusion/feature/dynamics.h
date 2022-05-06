#pragma once

#include "track_to_track_fusion/feature/feature_fusion_interfaces.h"

namespace track_to_track_fusion
{
class DynamicsFusionWeightedNormalDistribution : public DynamicsFusion
{
private:
  // clang-format off
  virtual void fuse(PerceptionKitObject const& object_a,
                    PerceptionKitObject const& object_b, 
                    float const& weight_a,
                    float const& weight_b,
                    PerceptionKitObject::_velocity_type& velocity,
                    PerceptionKitObject::_acceleration_type& acceleration,
                    PerceptionKitObject::_covariance_type& covariance) const override;
  // clang-format on
};

}  // namespace track_to_track_fusion