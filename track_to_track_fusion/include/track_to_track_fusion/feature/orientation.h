#pragma once
#include "track_to_track_fusion/feature/feature_fusion_interfaces.h"

namespace track_to_track_fusion
{
class OrientationFusionWeightedNormalDistribution : public OrientationFusion
{
private:
  // clang-format off
  virtual void fuse(PerceptionKitObject const& object_a,
                    PerceptionKitObject const& object_b, 
                    float const& weight_a,
                    float const& weight_b,
                    PerceptionKitObject::_yaw_type& yaw,
                    PerceptionKitObject::_yaw_rate_type& yaw_rate,
                    PerceptionKitObject::_covariance_type& covariance) const override;
  // clang-format on
};

}  // namespace track_to_track_fusion