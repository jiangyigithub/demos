#include "track_to_track_fusion/feature/position.h"
#include "track_to_track_fusion/feature/math_utils.h"

namespace track_to_track_fusion
{
void PoseFusionWeightedNormalDistribution::fuse(PerceptionKitObject const& object_a,
                                                PerceptionKitObject const& object_b, float const& weight_a,
                                                float const& weight_b, PerceptionKitObject::_position_type& pos,
                                                PerceptionKitObject::_covariance_type& covariance) const
{  // clang-format off
  {
    auto const covariance_index = DIAGONAL(PerceptionKitObject::ELEMENT_POSITION_X);
    math_utils::averageTwoNormalDistributions(object_a.position.x,
                                              object_b.position.x,
                                              object_a.covariance[covariance_index],
                                              object_b.covariance[covariance_index],
                                              weight_a,
                                              weight_b,
                                              pos.x,
                                              covariance[covariance_index]);
  }

  {
    auto const covariance_index = DIAGONAL(PerceptionKitObject::ELEMENT_POSITION_Y);
    math_utils::averageTwoNormalDistributions(object_a.position.y,
                                              object_b.position.y,
                                              object_a.covariance[covariance_index],
                                              object_b.covariance[covariance_index],
                                              weight_a,
                                              weight_b,
                                              pos.y,
                                              covariance[covariance_index]);
  }

  {
    auto const covariance_index = DIAGONAL(PerceptionKitObject::ELEMENT_POSITION_Z);
    math_utils::averageTwoNormalDistributions(object_a.position.z,
                                              object_b.position.z,
                                              object_a.covariance[covariance_index],
                                              object_b.covariance[covariance_index],
                                              weight_a,
                                              weight_b,
                                              pos.z,
                                              covariance[covariance_index]);
  }

}
// clang-format on
}  // namespace track_to_track_fusion
