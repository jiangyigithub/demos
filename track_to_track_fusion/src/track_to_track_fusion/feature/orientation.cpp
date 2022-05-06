#include "track_to_track_fusion/feature/orientation.h"
#include "track_to_track_fusion/feature/math_utils.h"

namespace track_to_track_fusion
{
// clang-format off
void OrientationFusionWeightedNormalDistribution::fuse(
    PerceptionKitObject const& object_a,
    PerceptionKitObject const& object_b,
    float const & weight_a,
    float const & weight_b,
    PerceptionKitObject::_yaw_type& yaw,
    PerceptionKitObject::_yaw_rate_type& yaw_rate,
    PerceptionKitObject::_covariance_type& covariance) const
{
  auto const yaw_index = DIAGONAL(PerceptionKitObject::ELEMENT_YAW);
  track_to_track_fusion::math_utils::averageTwoNormalDistributions(object_a.yaw,
                                                      object_b.yaw,
                                                      object_a.covariance[yaw_index],
                                                      object_b.covariance[yaw_index],
                                                      weight_a,
                                                      weight_b,
                                                      yaw,
                                                      covariance[yaw_index]);

  auto const yaw_rate_index = DIAGONAL(PerceptionKitObject::ELEMENT_YAW_RATE);
  track_to_track_fusion::math_utils::averageTwoNormalDistributions(object_a.yaw_rate,
                                                      object_b.yaw_rate,
                                                      object_a.covariance[yaw_rate_index],
                                                      object_b.covariance[yaw_rate_index],
                                                      weight_a,
                                                      weight_b,
                                                      yaw_rate,
                                                      covariance[yaw_rate_index]);

}
// clang-format on

}  // namespace track_to_track_fusion