#include "track_to_track_fusion/feature/dimension.h"
#include "track_to_track_fusion/feature/math_utils.h"

namespace track_to_track_fusion
{
// clang-format off
void DimensionFusionWeightedNormalDistribution::fuse(
    PerceptionKitObject const& object_a,
    PerceptionKitObject const& object_b,
    float const & weight_a,
    float const & weight_b,
    PerceptionKitObject::_length_type& length,
    PerceptionKitObject::_width_type& width,
    PerceptionKitObject::_height_type& height,
    PerceptionKitObject::_length_variance_type& length_variance,
    PerceptionKitObject::_width_variance_type& width_variance,
    PerceptionKitObject::_height_variance_type& height_variance) const
{
  track_to_track_fusion::math_utils::averageTwoNormalDistributions(object_a.length, object_b.length,
                                                      object_a.length_variance, object_b.length_variance,
                                                      weight_a, weight_b,
                                                      length,
                                                      length_variance);

  track_to_track_fusion::math_utils::averageTwoNormalDistributions(object_a.width, object_b.width,
                                                      object_a.width_variance, object_b.width_variance,
                                                      weight_a, weight_b,
                                                      width,
                                                      width_variance);

  track_to_track_fusion::math_utils::averageTwoNormalDistributions(object_a.height, object_b.height,
                                                      object_a.height_variance, object_b.height_variance,
                                                      weight_a, weight_b,
                                                      height,
                                                      height_variance);
}
// clang-format on

}  // namespace track_to_track_fusion