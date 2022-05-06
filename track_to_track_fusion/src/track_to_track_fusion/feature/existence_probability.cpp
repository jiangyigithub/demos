#include "track_to_track_fusion/feature/existence_probability.h"

namespace track_to_track_fusion
{
// clang-format off
void ExistenceProbabilityWeightedFusion::fuse(
    PerceptionKitObject const& object_a,
    PerceptionKitObject const& object_b,
    float const & weight_a,
    float const & weight_b,
    PerceptionKitObject::_existence_probability_type& existence_probability) const
// clang-format on
{
  existence_probability =
      (weight_a * object_a.existence_probability + weight_b * object_b.existence_probability) / (weight_a + weight_b);
}

}  // namespace track_to_track_fusion