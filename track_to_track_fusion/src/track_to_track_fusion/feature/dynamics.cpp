#include "track_to_track_fusion/feature/dynamics.h"
#include "track_to_track_fusion/feature/math_utils.h"

namespace track_to_track_fusion
{
// clang-format off
void DynamicsFusionWeightedNormalDistribution::fuse(PerceptionKitObject const& object_a,
                                                    PerceptionKitObject const& object_b, 
                                                    float const& weight_a,
                                                    float const& weight_b,
                                                    PerceptionKitObject::_velocity_type& velocity,
                                                    PerceptionKitObject::_acceleration_type& acceleration,
                                                    PerceptionKitObject::_covariance_type& covariance) const
{
  {
    auto const covariance_index = DIAGONAL(PerceptionKitObject::ELEMENT_VELOCITY_X);
    // clang-format off
    math_utils::averageTwoNormalDistributions(object_a.velocity.x,
                                              object_b.velocity.x,
                                              object_a.covariance[covariance_index],
                                              object_b.covariance[covariance_index],
                                              weight_a,
                                              weight_b,
                                              velocity.x,
                                              covariance[covariance_index]);
    // clang-format on
  }

  {
    auto const covariance_index = DIAGONAL(PerceptionKitObject::ELEMENT_VELOCITY_Y);
    // clang-format off
    math_utils::averageTwoNormalDistributions(object_a.velocity.y,
                                              object_b.velocity.y,
                                              object_a.covariance[covariance_index],
                                              object_b.covariance[covariance_index],
                                              weight_a,
                                              weight_b,
                                              velocity.y,
                                              covariance[covariance_index]);
    // clang-format on
  }

  {
    auto const covariance_index = DIAGONAL(PerceptionKitObject::ELEMENT_VELOCITY_Z);
    // clang-format off
    math_utils::averageTwoNormalDistributions(object_a.velocity.z,
                                              object_b.velocity.z,
                                              object_a.covariance[covariance_index],
                                              object_b.covariance[covariance_index],
                                              weight_a,
                                              weight_b,
                                              velocity.z,
                                              covariance[covariance_index]);
    // clang-format on
  }

  {
    auto const covariance_index = DIAGONAL(PerceptionKitObject::ELEMENT_ACCELERATION_X);
    // clang-format off
    math_utils::averageTwoNormalDistributions(object_a.acceleration.x,
                                              object_b.acceleration.x,
                                              object_a.covariance[covariance_index],
                                              object_b.covariance[covariance_index],
                                              weight_a,
                                              weight_b,
                                              acceleration.x,
                                              covariance[covariance_index]);
    // clang-format on
  }

  {
    auto const covariance_index = DIAGONAL(PerceptionKitObject::ELEMENT_ACCELERATION_Y);
    // clang-format off
    math_utils::averageTwoNormalDistributions(object_a.acceleration.y,
                                              object_b.acceleration.y,
                                              object_a.covariance[covariance_index],
                                              object_b.covariance[covariance_index],
                                              weight_a,
                                              weight_b,
                                              acceleration.y,
                                              covariance[covariance_index]);
    // clang-format on
  }

  {
    auto const covariance_index = DIAGONAL(PerceptionKitObject::ELEMENT_ACCELERATION_Z);
    // clang-format off
    math_utils::averageTwoNormalDistributions(object_a.acceleration.z,
                                              object_b.acceleration.z,
                                              object_a.covariance[covariance_index],
                                              object_b.covariance[covariance_index],
                                              weight_a,
                                              weight_b,
                                              acceleration.z,
                                              covariance[covariance_index]);
    // clang-format on
  }
}
}  // namespace track_to_track_fusion
