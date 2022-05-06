#pragma once
#include <math.h>

namespace track_to_track_fusion
{
namespace math_utils
{
#define DIAGONAL(x) x + x* PerceptionKitObject::NUM_COVARIANCE_ELEMENTS

template <typename T>
void averageTwoNormalDistributions(T const& mu_a, T const& mu_b, float const& sigma_a, float const& sigma_b,
                                   float const& weight_a, float const& weight_b, T& mu, float& sigma)
{
  // ROS_ERROR_STREAM("NA: mu: " << mu_a << " sigma: " << sigma_a << " weight: " << weight_a);
  // ROS_ERROR_STREAM("NB: mu: " << mu_b << " sigma: " << sigma_b << " weight: " << weight_b);

  mu = (mu_a * weight_a + mu_b * weight_b) / (weight_a + weight_b);
  auto const a = (sigma_a * sigma_a + mu_a * mu_a) * weight_a;
  auto const b = (sigma_b * sigma_b + mu_b * mu_b) * weight_b;
  sigma = sqrt(((a + b) / (weight_a + weight_b)) - mu * mu);
}

}  // namespace math_utils
}  // namespace track_to_track_fusion