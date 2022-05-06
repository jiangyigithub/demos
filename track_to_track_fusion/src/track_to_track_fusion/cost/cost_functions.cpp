#include "track_to_track_fusion/cost/cost_functions.h"
#include <math.h>

namespace track_to_track_fusion
{
namespace cost_calculation
{
CostCalculation::CostCalculation(CostFunction cost_function, double const threshold)
  : cost_function_(cost_function), threshold_(threshold)
{
}

CostCalculation::CostFunction CostCalculation::cost() const
{
  return cost_function_;
}

double CostCalculation::threshold() const
{
  return threshold_;
}

double euclideanDistanceSqrt(PerceptionKitObject const& a, PerceptionKitObject const& b)
{
  if (a.header.frame_id != b.header.frame_id)
    throw std::runtime_error("distance cannot be calculated for objects not in the same coordinate system.");

  double const diff_x = a.position.x - b.position.x;
  double const diff_y = a.position.y - b.position.y;
  double const diff_z = a.position.z - b.position.z;

  return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
}

double euclideanDistance(PerceptionKitObject const& a, PerceptionKitObject const& b)
{
  return sqrt(euclideanDistanceSqrt(a, b));
}
}  // namespace cost_calculation
}  // namespace track_to_track_fusion
