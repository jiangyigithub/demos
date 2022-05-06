#pragma once

#ifdef ROS2
#include <functional>
#include <perception_kit_msgs/msg/object.hpp>
using PerceptionKitObject = perception_kit_msgs::msg::Object;
#else
#include <perception_kit_msgs/Object.h>
using PerceptionKitObject = perception_kit_msgs::Object;
#endif

namespace track_to_track_fusion
{
namespace cost_calculation
{
class CostCalculation
{
public:
  using ConstPtr = std::unique_ptr<CostCalculation const>;
  using CostFunction = std::function<double(PerceptionKitObject const& a, PerceptionKitObject const& b)>;

  CostCalculation(CostFunction cost_function, double const threshold);

  CostFunction cost() const;

  double threshold() const;

private:
  std::function<double(PerceptionKitObject const& a, PerceptionKitObject const& b)> cost_function_;
  double threshold_;
};

double euclideanDistanceSqrt(PerceptionKitObject const& a, PerceptionKitObject const& b);
double euclideanDistance(PerceptionKitObject const& a, PerceptionKitObject const& b);

}  // namespace cost_calculation
}  // namespace track_to_track_fusion