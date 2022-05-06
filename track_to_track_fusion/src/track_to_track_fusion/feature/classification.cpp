#include "track_to_track_fusion/feature/classification.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef ROS2
#include "rclcpp/rclcpp.hpp"
#define ROS_WARN_COND(cond, log_message)                                                                               \
  RCLCPP_WARN_EXPRESSION(rclcpp::get_logger("track_to_track_fusion.classification"), cond, log_message)
using PerceptionKitClassification = perception_kit_msgs::msg::Classification;
#else
#include <ros/ros.h>
using PerceptionKitClassification = perception_kit_msgs::Classification;
#endif
#pragma GCC diagnostic pop

namespace track_to_track_fusion
{
namespace internal
{
// Normalizes the classification entries in a way, that the confidences sum up to 1.0 in the end. If there is no
// "unknown" class, and the confidences sum up to sth. less than 1.0, an unknown class is added with the remaining
// confidence. If the unknown class already exists or the confidences sum up to more than one, the confidences are
// scaled to sum to one
void normalizeClassification(PerceptionKitObject::_classification_type& classifications)
{
  bool no_unknown_class = true;
  float normalization_factor = 0.0f;
  for (auto& classification : classifications)
  {
    ROS_WARN_COND(classification.confidence < 0.0f, "Negative confidence values are not allowed. Confidence is set to "
                                                    "zero");
    classification.confidence = std::max(classification.confidence, 0.0f);

    no_unknown_class = no_unknown_class && classification.obj_class != "unknown";
    normalization_factor += classification.confidence;
  }

  if (normalization_factor < 1.0 && no_unknown_class)
  {
    PerceptionKitClassification unknown;
    unknown.confidence = 1.0 - normalization_factor;
    unknown.obj_class = "unknown";
    classifications.push_back(unknown);
  }
  else
  {
    std::for_each(std::begin(classifications), std::end(classifications),
                  [normalization_factor](PerceptionKitClassification& c) { c.confidence /= normalization_factor; });
  }
}
}  // namespace internal

// clang-format off
void UnionClassificationFusion::fuse(PerceptionKitObject const& object_a,
                                     PerceptionKitObject const& object_b,
                                     float const & weight_a,
                                     float const & weight_b,
                                     PerceptionKitObject::_classification_type& classification_out) const
// clang-format on
{
  std::map<std::string, PerceptionKitClassification::_confidence_type> classifications;
  for (auto const c : object_a.classification)
  {
    classifications[c.obj_class] = weight_a * c.confidence;
  }
  for (auto const c : object_b.classification)
  {
    if (classifications.count(c.obj_class) == 0)
    {
      classifications[c.obj_class] = weight_b * c.confidence;
    }
    else
    {
      classifications.at(c.obj_class) += weight_b * c.confidence;
    }
  }

  for (auto const& classification : classifications)
  {
    PerceptionKitClassification c;
    c.obj_class = classification.first;
    c.confidence = classification.second;
    classification_out.push_back(c);
  }

  internal::normalizeClassification(classification_out);
}

}  // namespace track_to_track_fusion
