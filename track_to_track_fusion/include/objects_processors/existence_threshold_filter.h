#pragma once

#include "objects_processors/objects_processors.h"

#ifdef ROS2
using PerceptionKitObject = perception_kit_msgs::msg::Object;
#else
using PerceptionKitObject = perception_kit_msgs::Object;
#endif
namespace objects_processors
{
class ExistenceThresoldFilter : public SingleObjectProcessor
{
public:
  explicit ExistenceThresoldFilter(std::map<std::string const, float const> const& existence_thresholds);

  explicit ExistenceThresoldFilter(float const& existence_threshold);

  /* Returns true, if the object passed the minimum existence threshold
   */
  bool process(PerceptionKitObject& object, std::string const& sensor_modality) const override;

private:
  std::map<std::string const, float const> const existence_thresholds_;
};
}  // namespace objects_processors