#include "objects_processors/existence_threshold_filter.h"

namespace objects_processors
{
ExistenceThresoldFilter::ExistenceThresoldFilter(std::map<std::string const, float const> const& existence_thresholds)
  : existence_thresholds_(existence_thresholds)
{
}

ExistenceThresoldFilter::ExistenceThresoldFilter(float const& existence_threshold)
  : existence_thresholds_({ { "default", existence_threshold } })
{
}

/* Returns true, if the object passed the minimum existence threshold
 */
bool ExistenceThresoldFilter::process(PerceptionKitObject& object, std::string const& sensor_modality) const
{
  if (existence_thresholds_.count(sensor_modality))
  {
    return object.existence_probability >= existence_thresholds_.at(sensor_modality);
  }
  return object.existence_probability >= existence_thresholds_.at("default");
}

}  // namespace objects_processors