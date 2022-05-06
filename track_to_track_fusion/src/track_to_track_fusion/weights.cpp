#include "track_to_track_fusion/weights.h"

namespace track_to_track_fusion
{
Weights::Weights(float default_weight) : default_weight_(default_weight)
{
  if (default_weight_ <= 0.0f)
    throw std::invalid_argument("weight needs to be bigger than zero");
}
void Weights::setWeight(std::string const& sensor_modality, ObjectFusion::Feature const& feature, float weight)
{
  if (weight <= 0.0f)
    throw std::invalid_argument("weight needs to be bigger than zero");

  weights_[sensor_modality][feature] = weight;
}

void Weights::setPolygonWeight(std::string const& sensor_modality, ObjectFusion::Feature const& feature,
                               boost_polygon const& polygon, float weight)
{
  auto polygon_copy = polygon;
  ::boost::geometry::correct(polygon_copy);
  weight_polygons_[sensor_modality][feature] = std::make_pair(polygon_copy, weight);
}

float Weights::getWeight(ObjectWithTrace::Trace const& trace, ObjectFusion::Feature const& feature,
                         PerceptionKitObject::_position_type const& position_in_weights_frame) const
{
  float ret = 0.0;
  for (auto const& t : trace)
  {
    ret += getWeight(t.first, feature, position_in_weights_frame);
  }
  return ret;
}

float Weights::getWeight(std::string const& sensor_modality, ObjectFusion::Feature const& feature,
                         PerceptionKitObject::_position_type const& position_in_weights_frame) const
{
  float ret = default_weight_;
  if (weights_.count(sensor_modality) && weights_.at(sensor_modality).count(feature))
  {
    ret = weights_.at(sensor_modality).at(feature);
  }

  if (weight_polygons_.count(sensor_modality) && weight_polygons_.at(sensor_modality).count(feature))
  {
    auto const& polygon = weight_polygons_.at(sensor_modality).at(feature).first;
    auto const pt = boost_point(position_in_weights_frame.x, position_in_weights_frame.y);
    if (::boost::geometry::within(pt, polygon))
    {
      ret = weight_polygons_.at(sensor_modality).at(feature).second;
    }
  }
  return ret;
}
}  // namespace track_to_track_fusion
