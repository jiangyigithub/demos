#pragma once

#include "track_to_track_fusion/object_with_trace.h"

#include <map>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include "track_to_track_fusion/feature/feature_fusion_interfaces.h"

#ifdef ROS2
#include <perception_kit_msgs/msg/object.hpp>
using PerceptionKitObject = perception_kit_msgs::msg::Object;
#else
#include <perception_kit_msgs/Object.h>
using PerceptionKitObject = perception_kit_msgs::Object;
#endif

namespace track_to_track_fusion
{
class Weights
{
public:
  using Ptr = std::unique_ptr<Weights>;
  using ConstPtr = std::unique_ptr<Weights const>;

  using boost_point = ::boost::geometry::model::point<double, 2, ::boost::geometry::cs::cartesian>;
  using boost_polygon = ::boost::geometry::model::polygon<boost_point>;

  explicit Weights(float default_weight = 1.0f);

  void setWeight(std::string const& sensor_modality, ObjectFusion::Feature const& feature, float weight);

  void setPolygonWeight(std::string const& sensor_modality, ObjectFusion::Feature const& feature,
                        boost_polygon const& polygon, float weight);

  float getWeight(ObjectWithTrace::Trace const& trace, ObjectFusion::Feature const& feature,
                  PerceptionKitObject::_position_type const& position_in_weights_frame) const;

private:
  float getWeight(std::string const& sensor_modality, ObjectFusion::Feature const& feature,
                  PerceptionKitObject::_position_type const& position_in_weights_frame) const;

private:
  std::map<std::string, std::map<ObjectFusion::Feature, float> > weights_;
  std::map<std::string, std::map<ObjectFusion::Feature, std::pair<boost_polygon, float> > > weight_polygons_;
  float default_weight_;
};

}  // namespace track_to_track_fusion