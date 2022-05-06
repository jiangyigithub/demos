#pragma once

#include <list>
#include <map>

#ifdef ROS2
#include <perception_kit_msgs/msg/object.hpp>
using PerceptionKitObject = perception_kit_msgs::msg::Object;
#else
#include <perception_kit_msgs/Object.h>
using PerceptionKitObject = perception_kit_msgs::Object;
#endif

namespace track_to_track_fusion
{
class ObjectWithTrace
{
public:
  using Container = std::list<ObjectWithTrace>;
  using ObjectIt = PerceptionKitObject const&;
  using Trace = std::map<std::string, int>;

  ObjectWithTrace(ObjectIt const& object, Trace trace, uint32_t internal_fusion_id);

  ObjectWithTrace(std::unique_ptr<PerceptionKitObject const> object, Trace trace, uint32_t internal_fusion_id);

  uint32_t internal_fusion_id() const;

  bool isSameObject(ObjectWithTrace const& other) const;

  bool hasCommonSensorTrace(ObjectWithTrace const& other) const;

  PerceptionKitObject const& object() const;

  Trace const& trace() const;

private:
  std::unique_ptr<PerceptionKitObject const> const object_ptr_{ nullptr };
  ObjectIt const object_;
  Trace const trace_;
  uint32_t const internal_fusion_id_;

  ObjectWithTrace() = delete;
  ObjectWithTrace(ObjectWithTrace const&) = delete;
  ObjectWithTrace operator=(ObjectWithTrace const&) = delete;

  friend std::ostream& operator<<(std::ostream& os, const ObjectWithTrace& cost_cell);
};
}  // namespace track_to_track_fusion