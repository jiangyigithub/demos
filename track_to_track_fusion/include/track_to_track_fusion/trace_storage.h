#pragma once

#include "track_to_track_fusion/object_with_trace.h"

#ifdef ROS2
#include <perception_kit_msgs/msg/objects.hpp>
#else
#include <perception_kit_msgs/Objects.h>
#endif

namespace track_to_track_fusion
{
class TraceStorage
{
public:
  static PerceptionKitObject::_id_type getIdFromTrace(ObjectWithTrace::Trace const& trace_this_cycle);
};
}  // namespace track_to_track_fusion