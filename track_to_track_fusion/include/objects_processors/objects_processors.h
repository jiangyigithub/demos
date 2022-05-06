#pragma once

#include <map>

#include "track_to_track_fusion/noncopyobjects.h"

#ifdef ROS2
using PerceptionKitObject = perception_kit_msgs::msg::Object;
#else
using PerceptionKitObject = perception_kit_msgs::Object;
#endif

namespace objects_processors
{
template <class T>
class ObjectsProcessor
{
public:
  // return true, if the object list is valid
  virtual bool process(T&, std::string const&) const
  {
    throw std::runtime_error("virtual function ObjectsProcessor::process() called.");
  };

protected:
  ObjectsProcessor() = default;

private:
  ObjectsProcessor operator=(ObjectsProcessor const&) = delete;
  ObjectsProcessor(ObjectsProcessor const&) = delete;
};

using SingleObjectProcessor = ObjectsProcessor<PerceptionKitObject>;
using ObjectListProcessor = ObjectsProcessor<track_to_track_fusion::NonCopyableObjects>;

using SingleObjectProcessorConstPtr = std::unique_ptr<SingleObjectProcessor const>;
using ObjectListProcessorConstPtr = std::unique_ptr<ObjectListProcessor const>;

}  // namespace objects_processors
