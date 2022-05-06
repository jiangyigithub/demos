#pragma once

#ifdef ROS2
#include <perception_kit_msgs/msg/objects.hpp>
using PerceptionKitObjects = perception_kit_msgs::msg::Objects;
#else
#include <perception_kit_msgs/Objects.h>
using PerceptionKitObjects = perception_kit_msgs::Objects;
#endif

namespace track_to_track_fusion
{
class NonCopyableObjects : public PerceptionKitObjects
{
public:
  using Ptr = std::shared_ptr<NonCopyableObjects>;
  using ConstPtr = std::shared_ptr<NonCopyableObjects const>;

  NonCopyableObjects(PerceptionKitObjects const& objects) : PerceptionKitObjects(objects)
  {
  }

private:
  NonCopyableObjects(NonCopyableObjects const&) = delete;
  NonCopyableObjects operator=(NonCopyableObjects const&) = delete;
  NonCopyableObjects() = delete;
};
}  // namespace track_to_track_fusion