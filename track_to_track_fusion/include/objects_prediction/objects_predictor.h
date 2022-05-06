#pragma once

#include "track_to_track_fusion/noncopyobjects.h"

#ifdef ROS2
#include "rclcpp/rclcpp.hpp"
using RosTime = rclcpp::Time;
#else
using RosTime = ros::Time;
#endif

namespace objects_prediction
{
class ObjectsPredictor
{
public:
  using ConstPtr = std::unique_ptr<ObjectsPredictor const>;

  virtual void predict(track_to_track_fusion::NonCopyableObjects& objects,
                       RosTime const& requested_prediction_time) const = 0;

protected:
  ObjectsPredictor() = default;

private:
  ObjectsPredictor(ObjectsPredictor const&) = delete;
};

template <typename TimeType>
double getSecondsFromRosTime(const TimeType& time)
{
#ifdef ROS2
  return time.seconds();
#else
  return time.toSec();
#endif
}

}  // namespace objects_prediction
