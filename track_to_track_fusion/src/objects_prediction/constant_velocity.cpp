#include "objects_prediction/constant_velocity.h"

namespace objects_prediction
{
void ConstantVelocityPredictor::predict(track_to_track_fusion::NonCopyableObjects& objects,
                                        RosTime const& requested_prediction_time) const
{
  objects.header.stamp = requested_prediction_time;
  for (auto& object : objects.objects)
  {
    auto const prediction_duration = getSecondsFromRosTime(requested_prediction_time - object.header.stamp);

    // @todo: Check if the time is far off ...

    object.header.stamp = requested_prediction_time;
    object.position.x += object.velocity.x * prediction_duration;
    object.position.y += object.velocity.y * prediction_duration;
    object.position.z += object.velocity.z * prediction_duration;
  }
}
}  // namespace objects_prediction