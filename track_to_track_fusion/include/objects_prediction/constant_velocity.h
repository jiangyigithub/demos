#pragma once
#include "objects_prediction/objects_predictor.h"

namespace objects_prediction
{
class ConstantVelocityPredictor : public ObjectsPredictor
{
public:
  virtual void predict(track_to_track_fusion::NonCopyableObjects& objects,
                       RosTime const& requested_prediction_time) const override;
};
}  // namespace objects_prediction