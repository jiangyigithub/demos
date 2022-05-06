#pragma once
#include "track_to_track_fusion/cost_cell.h"
#include <array>

namespace track_to_track_fusion
{
class ObjectFusion
{
public:
  using Feature = std::string const;
  using FeatureArray = std::array<Feature, 6>;

  static FeatureArray const& AllFeatures()
  {
    static FeatureArray const a{ "dimension", "classification", "position", "existence_probability",
                                 "dynamics",  "orientation" };
    return a;
  };

  using ConstPtr = std::unique_ptr<ObjectFusion const>;

  // clang-format off
  virtual void fuse(PerceptionKitObject const& object_a,
                    PerceptionKitObject const& object_b,
                    float const& weight_a,
                    float const& weight_b,
                    PerceptionKitObject& fusion_object) const = 0;
  // clang-format on
};

class DimensionsFusion : public ObjectFusion
{
public:
  using ConstPtr = std::unique_ptr<DimensionsFusion>;

  // clang-format off
  void fuse(PerceptionKitObject const& object_a,
            PerceptionKitObject const& object_b,
            float const& weight_a,
            float const& weight_b,
            PerceptionKitObject& fusion_object) const final override
  {
    fuse(object_a, object_b,
         weight_a, weight_b,
         fusion_object.length, fusion_object.width, fusion_object.height,
         fusion_object.length_variance, fusion_object.width_variance, fusion_object.height_variance);
  }
  // clang-format on

protected:
  // clang-format off
  virtual void fuse(PerceptionKitObject const& a,
                    PerceptionKitObject const& b,
                    float const& weight_a,
                    float const& weight_b,
                    PerceptionKitObject::_length_type& length,
                    PerceptionKitObject::_width_type& width,
                    PerceptionKitObject::_height_type& height,
                    PerceptionKitObject::_length_variance_type& length_variance,
                    PerceptionKitObject::_width_variance_type& width_variance,
                    PerceptionKitObject::_height_variance_type& height_variance) const = 0;
  // clang-format on
};

class PoseFusion : public ObjectFusion
{
public:
  using ConstPtr = std::unique_ptr<PoseFusion>;

  // clang-format off
  void fuse(PerceptionKitObject const& object_a,
            PerceptionKitObject const& object_b,
            float const& weight_a,
            float const& weight_b,
            PerceptionKitObject& fusion_object) const final override
  {
    fuse(object_a, object_b,
         weight_a, weight_b,
         fusion_object.position,
         fusion_object.covariance);
  }
  // clang-format on

protected:
  // clang-format off
  virtual void fuse(PerceptionKitObject const& object_a,
                    PerceptionKitObject const& object_b, 
                    float const& weight_a,
                    float const& weight_b,
                    PerceptionKitObject::_position_type& pos,
                    PerceptionKitObject::_covariance_type& covariance) const = 0;

  // clang-format on
};

class OrientationFusion : public ObjectFusion
{
public:
  using ConstPtr = std::unique_ptr<OrientationFusion>;

  // clang-format off
  void fuse(PerceptionKitObject const& object_a,
            PerceptionKitObject const& object_b,
            float const& weight_a,
            float const& weight_b,
            PerceptionKitObject& fusion_object) const final override
  {
    fuse(object_a, object_b,
         weight_a, weight_b,
         fusion_object.yaw,
         fusion_object.yaw_rate,
         fusion_object.covariance);
  }
  // clang-format on

protected:
  // clang-format off
  virtual void fuse(PerceptionKitObject const& object_a,
                    PerceptionKitObject const& object_b, 
                    float const& weight_a,
                    float const& weight_b,
                    PerceptionKitObject::_yaw_type& yaw,
                    PerceptionKitObject::_yaw_rate_type& yaw_rate,
                    PerceptionKitObject::_covariance_type& covariance) const = 0;

  // clang-format on
};

class ExistenceProbabilityFusion : public ObjectFusion
{
public:
  using ConstPtr = std::unique_ptr<ExistenceProbabilityFusion>;

  // clang-format off
  void fuse(PerceptionKitObject const& object_a,
            PerceptionKitObject const& object_b,
            float const& weight_a,
            float const& weight_b,
            PerceptionKitObject& fusion_object) const final override
  {
    fuse(object_a, object_b,
         weight_a, weight_b,
         fusion_object.existence_probability);
  }
  // clang-format on

protected:
  // clang-format off
  virtual void fuse(PerceptionKitObject const& object_a,
                    PerceptionKitObject const& object_b, 
                    float const& weight_a,
                    float const& weight_b,
                    PerceptionKitObject::_existence_probability_type& existence_probability) const = 0;

  // clang-format on
};

class DynamicsFusion : public ObjectFusion
{
public:
  using ConstPtr = std::unique_ptr<DynamicsFusion>;

  // clang-format off
  void fuse(PerceptionKitObject const& object_a,
            PerceptionKitObject const& object_b,
            float const& weight_a,
            float const& weight_b,
            PerceptionKitObject& fusion_object) const final override
  {
    fuse(object_a, object_b,
         weight_a, weight_b,
         fusion_object.velocity,fusion_object.acceleration,fusion_object.covariance);
  }
  // clang-format on

protected:
  // clang-format off
  virtual void fuse(PerceptionKitObject const& object_a,
                    PerceptionKitObject const& object_b, 
                    float const& weight_a,
                    float const& weight_b,
                    PerceptionKitObject::_velocity_type& velocity,
                    PerceptionKitObject::_acceleration_type& acceleration,
                    PerceptionKitObject::_covariance_type& covariance) const = 0;

  // clang-format on
};

class ClassificationFusion : public ObjectFusion
{
public:
  using ConstPtr = std::unique_ptr<ClassificationFusion>;

  // clang-format off
  void fuse(PerceptionKitObject const& object_a,
            PerceptionKitObject const& object_b,
            float const& weight_a,
            float const& weight_b,
            PerceptionKitObject& fusion_object) const final override
  {
    fuse(object_a, object_b,
         weight_a, weight_b,
         fusion_object.classification);
  }
  // clang-format on

protected:
  // clang-format off
  virtual void fuse(PerceptionKitObject const& object_a,
                    PerceptionKitObject const& object_b, 
                    float const& weight_a,
                    float const& weight_b,
                    PerceptionKitObject::_classification_type& classification) const = 0;

  // clang-format on
};

}  // namespace track_to_track_fusion
