#include <gtest/gtest.h>
#include <perception_kit_msgs/Object.h>

#include "objects_processors/header_verification.h"
#include "objects_processors/existence_threshold_filter.h"

TEST(header_verification, object_passing)
{
  objects_processors::HeaderVerification<perception_kit_msgs::Object> header_verification("correct_frame");

  perception_kit_msgs::Object passing_object;
  passing_object.header.frame_id = "correct_frame";

  ASSERT_TRUE(header_verification.process(passing_object, "sensor_modality_does_not_matter"));
}

TEST(header_verification, object_not_passing)
{
  objects_processors::HeaderVerification<perception_kit_msgs::Object> header_verification("correct_frame");

  perception_kit_msgs::Object not_passing_object;
  not_passing_object.header.frame_id = "incorrect_frame";

  ASSERT_THROW(header_verification.process(not_passing_object, "sensor_modality_does_not_matter"), std::runtime_error);
}

TEST(header_verification, objects_passing)
{
  objects_processors::HeaderVerification<perception_kit_msgs::Objects> header_verification("correct_frame");

  perception_kit_msgs::Objects passing_objects;
  passing_objects.header.frame_id = "correct_frame";

  ASSERT_TRUE(header_verification.process(passing_objects, "sensor_modality_does_not_matter"));
}

TEST(header_verification, objects_not_passing)
{
  objects_processors::HeaderVerification<perception_kit_msgs::Objects> header_verification("correct_frame");

  perception_kit_msgs::Objects not_passing_objects;
  not_passing_objects.header.frame_id = "incorrect_frame";

  ASSERT_THROW(header_verification.process(not_passing_objects, "sensor_modality_does_not_matter"), std::runtime_error);
}

TEST(existence_threshold, object_passing_same)
{
  objects_processors::ExistenceThresoldFilter existence_threshold_filter(0.3);

  perception_kit_msgs::Object object;
  object.existence_probability = 0.3;

  ASSERT_TRUE(existence_threshold_filter.process(object, "sensor_modality_does_not_matter"));
}

TEST(existence_threshold, object_not_passing)
{
  objects_processors::ExistenceThresoldFilter existence_threshold_filter(0.3);

  perception_kit_msgs::Object object;
  object.existence_probability = 0.2;

  ASSERT_FALSE(existence_threshold_filter.process(object, "sensor_modality_does_not_matter"));
}

TEST(existence_threshold, object_passing_higher)
{
  objects_processors::ExistenceThresoldFilter existence_threshold_filter(0.3);

  perception_kit_msgs::Object object;
  object.existence_probability = 0.7;

  ASSERT_TRUE(existence_threshold_filter.process(object, "sensor_modality_does_not_matter"));
}

TEST(existence_threshold, multi_sensor)
{
  objects_processors::ExistenceThresoldFilter existence_threshold_filter(
      { std::make_pair("sensor_1", 0.3f), std::make_pair("sensor_2", 0.7f) });

  perception_kit_msgs::Object object;

  object.existence_probability = 0.1;
  ASSERT_FALSE(existence_threshold_filter.process(object, "sensor_1"));
  ASSERT_FALSE(existence_threshold_filter.process(object, "sensor_2"));

  object.existence_probability = 0.4;
  ASSERT_TRUE(existence_threshold_filter.process(object, "sensor_1"));
  ASSERT_FALSE(existence_threshold_filter.process(object, "sensor_2"));

  object.existence_probability = 0.8;
  ASSERT_TRUE(existence_threshold_filter.process(object, "sensor_1"));
  ASSERT_TRUE(existence_threshold_filter.process(object, "sensor_2"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
