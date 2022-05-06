#include <gtest/gtest.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <ros/ros.h>
#pragma GCC diagnostic pop

#include "objects_prediction/constant_velocity.h"

TEST(constant_velocity, empty_objects)
{
  auto predictor = std::make_unique<objects_prediction::ConstantVelocityPredictor>();

  perception_kit_msgs::Objects objects_msg;
  track_to_track_fusion::NonCopyableObjects objects(objects_msg);

  predictor->predict(objects, ros::Time(10));
}

TEST(constant_velocity, empty_header)
{
  auto predictor = std::make_unique<objects_prediction::ConstantVelocityPredictor>();

  perception_kit_msgs::Objects objects_msg;
  perception_kit_msgs::Object object_msg;
  objects_msg.objects.push_back(object_msg);
  track_to_track_fusion::NonCopyableObjects objects(objects_msg);

  auto const prediction_time = ros::Time(10);
  predictor->predict(objects, prediction_time);
  ASSERT_EQ(objects.header.stamp, prediction_time);
}

TEST(constant_velocity, velocity_zero)
{
  auto predictor = std::make_unique<objects_prediction::ConstantVelocityPredictor>();

  perception_kit_msgs::Objects objects_msg;

  perception_kit_msgs::Object object_msg;
  object_msg.position.x = 0;
  object_msg.position.y = 0;
  object_msg.position.z = 0;
  object_msg.velocity.x = 0;
  object_msg.velocity.y = 0;
  object_msg.velocity.z = 0;
  objects_msg.objects.push_back(object_msg);
  track_to_track_fusion::NonCopyableObjects objects(objects_msg);

  auto const prediction_time = ros::Time(10);
  predictor->predict(objects, prediction_time);

  ASSERT_EQ(objects.header.stamp, prediction_time);
  ASSERT_EQ(objects.objects.size(), 1u);
  ASSERT_EQ(objects.objects.front().position.x, 0);
  ASSERT_EQ(objects.objects.front().position.y, 0);
  ASSERT_EQ(objects.objects.front().position.z, 0);
  ASSERT_EQ(objects.objects.front().velocity.x, 0);
  ASSERT_EQ(objects.objects.front().velocity.y, 0);
  ASSERT_EQ(objects.objects.front().velocity.z, 0);
}

TEST(constant_velocity, velocity_predict)
{
  auto predictor = std::make_unique<objects_prediction::ConstantVelocityPredictor>();

  auto const object_timestamp = ros::Time(0.1);
  auto const prediction_time = ros::Time(4.1);
  auto const time_diff = (prediction_time - object_timestamp).toSec();

  auto const px = 10.0;
  auto const py = -4.2;
  auto const pz = 200;

  auto const vx = 7.8;
  auto const vy = -20.0;
  auto const vz = 3.9;

  perception_kit_msgs::Objects objects_msg;

  perception_kit_msgs::Object object_msg;
  object_msg.header.stamp = object_timestamp;
  object_msg.position.x = px;
  object_msg.position.y = py;
  object_msg.position.z = pz;

  object_msg.velocity.x = vx;
  object_msg.velocity.y = vy;
  object_msg.velocity.z = vz;
  objects_msg.objects.push_back(object_msg);
  track_to_track_fusion::NonCopyableObjects objects(objects_msg);

  predictor->predict(objects, prediction_time);

  ASSERT_EQ(objects.header.stamp, prediction_time);
  ASSERT_EQ(objects.objects.size(), 1u);

  ASSERT_DOUBLE_EQ(objects.objects.front().position.x, px + vx * time_diff);
  ASSERT_DOUBLE_EQ(objects.objects.front().velocity.x, vx);
  ASSERT_DOUBLE_EQ(objects.objects.front().position.y, py + vy * time_diff);
  ASSERT_DOUBLE_EQ(objects.objects.front().velocity.y, vy);
  ASSERT_DOUBLE_EQ(objects.objects.front().position.z, pz + vz * time_diff);
  ASSERT_DOUBLE_EQ(objects.objects.front().velocity.z, vz);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
