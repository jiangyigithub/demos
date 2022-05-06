#include <gtest/gtest.h>
#include "track_to_track_fusion/cost/cost_functions.h"
#include <perception_kit_msgs/Object.h>

TEST(cost_functions, CostCalculation)
{
  track_to_track_fusion::cost_calculation::CostCalculation c(
      [](perception_kit_msgs::Object const& a, perception_kit_msgs::Object const& b) {
        return fabs(a.position.x - b.position.x);
      },
      123.0f);
  ASSERT_EQ(c.threshold(), 123.0f);

  perception_kit_msgs::Object a;
  perception_kit_msgs::Object b;
  a.position.x = 10;
  b.position.x = 100;

  ASSERT_EQ(c.cost()(a, b), 90.0f);
}

TEST(cost_functions, euclidean_distance_sqrt_throw)
{
  perception_kit_msgs::Object a;
  a.header.frame_id = "foo";

  perception_kit_msgs::Object b;
  b.header.frame_id = "bar";

  ASSERT_THROW(track_to_track_fusion::cost_calculation::euclideanDistanceSqrt(a, b), std::runtime_error);
}

TEST(cost_functions, euclidean_distance_throw)
{
  perception_kit_msgs::Object a;
  a.header.frame_id = "foo";

  perception_kit_msgs::Object b;
  b.header.frame_id = "bar";

  ASSERT_THROW(track_to_track_fusion::cost_calculation::euclideanDistance(a, b), std::runtime_error);
}

TEST(cost_functions, euclidean_distance_sqrt)
{
  perception_kit_msgs::Object a;
  a.position.x = 9;
  a.position.y = 0;
  a.position.z = 12;

  perception_kit_msgs::Object b;
  b.position.x = -1;
  b.position.y = 10;
  b.position.z = 2;

  auto const distance_ab = track_to_track_fusion::cost_calculation::euclideanDistanceSqrt(a, b);
  ASSERT_FLOAT_EQ(distance_ab, 300.0);

  auto const distance_ba = track_to_track_fusion::cost_calculation::euclideanDistanceSqrt(b, a);
  ASSERT_FLOAT_EQ(distance_ba, 300.0);
}

TEST(cost_functions, euclidean_distance)
{
  perception_kit_msgs::Object a;
  a.position.x = 4;
  a.position.y = 0;
  a.position.z = 2;

  perception_kit_msgs::Object b;
  b.position.x = -1;
  b.position.y = 10;
  b.position.z = 7;

  auto const distance_ab = track_to_track_fusion::cost_calculation::euclideanDistance(a, b);
  ASSERT_FLOAT_EQ(distance_ab, sqrt(150.0f));

  auto const distance_ba = track_to_track_fusion::cost_calculation::euclideanDistance(b, a);
  ASSERT_FLOAT_EQ(distance_ba, sqrt(150.0f));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
