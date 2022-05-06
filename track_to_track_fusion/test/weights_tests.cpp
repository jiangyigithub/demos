#include <gtest/gtest.h>
#include "track_to_track_fusion/object_with_trace.h"

#define private public
#define protected public
#include "track_to_track_fusion/weights.h"

TEST(Weights, default_weight)
{
  track_to_track_fusion::Weights w;
  perception_kit_msgs::Object o;

  ASSERT_FLOAT_EQ(w.getWeight("lidar", "position", o.position), 1.0f);
}

TEST(Weights, constructor_weight)
{
  track_to_track_fusion::Weights w(0.2);
  perception_kit_msgs::Object o;

  ASSERT_FLOAT_EQ(w.getWeight("lidar", "position", o.position), 0.2f);
}

TEST(Weights, constructor_throw)
{
  ASSERT_THROW(track_to_track_fusion::Weights w(-0.2), std::invalid_argument);
}

TEST(Weights, set_weight_throw)
{
  track_to_track_fusion::Weights w;
  ASSERT_THROW(w.setWeight("lidar", "position", -0.4f), std::invalid_argument);
}

TEST(Weights, set_weight)
{
  track_to_track_fusion::Weights w;
  w.setWeight("lidar", "position", 0.4f);

  perception_kit_msgs::Object o;
  auto const ret = w.getWeight("lidar", "position", o.position);
  ASSERT_FLOAT_EQ(ret, 0.4f);
}

TEST(Weights, set_polygon_weight)
{
  track_to_track_fusion::Weights w;
  track_to_track_fusion::Weights::boost_polygon poly;

  boost::geometry::read_wkt("POLYGON((-1 -1, -1 1, 1 1 , 1 -1 ))", poly);
  w.setPolygonWeight("lidar", "position", poly, 0.7f);

  perception_kit_msgs::Object o;
  auto const ret = w.getWeight("lidar", "position", o.position);
  ASSERT_FLOAT_EQ(ret, 0.7f);

  o.position.x = 2;
  auto const ret_outside = w.getWeight("lidar", "position", o.position);
  ASSERT_FLOAT_EQ(ret_outside, 1.0f);

  w.setWeight("lidar", "position", 2.0f);
  auto const ret_outside_set = w.getWeight("lidar", "position", o.position);
  ASSERT_FLOAT_EQ(ret_outside_set, 2.0f);
}

TEST(Weights, objects_with_trace)
{
  track_to_track_fusion::ObjectWithTrace::Trace trace;
  trace["lidar"] = 0;
  trace["radar"] = 0;

  track_to_track_fusion::Weights w;
  w.setWeight("lidar", "position", 0.2f);
  w.setWeight("radar", "position", 0.7f);

  perception_kit_msgs::Object o;
  auto const ret = w.getWeight(trace, "position", o.position);
  ASSERT_FLOAT_EQ(ret, 0.9f);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
