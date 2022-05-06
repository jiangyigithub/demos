#include <gtest/gtest.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <ros/ros.h>
#pragma GCC diagnostic pop

#define private public
#include "track_to_track_fusion/feature/feature_fusion_elements.h"

#define TEST_COMMUTATIVE(MEMBER)                                                                                       \
  perception_kit_msgs::Object a;                                                                                       \
  perception_kit_msgs::Object b;                                                                                       \
  perception_kit_msgs::Object output_a_b;                                                                              \
  perception_kit_msgs::Object output_b_a;                                                                              \
                                                                                                                       \
  a.MEMBER = 0.5f;                                                                                                     \
  b.MEMBER = 0.8f;                                                                                                     \
                                                                                                                       \
  float const w_a = 1.0f;                                                                                              \
  float const w_b = 1.0f;                                                                                              \
                                                                                                                       \
  fusion.fuse(a, b, w_a, w_b, output_a_b.MEMBER);                                                                      \
  fusion.fuse(b, a, w_b, w_a, output_a_b.MEMBER);                                                                      \
                                                                                                                       \
  ASSERT_FLOAT_EQ(output_a_b.MEMBER, output_a_b.MEMBER);

#define TEST_ASSOCIATIVE(MEMBER)                                                                                       \
                                                                                                                       \
  perception_kit_msgs::Object a;                                                                                       \
  perception_kit_msgs::Object b;                                                                                       \
  perception_kit_msgs::Object c;                                                                                       \
  perception_kit_msgs::Object ab;                                                                                      \
  perception_kit_msgs::Object bc;                                                                                      \
  perception_kit_msgs::Object ab_c;                                                                                    \
  perception_kit_msgs::Object a_bc;                                                                                    \
                                                                                                                       \
  float const w_a = 1.0f;                                                                                              \
  float const w_b = 1.0f;                                                                                              \
  float const w_c = 1.0f;                                                                                              \
  float const w_ab = w_a + w_b;                                                                                        \
  float const w_bc = w_b + w_c;                                                                                        \
                                                                                                                       \
  a.MEMBER = 0.5f;                                                                                                     \
  b.MEMBER = 0.8f;                                                                                                     \
  c.MEMBER = 0.3f;                                                                                                     \
                                                                                                                       \
  fusion.fuse(a, b, w_a, w_b, ab.existence_probability);                                                               \
  fusion.fuse(ab, c, w_ab, w_c, ab_c.existence_probability);                                                           \
  ASSERT_FLOAT_EQ(ab.existence_probability, 0.65f);                                                                    \
                                                                                                                       \
  fusion.fuse(b, c, w_b, w_c, bc.existence_probability);                                                               \
  fusion.fuse(a, bc, w_a, w_bc, a_bc.existence_probability);                                                           \
  ASSERT_FLOAT_EQ(bc.existence_probability, 0.55f);                                                                    \
                                                                                                                       \
  ASSERT_FLOAT_EQ(a_bc.existence_probability, ab_c.existence_probability);                                             \
  ASSERT_FLOAT_EQ(a_bc.existence_probability, 0.533333333333333333f);

TEST(existence_probability, commutative)
{
  track_to_track_fusion::ExistenceProbabilityWeightedFusion const fusion;
  TEST_COMMUTATIVE(existence_probability)
}

TEST(existence_probability, associative)
{
  track_to_track_fusion::ExistenceProbabilityWeightedFusion const fusion;
  TEST_ASSOCIATIVE(existence_probability)
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
