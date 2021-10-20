#ifndef VELODYNE_SCAN_V1_HPP_INCLUDED
#define VELODYNE_SCAN_V1_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/container/vfc_carray.hpp"

namespace common_daddy_interfaces
{
  /**
   * @brief The PointProjection struct
   * Projected point in the sensor coordinate frame
   */
  struct PointProjection
  {
    vfc::float32_t x;
    vfc::float32_t y;
    vfc::float32_t z;
  };

  struct ScanPoint
  {
    vfc::uint8_t laser_id;        // layer idendifier as provided by sensor
    vfc::float32_t range;
    vfc::uint8_t intensity;       // in range 0-255
    vfc::float32_t encoder_angle; // encoder position (in rad) clockwise | zero aligned with sensor x axis
    vfc::uint8_t return_ix;       // set if the sensor is able to provide multiple returns per measurement
    PointProjection point;
  };


  struct VelodyneScan_v1
  {
    enum { VERSION = 1 };

    enum
    {
      NUM_POINTS_PER_STACK = 384, // 32 points * 12 scans per udp packet
      NUM_STACKS = 5,
      MAX_LASERS = 64
    };

    /**
     * @brief The VelodyneUDPPointStack struct
     * contains all points originating from a single UDP packet
     */
    struct PointStack
    {
      vfc::TCArray<ScanPoint, NUM_POINTS_PER_STACK> scan_points;
    };


    vfc::TCArray<PointStack, NUM_STACKS> point_stacks;

  };
} //namespace

#endif
