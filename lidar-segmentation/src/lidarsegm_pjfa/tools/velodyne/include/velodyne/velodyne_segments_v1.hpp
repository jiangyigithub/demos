#ifndef VELODYNE_SEGMENTS_V1_HPP_INCLUDED
#define VELODYNE_SEGMENTS_V1_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/container/vfc_carray.hpp"

namespace velodyne
{
  struct VelodyneSegments_v1
  {
    enum { VERSION = 1 };

    enum
    {
      MAX_NUM_PTS = 65536, // maximal number of points that the segments are comrised of
      MAX_NUM_SEGMENTS = 512, // maximal number of segments in message
      MAX_NUM_CONTOUR_PTS = 1024, // maximal number of points per contour
      MAX_NUM_NONOBSTACLE_SEGMENTS = 32 // maximum number of non-obstacle segments in message
    };

    /**
     * @brief
     */
    struct PointProjectionXYZ
    {
      vfc::float32_t x;
      vfc::float32_t y;
      vfc::float32_t z;
    };

    struct SensorScanImagePosition
    {
      vfc::int16_t col;
      vfc::int16_t row;
    };

    /**
     * @brief 
     */
    struct ContourPoint
    {
      vfc::float32_t x;
      vfc::float32_t y;
      vfc::float32_t v_xx;
      vfc::float32_t v_xy;
      vfc::float32_t v_yy;
      vfc::float32_t z_min;
      vfc::float32_t z_max;
    };

    /**
     * @brief 
     */
    struct Contour
    {
      vfc::TCArray<ContourPoint, MAX_NUM_CONTOUR_PTS> points;
    };

    /**
     * @brief
     */
    struct Segment
    {
      vfc::uint16_t first_pt_ix; // index into points array
      vfc::uint16_t num_pts; // number of points in segment
      vfc::float64_t start_time; // measurement time of first point (Velodyne rotates clockwise (top-down))
      vfc::float64_t end_time; // measurement time of last point (Velodyne rotates clockwise (top-down))y
      vfc::uint8_t num_contour_pts; // number of contour points the contour comprises
      Contour contour;
    };

    vfc::TCArray<PointProjectionXYZ, MAX_NUM_PTS> points;
    vfc::TCArray<vfc::float64_t, MAX_NUM_PTS> points_stamps; // measurement timestamp in seconds
    vfc::TCArray<SensorScanImagePosition, MAX_NUM_PTS> sensor_img_positions;
    vfc::uint16_t num_segments; // number of segments
    vfc::TCArray<Segment, MAX_NUM_SEGMENTS> segments;

    vfc::TCArray<PointProjectionXYZ, MAX_NUM_PTS> nonobstacle_points;
    vfc::TCArray<SensorScanImagePosition, MAX_NUM_PTS> nonobstacle_sensor_img_positions;
    vfc::uint16_t num_nonobstacle_segments; // number of segments
    vfc::TCArray<Segment, MAX_NUM_NONOBSTACLE_SEGMENTS> nonobstacle_segments;
  };
} //namespace

#endif
