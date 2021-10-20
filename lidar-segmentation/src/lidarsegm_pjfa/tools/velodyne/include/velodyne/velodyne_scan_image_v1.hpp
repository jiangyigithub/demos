#ifndef VELODYNE_SCAN_IMAGE_V1_HPP_INCLUDED
#define VELODYNE_SCAN_IMAGE_V1_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/container/vfc_carray.hpp"
#include "vfc/image/vfc_imageview.hpp"

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

  struct ScanImagePixel
  {
    static constexpr vfc::uint8_t PT_RESET_BITS = 0;            // Default (point without flags)
    static constexpr vfc::uint8_t PT_LOCAL_NON_GROUND_BIT  = 1; // Bit is set if local non-ground criterion was positive
    static constexpr vfc::uint8_t PT_LOCAL_NOT_CHECKED_BIT = 2; // Bit is set if local non-ground criterion could not check the bit

    PointProjection point;     // xyz coordinates of point in sensor coordinate frame
    vfc::float64_t stamp;      // measurement time of this point
    vfc::float32_t distance;   // spherical distance from sensor (spherical distance image) in meters
    vfc::float32_t depth;      // cylindrical (xy-plane) distance from sensor in meters (cylindrical image) in meters
    vfc::uint8_t   intensity;  // return intensitiy in range 0-255
    vfc::uint8_t   pointFlags; // flags associated with this point (e.g. ground)
    vfc::int8_t    valid;      // is this pixel a valid pixel?
    vfc::int8_t    padding;    // padding to fit exactly 32 bytes with this struct
  };


  struct VelodyneScanImage_v1
  {
    enum { VERSION = 1 };

    enum
    {
      MAX_NUMBER_OF_IMAGE_COLUMNS = 800, // maximum number of image columns per scan image
      SENSOR_WIDTH = 2048,               // sensor width (full spin) in image columns
      MAX_LASERS = 64                    // maximum number of lasers == maximum image height
    };

    vfc::int32_t start_col;          // Start column w.r.t. full sensor width
    vfc::float64_t min_time;         // Minimum timestamp w.r.t. all pixel timestamps in image
    vfc::float64_t max_time;         // Maximum timestamp w.r.t. all pixel timestamps in image

    vfc::TImageView<ScanImagePixel> scan_image;
    vfc::TCArray<ScanImagePixel, MAX_NUMBER_OF_IMAGE_COLUMNS*MAX_LASERS> scan_image_data;

  };
} //namespace

#endif
