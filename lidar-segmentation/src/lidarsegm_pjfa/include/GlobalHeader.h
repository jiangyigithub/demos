//TODO: rename to lipe_types.h
//TODO: check the inclusions
//TODO: define pcl::RGB colors here

#ifndef GLOBALHEADER_H
#define	GLOBALHEADER_H

#include <string>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <climits>
#include <unordered_set>

#include <memory>
#include <unordered_map>
#include <functional>
#include <array>
#include <thread>
#include <mutex>
#include <limits>
#include <stack>

#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/MarkerArray.h>


// We can't move these inside the lipe namespace
// See https://github.com/PointCloudLibrary/pcl/issues/1152 for details.
// @todo - major refactoring suggestion - remove pcl point type and cloud altogether, we do not really use it. Use it only for visu, if necessary. Inside use an own storage, without the nr_valid_points.
struct PCLMyPointType {
    //PCL data
    // preferred way of adding a XYZ+padding
    PCL_ADD_POINT4D;

    //PCL rgb
    PCL_ADD_RGB

    //data from reading
    float Intensity;

    //segmentation class
    int cl; 
    uint channel;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned

} EIGEN_ALIGN16; // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PCLMyPointType, // here we assume a XYZ + "test" (as fields)
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (float, Intensity, Intensity)
        );


namespace lipe{

  typedef unsigned int uint;

  typedef ::PCLMyPointType PCLMyPointType;
  typedef pcl::PointCloud<PCLMyPointType> Cloud;
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

  enum PointClasses {
    ROAD = 0,
    SPARSE = 1,
    WALL = 2,
    OBJECT = 3,
    NOT_CLASSIFIED = 4
  };

  enum FreeSpaceMsgModes{
    FROM_GROUND,
    FROM_OBJECTS,
    COMBINED,
    MAX_MODE
  };

}

#endif
