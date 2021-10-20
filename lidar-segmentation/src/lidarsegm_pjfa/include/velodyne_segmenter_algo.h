/**
 * The algo class for "lipe". 
 * 
 * It runs the steps of the algo in sequence.
 * 
 * @todo add license information here. In the meantime this is the property of Robert Bosch Kft, all rights reserved, use your own risk, etc.
 * 
 * @author Péter Lakatos (CC-AD/ENG1-Bp)
 */

#ifndef LIPE_VELODYNE_SEGMENTER_ALGO_H_
#define LIPE_VELODYNE_SEGMENTER_ALGO_H_

#include "segmenter.h"
#include "grid.h"
#include "config.h"
#include "lipe_interface.h"
#include "blobdetector.h"
#include "visualizer.h"
#include "tracker.h"
#include <visualization_msgs/MarkerArray.h>

namespace lipe {

  /**
   * The algo class for "lipe". 
   * 
   * It runs the steps of the algo in sequence.
   * It could be use as a member of a ROS node object, or as a standalone object as well.
   * 
   * # Basic usage:
   * - run the "onInit" function before the first usage. For example in the input callback function of the parent ROS node of this object.
   * - run the "run" function for every algo input. For example in the input callback function of the parent ROS node of this object.
   * - finetune the parameters in the "_config" structs.
   */
  class VelodyneSegmenterAlgo
  {

    public:

      /**
       * Default constructor.
       */
      VelodyneSegmenterAlgo();

      /**
       * Default initializer.
       * 
       * Should be called once before first usage. 
       * For example in the "onInit" function of the parent ROS node of this object.
       */
      void onInit();

      /**
       * Processing one algo input.
       * 
       * Should be called once for every input.
       * For example in the input callback function of the parent ROS node of this object.
       * 
       * @param cloud_raw A pcl pointcloud contining the deskewed and odometry-compensated velodyne point cloud
       * @param cloud_size The number of valid points in the cloud.
       * @param marker_arr a visualization msg for ros, which contains data regarding the centers, corners and IDs of objects
       */
      void run(Cloud& cloud_raw, long int cloud_size, visualization_msgs::MarkerArray& marker_arr);

      /**
       * Default getter for the grid.
       * 
       * @return a const reference for the grid.
       */
      const Grid& getGrid() const;

      /**
       * Default getter for the freespace calculator.
       * 
       * @return a const reference for the freespace calculator.
       */
      const FreeSpace& getFreeSpace() const;

      /**
       * Checks if the three values are forming a valid input point.
       * 
       * Check if neither of them is nan and they are inside the grid, and has a valid height.
       * 
       * @return True if the point is admissable for the grid, False otherwise.
       */
      bool checkInputPoint(const float& x, const float& y, const float& z) const;

      /**
       * Default getter for the max number of lidar points.
       * 
       * @return the max number of lidar points based on the lidar_config.
       */
      long int getMaxNumberOfLidarPoints() const;
      
      /**
       * @brief returns the tracked object list of the tracker. this is called by the velodyne segmenter and passed to the visualizer in every spin 
       */
      Blob* GetTrackedObjectList();
      int GetMaxTrackedLen();

    private:

      // configs:
      GridConfig grid_config;                 /**< Configs for the grid */
      LidarConfig lidar_config;               /**< Configs for the lidar parameters */
      SegmenterConfig segmenter_config;       /**< Configs for the segmenter */
      DetectorConfig detector_config;         /**< Configs for the detector */
      KalmanFilterConfig kalman_config;       /**< Configs for the Kalman-filter */
      TrackerConfig tracker_config;           /**< Configs for the tracker */
      

      Grid grid;                              /**< The grid */
      Segmenter segm;                         /**< The grid segmenter */
      BlobDetector detector;                  /**< The blob detector */
      FreeSpace free_space;                   /**< The free space calculator */
      Tracker tracker;
  };
};

#endif  // LIPE_VELODYNE_SEGMENTER_ALGO_H_
