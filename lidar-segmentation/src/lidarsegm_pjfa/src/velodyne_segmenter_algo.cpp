#include <velodyne_segmenter_algo.h>
#include <logger.h>

namespace lipe{

  VelodyneSegmenterAlgo::VelodyneSegmenterAlgo():
    grid_config(),
    lidar_config(),
    segmenter_config(),
    detector_config(),
    kalman_config(),
    tracker_config(),
    grid(grid_config, lidar_config),
    segm(segmenter_config, lidar_config),
    detector(detector_config, kalman_config, grid),
    tracker(tracker_config),
    free_space()
  {
  }

  bool VelodyneSegmenterAlgo::checkInputPoint(const float& x, const float& y, const float& z) const
  {
    // check for isnan
    if( true == std::isnan(x) ) { return false; }
    if( true == std::isnan(y) ) { return false; }
    if( true == std::isnan(z) ) { return false; }

    // check if it is in the grid.
    if( false == getGrid().checkIfInputPointInRange(x, y))
    {
      return false;
    }

    // check height:
    if( false == getGrid().checkInputPointHeight(z))
    {
      return false;
    }
    
    return true;
  }

  const Grid& VelodyneSegmenterAlgo::getGrid() const
  {
    return grid;
  }

  const FreeSpace& VelodyneSegmenterAlgo::getFreeSpace() const
  {
    return free_space;
  }

  long int VelodyneSegmenterAlgo::getMaxNumberOfLidarPoints() const
  {
    return lidar_config.max_number_of_lidar_points;
  }
  
  void VelodyneSegmenterAlgo::onInit()
  {
    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenterAlgo] [onInit] start";
    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenterAlgo] [onInit] finished";
  }
  
  Blob* VelodyneSegmenterAlgo::GetTrackedObjectList()
  {
    return tracker.GetTrackedObjects();
  }
  
  int VelodyneSegmenterAlgo::GetMaxTrackedLen()
  {
    return tracker.GetMaxTrackedLen();
  }

  void VelodyneSegmenterAlgo::run(Cloud& cloud_raw, long int cloud_size, visualization_msgs::MarkerArray& marker_arr)
  {
    
    float full_det_time = clock();
    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenterAlgo] [run] start";
    BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenterAlgo] [run] cloud_size = " << cloud_size;

    // segmentation step
    float time = clock();
    segm.ExecuteSegmentation(&cloud_raw, cloud_size, grid);
    double execution_time = (((float) clock() - time) / CLOCKS_PER_SEC)*1000;
    BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenterAlgo] [run]" << " Grid setup and segmentation runtime: " << execution_time << " ms";  
        
    time = clock();
    // detection step
    detector.FillBlobList();
    detector.ClassifiyBlobs();
    execution_time = (((float) clock() - time) / CLOCKS_PER_SEC)*1000;
    BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenterAlgo] [run]" << " Number of detected objects: " << detector.current_object_list_len_;  
    BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenterAlgo] [run]" << " Blob collection and classification runtime: " << execution_time << " ms";  

    time = clock();
    // tracking step
    tracker.Run(detector.object_list_, detector.current_object_list_len_);
    execution_time = (((float) clock() - time) / CLOCKS_PER_SEC)*1000;
    BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenterAlgo] [run]" << " Number of tracked objects: " << tracker.GetNumTracked();  
    BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenterAlgo] [run]" << " Tracking runtime: " << execution_time << " ms";  
    
    time = clock();
    // free space calcualtion step
    free_space.Reset();       
    free_space.CalcFreeSpaceForBP(grid);
    execution_time = (((float) clock() - time) / CLOCKS_PER_SEC)*1000;
    BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenterAlgo] [run]" << " FreeSpace runtime: " << execution_time << " ms";

    full_det_time = (((float) clock() - full_det_time) / CLOCKS_PER_SEC)*1000;
    BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenterAlgo] [run]" << " Full segmentation+detection runtime: " << full_det_time << " ms";
    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenterAlgo] [run] finished";

  }

}  // namespace