/**
 * The visualizer class for "lipe". 
 * 
 * It runs the steps of the algo in sequence.
 * 
 * @todo add license information here. In the meantime this is the property of Robert Bosch Kft, all rights reserved, use your own risk, etc.
 * 
 * @author Károly Harsányi (CC-AD/EAU-BP)
 */

#ifndef LIPE_VISUALIZER_H_
#define	LIPE_VISUALIZER_H_

#include "GlobalHeader.h"
#include "grid.h"

namespace lipe{
  
  class Visualizer{
  public:
       // pretty basic constructor with a config
       Visualizer(VisualizerConfig& config);
  
      /**
       * @brief goes through to points of the cloud and colors each point according to the class of their corresponding cell
       * @param grid
       * @param cloud
       * @param cloud_size
       */
      void ColorCloud(const Grid* grid, Cloud* cloud, long int cloud_size); 
      /**
       * @brief goes through to tracked object array and fills a marker array based on the params of the objects
       * @param tracked_objects array originating from the tracker of the segmenter_algo
       * @param length of the tracked_object array
       * @param the MarkerArray to be filled
       */  
      void FillMarkerArray(Blob* tracked_objects, int max_tracked_objects_len, visualization_msgs::MarkerArray* arr);
      
      
  private:
      VisualizerConfig* config_;
      
      // assigns a color for each object, based on their ID
      pcl::RGB IDToColor(int ID);
      
      // ~ getters with the colors of each class
      pcl::RGB GetUnvisualizedObjectColor();
      pcl::RGB GetWallColor();
      pcl::RGB GetGroundColor();
      pcl::RGB GetNoiseColor();
      pcl::RGB GetUnknownColor();
      pcl::RGB GetObjectColor();
      pcl::RGB GetRandomColor();      
      
      // runs over the cloud's points and colors them based on the class of their containing cell in the segmentation
      void ColorPointSegmentation(Cell* cell, PCLMyPointType* point);
      
      // runs over the cloud's points and colors them based on the class of their containing cell in the detection
      void ColorPointDetection(Cell* cell, PCLMyPointType* point);
      
      // sets some basic params for the input marker
      void SetMarkerParams(visualization_msgs::Marker* marker);
  };
  
}
#endif // LIPE_VISUALIZER_H_
