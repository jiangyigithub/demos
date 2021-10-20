
#ifndef LIPE_SEGMENTER_H
#define	LIPE_SEGMENTER_H
#include <GlobalHeader.h>
#include <config.h>
#include "grid.h"

namespace lipe{

class Segmenter {
  public:
    Segmenter( const SegmenterConfig& segm_config, const LidarConfig& lid_config )
    : 
    segm_config_( &segm_config ), 
    lidar_config_( &lid_config ) 
    {};
    
    /**
    * Do the segmentation on the point cloud
    *
    * @param cloud is a Cloud*, pointer to the actual point cloud, which contains the points from the LiDar meaurement
    * @param cloud_size is the number of the valid points in the Cloud
    * @param grid is a reference to the Grid world, the cell of the Grid world will store the result of segmentation
    * @returns If the given cloud is empty
    */
    bool ExecuteSegmentation( Cloud* cloud, long int cloud_size, Grid& grid );

  private:
    
    /**
    * Do the segmentation on a single cell. It decides the type of the cell
    *
    * @param act_cell is a Cell*, a pointer to the actual cell
    * @param act_cell_prop is CellProp*, a pointer to the prperties of the actual cell
    */
    void ClassifyCell( Cell* act_cell, CellProp* act_cell_prop );
    
    /**
    * In case of Ibeo LiDar, there are some ghost object and the Ibeo has 4 seperate layer.
    * If all of the points belongs only one layer, the cell classified as a cekk with ghost object.
    * This function check if all the points in the cell belongs to the same layer
    *
    * @param pointer to the actual cell
    * @returns If all of the points in the cell are belongs to the same LiDar layer
    */
    bool multipleLayersInCell( Cell* act_cell );
    const SegmenterConfig* segm_config_;
    const LidarConfig* lidar_config_;
    };
};
#endif
