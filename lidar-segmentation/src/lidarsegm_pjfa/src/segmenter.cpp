
#include "../include/segmenter.h"
namespace lipe {

  void Segmenter::ClassifyCell( Cell* act_cell, CellProp* act_cell_prop ){

    // IBEO
    if( lidar_config_->type == LidarTypes::IBEO ) {
      if( act_cell->num_of_points > act_cell_prop->min_num_of_points &&
          ( multipleLayersInCell( act_cell ) ) ) {
        // ground
        if ( ( act_cell->max_z - act_cell->min_z ) < segm_config_->ground_height_difference &&
            act_cell->min_z < segm_config_->ground_height ) {
          act_cell->label = ROAD;
        }
        else {
          act_cell->label = PointClasses::NOT_CLASSIFIED;
        }
      }
      else {
        act_cell->label = PointClasses::SPARSE;
      }
    }
    else if( lidar_config_->type == LidarTypes::VELODYNE ){

      if(act_cell_prop->x_dist_from_center < segm_config_->ignore_dist_x && act_cell_prop->y_dist_from_center < segm_config_->ignore_dist_y){
        act_cell->label = PointClasses::SPARSE;
      }
      else if( act_cell->num_of_points > segm_config_->min_point_num ) {
        // ground
        if ( ( act_cell->max_z - act_cell->min_z ) < segm_config_->ground_height_difference &&
            act_cell->min_z < segm_config_->ground_height ) {
          act_cell->label = PointClasses::ROAD;
        }
        else {
          act_cell->label = PointClasses::NOT_CLASSIFIED;
        }
      }
      else {
        act_cell->label = PointClasses::SPARSE;
      }
    }
  }

  bool Segmenter::ExecuteSegmentation( Cloud* cloud, long int cloud_size, Grid& grid ) {
    
    if ( cloud->empty() )
    {
      return false;
    }

    grid.FillGrid(cloud, cloud_size);
        
    for( uint i = 0; i < grid.GetNrRows(); ++i ){
      for( uint j = 0; j < grid.GetNrColumns(); ++j ) {
        Cell* act_cell = & ( grid.cells_[ i ][ j ] );
        CellProp* act_cell_prop = & ( grid.cells_prop_[ i ][ j ] );
        ClassifyCell( act_cell, act_cell_prop );
      }
    }
    
    return true;
  }

  bool Segmenter::multipleLayersInCell( Cell* act_cell ) {
    int points_in_layer = 0;
    for( int i = 0; i < LAYERS_IN_IBEO; ++i )
    {
      points_in_layer += act_cell->num_of_point_in_layer[ i ] > 0
                       ? 1
                       : 0;
    }
    return points_in_layer > 1;
  }
}
