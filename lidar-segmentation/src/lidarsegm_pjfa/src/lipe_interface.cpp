/*
 * lipe_interface.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: kvv2bp
 */

#include "lipe_interface.h"

namespace lipe {

  // @todo - move these to a config file or create an enum from them.  
  #define INVALID_DISTANCE_ROAD 0
  #define INVALID_DISTANCE_OBJECT 999.0

  void FreeSpace::CalcFreeSpaceForBP( const Grid& grid ) {
    for( uint i = 0; i < grid.GetNrRows(); ++i )
    {
      for( uint j = 0; j < grid.GetNrColumns(); ++j )
      {
        Cell* act_cell = &(grid.cells_[ i ][ j ]);
        CellProp* act_cell_prop = &(grid.cells_prop_[ i ][ j ]);
        if( act_cell->label == PointClasses::ROAD )
        {
          unsigned int idx = static_cast< unsigned int >( act_cell_prop->angle_from_ego );
          float* from_ground = &free_space_in_polar_[ FreeSpaceMsgModes::FROM_GROUND ][ idx ];
          if( std::abs( *from_ground ) < std::abs( act_cell_prop->dist_from_ego ) )
          {
            *from_ground = act_cell_prop->dist_from_ego;
          }
        }
        else if( act_cell->label == PointClasses::OBJECT ||
                 act_cell->label == PointClasses::NOT_CLASSIFIED ||
                 act_cell->label == PointClasses::WALL )
        {
          unsigned int idx = static_cast< unsigned int >( act_cell_prop->angle_from_ego );
          float* from_objects = &free_space_in_polar_[ FreeSpaceMsgModes::FROM_OBJECTS ][ idx ];
          if( std::abs( *from_objects ) > std::abs( act_cell_prop->dist_from_ego ) )
          {
            *from_objects = act_cell_prop->dist_from_ego;
          }
        }
      }
    }
    for( uint i = 0; i < 360; ++i )
    {
      float* combined = &free_space_in_polar_[ FreeSpaceMsgModes::COMBINED ][ i ];
      float* from_ground = &free_space_in_polar_[ FreeSpaceMsgModes::FROM_GROUND ][ i ];
      float* from_objects = &free_space_in_polar_[ FreeSpaceMsgModes::FROM_OBJECTS ][ i ];
      (*combined = *from_ground - INVALID_DISTANCE_ROAD) < 1e-4
                || *from_ground > *from_objects
                ? *from_objects
                : *from_ground;
    }
  }

  void FreeSpace::Reset()
  {
    for( int i = 0; i < 360; i++ )
    {
      free_space_in_polar_[ FreeSpaceMsgModes::FROM_GROUND ][ i ] = INVALID_DISTANCE_ROAD;
      free_space_in_polar_[ FreeSpaceMsgModes::FROM_OBJECTS ][ i ] = INVALID_DISTANCE_OBJECT;
      free_space_in_polar_[ FreeSpaceMsgModes::COMBINED ][ i ] = INVALID_DISTANCE_OBJECT;
    }
  }
}



