/*
 * lipe_interface.h
 *
 *  Created on: Nov 21, 2018
 *      Author: kvv2bp
 */

#ifndef LIPE_LIPE_INTERFACE_H_
#define LIPE_LIPE_INTERFACE_H_

#include "grid.h"

namespace lipe{

  class FreeSpace
  {
    public:
      float free_space_in_polar_[ FreeSpaceMsgModes::MAX_MODE ][ 360 ];   // @todo - magic number - move it to config
      
      /**
      * Reinitialize the freespace values
      */
      void Reset();
      FreeSpace() { Reset(); }
      
      /**
      * The behaviour prediction needs the freespace information in a polar coordinate system, around the
      * This function calculate this in 3 methods: It check the farthest ground cell, the closest object,
      * and the combined method store the smaller distance between the 2 previous one
      *
      * @param grid is a pointer to the grid world
      */
      void CalcFreeSpaceForBP( const Grid& grid );
  };
}




#endif /* LIPE_LIPE_INTERFACE_H_ */
