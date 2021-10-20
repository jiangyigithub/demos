/*
 * grid.h
 *
 *  Created on: Nov 19, 2018
 *      Author: kvv2bp
 */

#ifndef LIPE_GRID_H_
#define LIPE_GRID_H_

#include "config.h"
#include "blob.h"

namespace lipe
{

  enum CellNeighbourTypes{
    NORTH      = 0,
    NORTH_WEST = 1,
    WEST       = 2,
    SOUTH_WEST = 3,
    SOUTH      = 4,
    SOUTH_EAST = 5,
    EAST       = 6,
    NORTH_EAST = 7,
    COUNT
  };

  struct Cell{
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
    uint num_of_points;

    int label;//0:noise, 1:ground, 2:object, 3:wall, 4:unknown

    // the blob that contains the cell
    bool has_blob;
    Blob* cont_blob;
    uint num_of_point_in_layer[ LAYERS_IN_IBEO ];   //@todo - major improvement suggestion - make this field and channel field of points are all a template parameter of LidarTypes. Or maybe just remove ibeo-related stuff altogether?
  };

  struct CellProp
  {
    float dist_from_ego;
    float angle_from_ego;
    float x_dist_from_center;
    float y_dist_from_center;
    uint min_num_of_points;
  };

  /**
   * The grid class for "lipe". 
   * 
   * @todo - finish the the description. This is only the description of the coordinate system
   * 
   * # Grid geometry:
   * 
   * ## Coordinate system:
   * The raw scan points are represented in the coordinate system of the velodyne 64 sensor.
   * This is a 3D cartesian coordinate system, the "regular" configuration is the following:
   * - The sensor (lidar) is in the center of the coordinate system.
   * - It is a right-handed coordinate system.
   * - The x-y plane is parallel to the ground.
   * - The z axis is vertical, pointing up.
   * - The x axis is pointing "forward". parallel to the longitudinal axis of the vehicle.
   * - The y axis is pointing "left", perpendicular to the longitudinal axis of the vehicle.
   * 
   * ## Indexing
   * The indexing of the grid cells in their matrix:
   * - Rows and columns are parallel to the axis of the aformentioned coordinate system.
   * - If we observe it from above, the indexing is started in the top-left corner.
   * - The first index is changing along the x axis. 
   * - The second index is changing along the y axis.
   * - Both indexing is increasing in a direction opposite the direction of the corresponding axis.   
   * - The origo of the aformentioned coordinate system is in the center of the central cell.
   * -- At least if we would project it to the ground.
   * -- The number of cells along both directions is odd.
   * 
   * 
   * If the size of grid is 2xN+1 x 2xM+1:
   * 
   * +-------+-------+-----
   * |       |       |
   * | (0,0) | (0,1) |
   * |       |       |
   * +-------+-------+-----
   * |       |       |
   * | (1,0) | (1,1) |
   * |       |       |
   * +-------+-------+-----
   * |       |       |
   *                                     ^ X
   *                                     |
   * |       |       |               |   |   |
   * +-------+-------+----        ---+---|---+----
   * |       |       |         Y     |   |   |
   * | (N,0) | (N,1) |        <----------0   |
   * |       |       |               |  (N,M)| 
   * +-------+-------+----        ---+-------+----
   * |       |       |               |       |
   */
  class Grid{
  private:
    const GridConfig* grid_config_;
    const LidarConfig* lidar_config_; 

    /**
    * Along the distance, fewer points are enough to classify the cell as not Sparse
    *
    * @param x index of the cell along x axis
    * @param y index of the cell along y axis
    * @returns the extra points
    */      
    uint GetExtraPointsForCell( int x, int y );
    
    /**
    * Update point statistics of the cell with the given point
    *
    * @param point: reference to the point, which would be added to the grid
    */
    void AddPoint( PCLMyPointType& point );

    /**
     * The number of grid cells from the origo in the direction of x axis.
     * 
     * @return N, if the size of the grid is 2*N+1.
    */
    uint GetHalfGridSizeX() const;

    /**
     * The number of grid cells from the origo in the direction of y axis.
     * 
     * @return N, if the size of the grid is 2*N+1.
    */
    uint GetHalfGridSizeY() const;

  public:
    Cell** cells_; //TODO: check template based solution
    CellProp** cells_prop_;
    
    
   /**
    * Constructor of grid

    * @param config is reference to a GridConfig, which contains the config parameters belonging to the grid
    * @param lidar_config is a reference to a LidarConfig, which contains the config parameters belonging to the actual LiDar
    */
    Grid( const GridConfig& config, const LidarConfig& lidar_config );
    
    /**
    * Reinitialize the parameters of the grid
    */
    void Reset();
    
    // @todo - commenting should be in the header file. See https://google.github.io/styleguide/cppguide.html#Function_Comments
    /**
    * Update the cells with the LiDar point cloud
    *
    * @param cloud pointer to the LiDar point cloud
    * @param cloud_size number of the valid points in the point cloud
    */
    void FillGrid( Cloud* cloud, long int cloud_size );
    

    uint GetNrRows() const { return grid_config_->nr_rows; }
    uint GetNrColumns() const { return grid_config_->nr_columns; }

    Cell* GetCellForPoint( PCLMyPointType& point ) const; //@todo - this is a private function - I only moved here for testing.
 

    
    /**
    * Calculate the distance from the cell with given index from the ego vehicle, which is in the midle of the grid. (0, 0) point is the top-left corner
    *
    * @param i index of the cell along x axis
    * @param j index of the cell along y axis
    * @returns distance the distance of the cell from the ego [m]
    */
    float GetCellDistanceFromEgo( int i, int j);    //@todo - this is a private function - I only moved here for testing.

    /**
     * Checks if the two values are inside the grid and the lidar range.
     * 
     * @return True if the point is inside the grid AND the range of the lidar, False otherwise.
    */
    bool checkIfInputPointInRange(const float& x, const float& y) const;

    /**
     * Checks if the height value is valid.
     * 
     * Checks again two values relative to the ground. These values are specified in GridConfig:
     * - input_point_lower_threshold_relative_to_ground
     * - input_point_higher_threshold_relative_to_ground
     * 
     * @return True if the height is valid, False otherwise.
    */
    bool checkInputPointHeight(const float& z) const;

  private:
    float min_x;    /**< The coordinates of the right-bottom corner of the right-bottom cell. */
    float min_y;
    float max_x;    /**< The coordinates of the left-top corner of the left-top cell. */
    float max_y;
    uint half_gridsize_x;     /**< The number of grid cells from the origo. It is N, if the size of the grid is 2*N+1. */
    uint half_gridsize_y;
  };
}


#endif /* INCLUDE_GRID_H_ */
