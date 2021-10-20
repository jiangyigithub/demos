#include "grid.h"
#include "logger.h"


lipe::Grid::Grid( const GridConfig& config, const LidarConfig& lidar_config )
{    
  
  grid_config_ = &config;
  lidar_config_ = &lidar_config;
  
  // set half sizes:
  half_gridsize_x = (GetNrRows() - 1) / 2;
  half_gridsize_y = (GetNrColumns() - 1) / 2;

  // set the borders:
  const uint delta_i = GetHalfGridSizeX();
  const uint delta_j = GetHalfGridSizeY();
  const float center_x = static_cast< float >( delta_i ) * grid_config_->cell_size;
  const float center_y = static_cast< float >( delta_j ) * grid_config_->cell_size;
  max_x = center_x + grid_config_->cell_size * 0.5f;
  max_y = center_y + grid_config_->cell_size * 0.5f;
  min_x = -1.0f * max_x;    
  min_y = -1.0f * max_y;
  

  cells_ = new Cell* [ grid_config_->nr_rows ];
  cells_prop_ = new CellProp* [ grid_config_->nr_rows ];
  for( uint i = 0; i < grid_config_->nr_rows; ++i )
  {
    cells_[ i ] = new Cell[ grid_config_->nr_columns ];
    cells_prop_[ i ] = new CellProp[ grid_config_->nr_columns ];
    for( uint j = 0; j < grid_config_->nr_columns; ++j )
    {
      int min_points = std::max( static_cast<uint>( 1 ),
                                grid_config_->dist_for_extra_min_points - GetExtraPointsForCell( i, j ) );
      cells_prop_[ i ][ j ].min_num_of_points = min_points;
      cells_prop_[ i ][ j ].dist_from_ego = GetCellDistanceFromEgo( i, j );
      cells_prop_[ i ][ j ].x_dist_from_center = fabs((static_cast<float>(GetHalfGridSizeX()) - i) * grid_config_->cell_size);
      cells_prop_[ i ][ j ].y_dist_from_center = fabs((static_cast<float>(GetHalfGridSizeY()) - j) * grid_config_->cell_size);
            
      const float delta_i = static_cast< float >( i ) - static_cast< float >( GetHalfGridSizeX() );
      const float delta_j = static_cast< float >( j ) - static_cast< float >( GetHalfGridSizeY() );
      cells_prop_[ i ][ j ].angle_from_ego = ( std::atan2( delta_j, delta_i )  + M_PI ) * ( 180.0 / M_PI );      
      // @todo - If we use any nonstandard units, the name should reflect that!!! Rename it to angle_from_ego_degree or even better: make this member an unsigned int and rename it to free_space_index.
    }
  }

  Reset();
}

void lipe::Grid::Reset()
{
  Cell tmpcell;
  tmpcell.num_of_points = 0;
  tmpcell.min_x = tmpcell.min_y = tmpcell.min_z = INT_MAX;
  tmpcell.max_x = tmpcell.max_y = tmpcell.max_z = INT_MIN;
  memset( &tmpcell.num_of_point_in_layer, 0,
          LAYERS_IN_IBEO * sizeof( tmpcell.num_of_point_in_layer[ 0 ] ) );
  tmpcell.num_of_points = 0;
  tmpcell.has_blob = false;
  tmpcell.cont_blob = nullptr;
  tmpcell.label = SPARSE;
  for( uint i = 0; i < grid_config_->nr_rows; ++i )
  {
    /*copy the content of the tmpcell for every cell in the first row*/
    if(i == 0)
    {
      for( uint j = 0; j < grid_config_->nr_columns; ++j )
      {
        Cell* act_cell = &cells_[ i ][ j ];
        memcpy( act_cell, &tmpcell, sizeof( tmpcell ) );
      }
    }
    /*copy the content of the first row into the other rows*/
    else
    {
      memcpy( cells_[ i ], cells_[ 0 ],
              grid_config_->nr_columns * sizeof( cells_[ 0 ][ 0 ] ) );
    }
  }
}


float lipe::Grid::GetCellDistanceFromEgo( int i, int j)
{
  const float& cell_size =  grid_config_->cell_size;  
  const float xdistance = (static_cast<float>(GetHalfGridSizeX()) - i) * cell_size; 
  const float ydistance = (static_cast<float>(GetHalfGridSizeY()) - j) * cell_size;

  const float distance = std::hypot( xdistance, ydistance );
  return distance;
}


uint lipe::Grid::GetExtraPointsForCell( int x, int y )
{

  float distance = GetCellDistanceFromEgo( x, y );
  uint extra_point =  distance / grid_config_->dist_for_extra_min_points;

  return std::max( 0U, extra_point );
}


void lipe::Grid::AddPoint ( PCLMyPointType& point ) {

    // let's calculate which grid cell this point goes in
    Cell* act_cell = GetCellForPoint( point );
    if( nullptr == act_cell)
    {
      BOOST_LOG_TRIVIAL(error) << "[lipe] [Grid] [AddPoint] Out of grid point with coordinates ( " << std::setprecision(5) << point.x << ", " << point.y << " ).";
      return;      
    }

    // add the point and increase the pointnum in that grid cell
    ++( act_cell->num_of_points );
    /* if this is the first point or it changes the min/max,
     let's update the min/max values in the grid cell */

    if ( act_cell->min_x > point.x ) {
        act_cell->min_x = point.x;
    }
    if ( act_cell->max_x < point.x ) {
        act_cell->max_x = point.x;
    }

    if ( act_cell->min_y > point.y ) {
        act_cell->min_y = point.y;
    }
    if ( act_cell->max_y < point.y ) {
        act_cell->max_y = point.y;
    }

    if ( act_cell->min_z > point.z ) {
        act_cell->min_z = point.z;
    }
    if ( act_cell->max_z < point.z ) {
        act_cell->max_z = point.z;
    }

    if ( lidar_config_->type == LidarTypes::IBEO )
    {
      ++( act_cell->num_of_point_in_layer[ point.channel ] );
    }
}

void lipe::Grid::FillGrid(Cloud* cloud, long int cloud_size) {
  Reset();
  for ( uint i = 0; i < cloud_size; ++i ) {
      AddPoint( cloud->points[ i ] );
  }
}

bool lipe::Grid::checkIfInputPointInRange(const float& x, const float& y) const
{  
  if( x <= min_x) { return false; } 
  if( x >= max_x) { return false; } 
  if( y <= min_y) { return false; } 
  if( y >= max_y) { return false; } 

  const float range = lidar_config_->range;
  if ( x * x + y * y >= range * range ) { return false; } 

  return true;
}

bool lipe::Grid::checkInputPointHeight(const float& z) const
{ 
  const float min_z = -1.0f * lidar_config_->height + grid_config_->input_point_lower_threshold_relative_to_ground; 
  if( z < min_z) { return false; } 
  
  const float max_z = -1.0f * lidar_config_->height + grid_config_->input_point_higher_threshold_relative_to_ground;         
  if( z > max_z) { return false; } 

  return true;
}

uint lipe::Grid::GetHalfGridSizeX() const
{
  return half_gridsize_x;
}
 
uint lipe::Grid::GetHalfGridSizeY() const
{
  return half_gridsize_y;
}

lipe::Cell* lipe::Grid::GetCellForPoint( PCLMyPointType& point ) const
{ 
  const float& x = point.x;
  const float& y = point.y;
  
  // resacling x into [N+0.5, -N-0.5] wher N is the number of rows:
  const float rescaled_x = x / grid_config_->cell_size;   
  // rounding:
  const int relative_i = round(rescaled_x);  
  // change orientation and shift:
  const int i = GetHalfGridSizeX() - relative_i;

  const float rescaled_y = y / grid_config_->cell_size;
  const int relative_j = round(rescaled_y);
  const int j = GetHalfGridSizeY() - relative_j;

  if( i < 0 ) { return nullptr; }
  if( j < 0 ) { return nullptr; }
  if( i >= GetNrRows() ) { return nullptr; }
  if( j >= GetNrColumns() ) { return nullptr; }
  
  return &(cells_[i][j]);
}

