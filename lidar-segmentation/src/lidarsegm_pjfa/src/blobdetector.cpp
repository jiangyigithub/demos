#include "blobdetector.h"


lipe::BlobDetector::BlobDetector(const DetectorConfig& config, const KalmanFilterConfig& kalman_config, Grid& grid){
  
  config_ = &config;
  grid_ = &grid;                    // todo - grid_ could be a reference, and passed to the detector once, in the constructor's initialization list.
  kalman_config_ = &kalman_config;
  
  visited_ = new int* [grid_->GetNrRows()];
  for( uint i = 0; i < grid_->GetNrRows(); ++i )
  {
    visited_[i] = new int[grid_->GetNrColumns()];
  }
  

  for(uint i = 0; i < grid_->GetNrRows(); ++i){
    for(uint j = 0; j < grid_->GetNrColumns(); ++j){
      visited_[i][j] = 0;
    }
  }
     
  blob_list_ = new Blob[config_->max_blob_list_len]; 
  object_list_ = new Blob*[config_->max_object_list_len];
  other_list_ = new Blob*[config_->max_other_list_len];
  
  // set kalman_config_ for every blob
  for( int i = 0; i < config_->max_blob_list_len; ++i ){
    blob_list_[i].kalman_config_ = &kalman_config;
  }  
}
  
void lipe::BlobDetector::ResetBlobList(){

  for(uint i = 0; i < grid_->GetNrRows(); ++i){
    for(uint j = 0; j < grid_->GetNrColumns(); ++j){
      visited_[i][j] = 0;
    }
  }
  
  for( int i = 0; i < config_->max_blob_list_len; ++i ){
    blob_list_[i].Init();
  }
  
  std::fill_n( &object_list_[0], config_->max_object_list_len, nullptr );
  std::fill_n( &other_list_[0], config_->max_other_list_len, nullptr );
    
  current_blob_list_len_ = 0;
  current_object_list_len_ = 0;
  current_other_list_len_ = 0;
}

void lipe::BlobDetector::FillBlobList(){  
  ResetBlobList();

  // grid dimensions
  uint grid_size_x = grid_->GetNrRows();
  uint grid_size_y = grid_->GetNrColumns();
  
  bool early_stop = false;
  
  for( int i = 0; i < grid_size_x; ++i ){
    
    if(early_stop){
      break;
    }
    
    for( int j = 0; j < grid_size_y; ++j ){

      // found a new island
      if( grid_->cells_[i][j].label == NOT_CLASSIFIED && !visited_[i][j] ){
               
        //if( current_blob_idx >= config_->max_blob_list_len){
        if( current_blob_list_len_ >= config_->max_blob_list_len ){
          BOOST_LOG_TRIVIAL(error) << "[lipe] [BlobDetector] [FillBlobList] the blob list is full. early stop in blob detection.";
          early_stop = true;
          break;
        } 

        // queue with x and y coordinates
        int x_queue[config_->cells_per_blob];
        int y_queue[config_->cells_per_blob];
        int queue_front = 0;
        int queue_len = 1;
        
        // reset queue
        memset( x_queue, -1, sizeof(x_queue) );
        memset( y_queue, -1, sizeof(y_queue) );
        
        // get the tmp blob
        Blob* current_blob = &blob_list_[current_blob_list_len_];
        ++current_blob_list_len_;
        
        visited_[i][j] = 1;
        // push [i,j] to queue
        x_queue[queue_front] = i;
        y_queue[queue_front] = j;
        
        while( queue_front != queue_len){

          int x = x_queue[queue_front];
          int y = y_queue[queue_front];
          
          // get tmp cell and cell_prop
          Cell* current_cell = &grid_->cells_[x][y];
          CellProp* current_cell_prop = &grid_->cells_prop_[x][y];
          

          if( current_blob->num_cells_ >= config_->cells_per_blob ){
            BOOST_LOG_TRIVIAL(error) << "[lipe] [BlobDetector] [FillBlobList] too man cells in a blob. Breaking up current blob and starting a new one.";
            break;
          }
          
          ++queue_front;
                    
          // update Blob config_s based on the Cell
          ++current_blob->num_cells_;
          current_blob->num_points_ += current_cell->num_of_points;
          current_cell->cont_blob = current_blob;
          current_cell->has_blob = true;
                    
          if( x < current_blob->min_i_cell_idx_ ) current_blob->min_i_cell_idx_ = x;
          if( x > current_blob->max_i_cell_idx_ ) current_blob->max_i_cell_idx_ = x;
          if( y < current_blob->min_j_cell_idx_ ) current_blob->min_j_cell_idx_ = y;
          if( y > current_blob->max_j_cell_idx_ ) current_blob->max_j_cell_idx_ = y;
          
          if( current_blob->min_x_ > current_cell->min_x ) current_blob->min_x_ = current_cell->min_x;
          if( current_blob->min_y_ > current_cell->min_y ) current_blob->min_y_ = current_cell->min_y;
          if( current_blob->min_z_ > current_cell->min_z ) current_blob->min_z_ = current_cell->min_z;//lowest point
          if( current_blob->max_x_ < current_cell->max_x ) current_blob->max_x_ = current_cell->max_x;
          if( current_blob->max_y_ < current_cell->max_y ) current_blob->max_y_ = current_cell->max_y;
          if( current_blob->max_z_ < current_cell->max_z ) current_blob->max_z_ = current_cell->max_z;//highest point

          if( !current_blob->in_ego_circle_ && current_cell_prop->dist_from_ego < config_->ego_circle_radius ) current_blob->in_ego_circle_ = true;
          
          // check adjacent cells
          // look NORTH
          if( x > 0 && grid_->cells_[x-1][y].label == NOT_CLASSIFIED && !visited_[x-1][y] && queue_len< config_->cells_per_blob ){
            visited_[x-1][y] = true;
            // add to the end of the queue
            x_queue[queue_len] = x-1;
            y_queue[queue_len] = y;
            ++queue_len;
          }
          // look NORTH-EAST
          if( x > 0 && y < grid_size_y-1 && grid_->cells_[x-1][y+1].label == NOT_CLASSIFIED && !visited_[x-1][y+1] && queue_len < config_->cells_per_blob ){
            visited_[x-1][y+1] = true;
            // add to the end of the queue
            x_queue[queue_len] = x-1;
            y_queue[queue_len] = y+1;
            ++queue_len;
          }
          // look EAST
          if( y < grid_size_y-1 && grid_->cells_[x][y+1].label == NOT_CLASSIFIED && !visited_[x][y+1] && queue_len < config_->cells_per_blob ){
            visited_[x][y+1] = true;
            // add to the end of the queue
            x_queue[queue_len] = x;
            y_queue[queue_len] = y+1;
            ++queue_len;
          }
          // look SOUTH-EAST
          if( x < grid_size_x-1 && y < grid_size_y-1 && grid_->cells_[x+1][y+1].label == NOT_CLASSIFIED && !visited_[x+1][y+1] && queue_len < config_->cells_per_blob ){
            visited_[x+1][y+1] = true;
            // add to the end of the queue
            x_queue[queue_len] = x+1;
            y_queue[queue_len] = y+1;
            ++queue_len;
          }
          // look SOUTH
          if( x < grid_size_x-1 && grid_->cells_[x+1][y].label == NOT_CLASSIFIED && !visited_[x+1][y] && queue_len < config_->cells_per_blob ){
            visited_[x+1][y] = true;
            // add to the end of the queue
            x_queue[queue_len] = x+1;
            y_queue[queue_len] = y;
            ++queue_len;
          }
          // look SOUTH-WEST
          if( x < grid_size_x-1 && y > 0 && grid_->cells_[x+1][y-1].label == NOT_CLASSIFIED && !visited_[x+1][y-1] && queue_len < config_->cells_per_blob ){
            visited_[x+1][y-1] = true;
            // add to the end of the queue
            x_queue[queue_len] = x+1;
            y_queue[queue_len] = y-1;
            ++queue_len;
          }
          // look WEST
          if( y > 0 && grid_->cells_[x][y-1].label == NOT_CLASSIFIED && !visited_[x][y-1] && queue_len < config_->cells_per_blob ){
            visited_[x][y-1] = true;
            // add to the end of the queue
            x_queue[queue_len] = x;
            y_queue[queue_len] = y-1;
            ++queue_len;
          }
          // look NORTH-WEST
          if( x > 0 && y > 0 && grid_->cells_[x-1][y-1].label == NOT_CLASSIFIED && !visited_[x-1][y-1] && queue_len < config_->cells_per_blob ){
            visited_[x-1][y-1] = true;
            // add to the end of the queue
            x_queue[queue_len] = x-1;
            y_queue[queue_len] = y-1;
            ++queue_len;
          }
        }
      }
    }
  }
  
  // at the end the blobdetector calculates all the blob centers
  for( int i = 0; i < current_blob_list_len_; ++i ){
    blob_list_[i].center_x_ = (blob_list_[i].max_x_ + blob_list_[i].min_x_)/2.0;
    blob_list_[i].center_y_ = (blob_list_[i].max_y_ + blob_list_[i].min_y_)/2.0;
  }
}

void lipe::BlobDetector::SetToObject(Blob* blob){
  // check if there is more space in the object_list_ and add the blob
  //assert(current_object_list_len_ < config_->max_object_list_len && "OBJECT LIST IS FULL");
  if(current_object_list_len_ >= config_->max_object_list_len){
    BOOST_LOG_TRIVIAL(error) << "[lipe] [BlobDetector] [SetToObject] the object list is full. omitting all further objects.";
    return;
  } 
  
  object_list_[current_object_list_len_] = blob;
  object_list_[current_object_list_len_]->label_ = OBJECT;
  ++ current_object_list_len_;
}

void lipe::BlobDetector::SetToOther(Blob* blob){
  //assert(current_other_list_len_ < config_->max_other_list_len && "OTHER LIST IS FULL");
  if(current_other_list_len_ >= config_->max_other_list_len){
    BOOST_LOG_TRIVIAL(error) << "[lipe] [BlobDetector] [SetToOther] the other list is full. omitting all further 'other' blobs.";
    return;
  }
  
  other_list_[current_other_list_len_] = blob;
  ++current_other_list_len_;
  blob->label_ = WALL;
}

void lipe::BlobDetector::ClassifiyBlobs(){
  for( int i = 0; i < current_blob_list_len_; ++i ){
    
    Blob* current_blob = &blob_list_[i];
    
    if( !config_->use_tesla_demo_params ){
      // if the object doesnt have enough points = NOT_CLASSIFIED
      if(current_blob->num_points_ < config_->min_points_per_blob){
        current_blob->label_ == NOT_CLASSIFIED;
      }
      
    
      // if it is very tall, very low, or has a large area, it is a WALL
      else if(current_blob->max_z_ > config_->overall_height_threshold || current_blob->max_z_ < config_->overall_height_lower_threshold ||
              (current_blob->max_x_-current_blob->min_x_)*(current_blob->max_y_-current_blob->min_y_) > config_->overall_area_threshold){
        SetToOther(current_blob);
      }
      
      // examine blobs in the ego circle
      else if(current_blob->in_ego_circle_){
        // large side lenght or large area -> WALL
        if(( current_blob->max_x_ - current_blob->min_x_ ) > config_->inside_ego_circle_side_length_threshold || 
           ( current_blob->max_y_ - current_blob->min_y_ ) > config_->inside_ego_circle_side_length_threshold ||
           ( current_blob->max_x_ - current_blob->min_x_ ) * ( current_blob->max_y_ - current_blob->min_y_ ) > config_->inside_ego_circle_area_threshold ){
          SetToOther(current_blob);
        }
        else {
          SetToObject(current_blob);
        }
      }
      // if its not in the ego circle and its realtively tall, it is an object
      else if(current_blob->max_z_-current_blob->min_z_ > config_->outside_ego_circle_wall_height_difference || 
              current_blob->max_z_ > config_->outside_ego_circle_wall_height){
        SetToOther(current_blob);
      }
      else{
        SetToObject(current_blob);
      }
    } 
    else {
      // if the object doesnt have enough points = NOT_CLASSIFIED
      if(current_blob->num_points_ < config_->min_points_per_blob){
        current_blob->label_ == NOT_CLASSIFIED;
      }
      
      // classification from tesla demo
      else if( fabs(current_blob->min_z_ - (-1 * config_->lidar_height)) < config_->floor_threshold && // has a similar floor to the tesla
               fabs(current_blob->max_z_ - 0.0) < config_->roof_threshold && // has a similar roof to the tesla
               current_blob->max_z_ - current_blob->min_z_ > config_->height_lower_bound && // not too short
               current_blob->max_z_ - current_blob->min_z_ < config_->height_upper_bound && // not too tall
               current_blob->max_x_ - current_blob->min_x_ > config_->x_lower_bound && 
               current_blob->max_x_ - current_blob->min_x_ < config_->x_upper_bound && 
               current_blob->max_y_ - current_blob->min_y_ > config_->y_lower_bound && 
               current_blob->max_y_ - current_blob->min_y_ < config_->y_upper_bound && 
               (current_blob->max_y_ - current_blob->min_y_) * (current_blob->max_x_ - current_blob->min_x_) > config_->area_lower_bound && // not too small area
               (current_blob->max_y_ - current_blob->min_y_) * (current_blob->max_x_ - current_blob->min_x_) < config_->area_upper_bound && // not too large area
               (current_blob->max_y_ - current_blob->min_y_)/(current_blob->max_x_ - current_blob->min_x_) > config_->shape_lower_bound && // weird shape
               (current_blob->max_y_ - current_blob->min_y_)/(current_blob->max_x_ - current_blob->min_x_) < config_->shape_upper_bound // weird shape
              )
      {
        SetToObject(current_blob);
      } else {
        SetToOther(current_blob);
      }
    }
  }
}