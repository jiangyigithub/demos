#include "blob.h"

namespace lipe {
  Blob::Blob(){
    ID_ = -99; // init with an invalid ID
    status_ = PLACEHOLDER;
  }
  
  void Blob::Init(){
    num_cells_ = 0;
    num_points_ = 0;
    
    //TODO: init cells_ and fill it with dummy values
    min_i_cell_idx_ = INT_MAX;
    min_j_cell_idx_ = INT_MAX;
    max_i_cell_idx_ = INT_MIN;
    max_j_cell_idx_ = INT_MIN;
    
    min_x_ = min_y_ = min_z_ = INT_MAX;
    max_x_ = max_y_ = max_z_ = INT_MIN;
    
    in_ego_circle_ = false;
    
    label_ = NOT_CLASSIFIED; // this will be updated in the blobdetector
    
    ID_ = -1; // this will be determined in the blobtracker
    status_ = IN_DETECTION;
    
    center_x_ = -1;
    center_y_ = -1;
    prev_center_x_ = -1;
    prev_center_y_ = -1;
    speed_x_ = 0;
    speed_y_ = 0;
    
    kalman_active_ = false;
    
    tracked_version_ = nullptr;
    
    is_active_ = false;
    last_seen_ = 0;
    continously_seen_ = 1;
    max_continously_seen_ = 1;
    visualized_ = false;
  }
   
  void Blob::InitKalman(){
    kalman_.Init(kalman_config_, prev_center_x_, prev_center_y_, center_x_, center_y_);
  }
  
  void Blob::PredictKalman(){
    kalman_.Predict(prev_center_x_, prev_center_y_, center_x_, center_y_, speed_x_, speed_y_);
  }
  

  void Blob::CorrectKalman(float new_meas_x, float new_meas_y){
    kalman_.Correct(new_meas_x, new_meas_y, prev_center_x_, prev_center_y_, center_x_, center_y_, speed_x_, speed_y_);   
  }
  
}

