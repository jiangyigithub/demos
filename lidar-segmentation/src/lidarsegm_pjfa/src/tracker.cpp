#include "tracker.h"


lipe::Tracker::Tracker(const TrackerConfig& config){
  config_ = &config;
  frame_counter_ = 0;
  current_rolling_ID_ = 1; // 0 is ego in BP
  
  tracked_objects_ = new Blob[config_->max_tracked_objects]; 
};


lipe::Blob* lipe::Tracker::GetTrackedObjects(){
  return tracked_objects_;
}

int lipe::Tracker::GetMaxTrackedLen(){
  return config_->max_tracked_objects;
}

int lipe::Tracker::GetNumTracked(){
  return num_tracked_objects_;
}


void lipe::Tracker::UpdateBlob(Blob* old_version, Blob* new_version){

  old_version->num_cells_ = new_version->num_cells_;
  old_version->num_points_ = new_version->num_points_;

  old_version->min_i_cell_idx_ = new_version->min_i_cell_idx_;
  old_version->min_j_cell_idx_ = new_version->min_j_cell_idx_;
  old_version->max_i_cell_idx_ = new_version->max_i_cell_idx_;
  old_version->max_j_cell_idx_ = new_version->max_j_cell_idx_;

  old_version->min_x_ = new_version->min_x_;
  old_version->min_y_ = new_version->min_y_;
  old_version->min_z_ = new_version->min_z_;
  old_version->max_x_ = new_version->max_x_;
  old_version->max_y_ = new_version->max_y_;
  old_version->max_z_ = new_version->max_z_;

  old_version->in_ego_circle_ = new_version->in_ego_circle_;

  old_version->prev_center_x_ = old_version->center_x_;
  old_version->prev_center_y_ = old_version->center_y_;
  
  old_version->center_x_ = new_version->center_x_;
  old_version->center_y_ = new_version->center_y_;
  
  
  if(old_version->kalman_active_){
    old_version->CorrectKalman(new_version->center_x_, new_version->center_y_);    
  } else {
    old_version->InitKalman();
    old_version->kalman_active_=true;
  }
  
  ++old_version->continously_seen_;
  if(old_version->continously_seen_ > old_version->max_continously_seen_){
    old_version->max_continously_seen_ = old_version->continously_seen_;
  }
  
  // after this threshold the object will be visualized
  if(old_version->max_continously_seen_ > config_->visualization_threshold){
    old_version->visualized_ = true;
  }
  
  old_version->is_active_ = true;
  old_version->last_seen_ = 0;
  
  new_version->tracked_version_ = old_version;
};


void lipe::Tracker::InitTracking(Blob** current_objects, int current_objects_length){

  for( int i=0; i<current_objects_length; ++i ){
    
    if( i >= config_->max_tracked_objects){
     BOOST_LOG_TRIVIAL(error) << "[lipe] [Tracker] [InitTracking] Too many objects to track.";
    }
    tracked_objects_[i] = *current_objects[i];
    SetNewBlobParams(tracked_objects_[i]);   
    current_objects[i]->tracked_version_ = &tracked_objects_[i];    
    
  }
};


void lipe::Tracker::RemoveOutdatedBlobs(){

  for( int i = 0; i < config_->max_tracked_objects; ++i ){
    // if it was a valid blob before
    if(tracked_objects_[i].status_ != PLACEHOLDER ){
      // but it hasnt been seen in a while
      if(tracked_objects_[i].last_seen_ >= config_->forget_time){
        // kill it
        tracked_objects_[i].status_ = PLACEHOLDER;
      }
    }
  }
};


// for every valid/alive blob with an active kf: predict new center
void::lipe::Tracker::MakePredictedMovements(){

  for( int i = 0; i < config_->max_tracked_objects; ++i ){
    if( tracked_objects_[i].status_ != PLACEHOLDER ){
      // if the blobs kf has been initialized
      if( tracked_objects_[i].kalman_active_ ){ 
        // move center
        tracked_objects_[i].PredictKalman();
      }
    }    
  }
    
};

// TODO: try an implementation without dynamic memory allocation (fix, D, assignments, maybe make changes HungarianMethod)
int* lipe::Tracker::GetAssignments(Blob** current_objects, int current_objects_length)
{
  // count valid/alive objects
  int N = 0;
  for( int i = 0; i < config_->max_tracked_objects; ++i ){
    if( tracked_objects_[i].status_ != PLACEHOLDER ) ++N;
  }
  int M = current_objects_length;
  int* D = new int[N * M];
  
  
  // iterate on the tracked objects
  int current_tr_obj_idx = 0;
  for( int i = 0; i < config_->max_tracked_objects; ++i ){
    // if it is a valid tracked object
    if( tracked_objects_[i].status_ != PLACEHOLDER ){
      
      // iterate on the current objects
      for( int j = 0; j < M; ++j ){
        
        // in regards to "Magic number 100": the HungarianMethod lib expects ints from 0-100 as an input
        D[current_tr_obj_idx * M + j] = (int)(GetSimilarity(tracked_objects_[i], *current_objects[j]) * 100);
      }
      ++current_tr_obj_idx;
    }
  }
  
  // decalare a hungarian problem, and a method
  hungarian_problem_t HP;
  CHungarianMethod H;

  // build cost matrix (I think)
  int** cost = H.array_to_matrix(D, N, M);
  // init and solve the problem
  H.hungarian_init(&HP, cost, N, M, 0);
  H.hungarian_solve(&HP);
  
  int* assignments = new int[N];
  for( int i = 0; i < N; ++i ){
    assignments[i] = -1;
    for( int j = 0; j < M; ++j ){
      if(HP.assignment[i][j] == 1 && D[i*M+j] < 100){
        assignments[i] = j;
      }
    }
  }
  
  delete [] D;
  return assignments;
  
};


void lipe::Tracker::RefreshTrackedBlobs(Blob** current_objects, int current_objects_length, int* assignments)
{
  
  bool* is_matched = new bool[current_objects_length];
  std::fill_n( &is_matched[0], current_objects_length, false);
  
  // go trought the tracked blobs and check for assignments
  int current_idx = 0;
  for(int i=0; i < config_->max_tracked_objects; ++i){
    
    // if it is a valid object
    if( tracked_objects_[i].status_ != PLACEHOLDER ){
      
      //if it has no match
      if(assignments[current_idx] == -1){
        tracked_objects_[i].is_active_ = false;
        tracked_objects_[i].last_seen_++;
        tracked_objects_[i].continously_seen_ = 0;
        tracked_objects_[i].visualized_ = false;
      }
      // if it has a match we need to update the object accordingly
      else {
        is_matched[assignments[current_idx]] = true;
        UpdateBlob(&tracked_objects_[i], current_objects[assignments[current_idx]]);
      }
      ++current_idx;
    }
  }
  
  // we put the unmached blobs into the tracker as new objects
  int idx_in_tracking=0;
  for( int i = 0; i < current_objects_length; ++i ){
    // if the object has no match, we find a place for it in the tracking
    if(!is_matched[i]){
      Blob* tmp_blob = current_objects[i];
      
      for( int j = idx_in_tracking; j < config_->max_tracked_objects; ++j ){
        // if we find an empty spot in the array
        if( tracked_objects_[j].status_ == PLACEHOLDER ){
          
          tracked_objects_[j] = *current_objects[i];
          SetNewBlobParams(tracked_objects_[j]);
          current_objects[i]->tracked_version_ = &tracked_objects_[j];          
          
          idx_in_tracking = j+1;
          break;          
        }
      }
    }
  }
  
  delete [] assignments;
  delete [] is_matched;
}
  

void lipe::Tracker::AssignBlobIDs(){
  
  for( int i = 0; i < config_->max_tracked_objects; ++i){
    Blob* tmp_blob = &tracked_objects_[i];
    // if it is a valid object, but without an ID
    if(tmp_blob->ID_ == -1 && tmp_blob->continously_seen_ >= config_->validation_time && tmp_blob->is_active_){
      // assign id
      tmp_blob->ID_ = current_rolling_ID_;
      ++current_rolling_ID_;
      // start sending object msg
      tmp_blob->status_ = IN_TRACKING;
    }
  }
}


void lipe::Tracker::Run(Blob** current_objects, int current_objects_length){
  
  // count tracked objects
  int N = 0;
  for(int i=0; i<config_->max_tracked_objects; ++i){
    if( tracked_objects_[i].status_ != PLACEHOLDER ) ++N;
  }
  num_tracked_objects_ = N;
  
  // if this is the first frame
  if( frame_counter_ == 0 || N == 0 ){
    InitTracking(current_objects, current_objects_length);
  } 
  else{
    // else start the prediction and matching process
    RemoveOutdatedBlobs();
    
    MakePredictedMovements();
    
    int* assignments = GetAssignments(current_objects, current_objects_length);
    
    RefreshTrackedBlobs(current_objects, current_objects_length, assignments);
  }
  
  AssignBlobIDs();
  ++frame_counter_;    
};

// returns IOU. if there is a big overlap return 1 else return 0
float lipe::Tracker::GetIoU(const Blob& b1, const Blob& b2){
  
  // if an object was not seen on the previous frame, we dont use the IoU because we dont predict for the bbox
  if( b1.last_seen_ != 0 ) return 0;
  
  // get area
  float bbox1_area = ( b1.max_x_ - b1.min_x_ + 0.01 ) * ( b1.max_y_ - b1.min_y_ + 0.01 ); // 0.01 is there to avoid division by zero later
  float bbox2_area = ( b2.max_x_ - b2.min_x_ + 0.01 ) * ( b2.max_y_ - b2.min_y_ + 0.01 );

  // get intersection area
  float s_top = std::max( b1.min_x_, b2.min_x_ );
  float s_left = std::max( b1.min_y_, b2.min_y_ );
  float s_bottom = std::min( b1.max_x_, b2.max_x_ );
  float s_right = std::min( b1.max_y_, b2.max_y_ );
  float inter_area = std::max(0.0, s_bottom - s_top + 0.01) * std::max(0.0, s_right - s_left + 0.01);
  
  // return iou
  return inter_area / (bbox1_area + bbox2_area - inter_area);
};


// returns a value between 0 and 1. 1-> the objects are far away. 0-> the objects are close
float lipe::Tracker::GetCenterDistance(const Blob& b1, const Blob& b2){
  float x_diff = (b1.center_x_ - b2.center_x_);
  float y_diff = (b1.center_y_ - b2.center_y_);

  float dist = sqrt(pow((x_diff),2) + pow((y_diff),2));

  if(dist > config_->center_dist_limit){
    dist = config_->center_dist_limit;
  }

  return dist/config_->center_dist_limit;
};


// ~ 1: new object, bad match. ~ 0: old object good match
float lipe::Tracker::GetTimeValue(const Blob& b){
  
  float tv = b.continously_seen_;
  if( tv == 0 ) return 1.0; 
  
  if( tv > config_->time_value_ignore_threshold ) return 0.0;

  // now tv is between 0 and the limit
  tv = 0.1 * tv; // now between 0-1, where 1=seen for 5 frames or more
  tv = 1.0 - tv; // now 0 = seen for 5 frames
  return tv;
};


float lipe::Tracker::GetSimilarity(const Blob& b1, const Blob& b2){
  float iou = GetIoU(b1, b2);
  float center_dist = GetCenterDistance(b1,b2);
  
  float similarity = center_dist * config_->center_dist_weight + (1-iou) * config_->IoU_weight;
  
  if( similarity < config_->time_value_limit ){
    similarity *= GetTimeValue(b1);
  }
  return similarity;
}

void lipe::Tracker::SetNewBlobParams(Blob& b){
  b.is_active_ = true;
  b.last_seen_ = 0;
  b.continously_seen_ = 1;
  b.max_continously_seen_ = 1;    
}
