#include "visualizer.h"


pcl::RGB lipe::Visualizer::IDToColor(int ID){
  pcl::RGB color;

  // assign a bright color to the object based on its ID
  color.r = 100+((abs(0+ID)*29)%155); // red between 100 and 255
  color.b = 100+((abs(100-ID)*49)%155); // blue between 100 and 255
  color.g = 100+((abs(50-ID)*19)%155); // green between 100 and 255
  return color;

}

// color for objects that are not visualized yet (not validated by the tracker)
pcl::RGB lipe::Visualizer::GetUnvisualizedObjectColor(){
  pcl::RGB color;
  color.r = config_->unvisualized_object_color[0];
  color.b = config_->unvisualized_object_color[1];
  color.g = config_->unvisualized_object_color[2];
  return color;  
}

pcl::RGB lipe::Visualizer::GetWallColor(){
  pcl::RGB wall_color;
  wall_color.r = config_->wall_color[0];
  wall_color.g = config_->wall_color[1];
  wall_color.b = config_->wall_color[2];
  return wall_color; 
}

pcl::RGB lipe::Visualizer::GetGroundColor(){
  pcl::RGB ground_color;
  ground_color.r = config_->ground_color[0];
  ground_color.g = config_->ground_color[1]; 
  ground_color.b = config_->ground_color[2]; 
  return ground_color;
}

pcl::RGB lipe::Visualizer::GetNoiseColor(){
  pcl::RGB noise_color;
  noise_color.r = config_->noise_color[0];
  noise_color.g = config_->noise_color[1];
  noise_color.b = config_->noise_color[2];
  return noise_color;  
}

pcl::RGB lipe::Visualizer::GetObjectColor(){
  pcl::RGB object_color;
  object_color.r = config_->object_color[0];
  object_color.g = config_->object_color[1];
  object_color.b = config_->object_color[2];
  return object_color; 
}

pcl::RGB lipe::Visualizer::GetUnknownColor(){
  pcl::RGB un_color;
  un_color.r = config_->unknown_color[0];
  un_color.g = config_->unknown_color[1];
  un_color.b = config_->unknown_color[2];
  return un_color; 
}

// assign a random (bright) color. each value is between 100 and 255
pcl::RGB lipe::Visualizer::GetRandomColor(){
  pcl::RGB random_color;
  random_color.r = 100 + (rand() % static_cast<int>(255 - 99));
  random_color.g = 100 + (rand() % static_cast<int>(255 - 99));
  random_color.b = 100 + (rand() % static_cast<int>(255 - 99));
  return random_color; 
}


lipe::Visualizer::Visualizer(VisualizerConfig& config){
  config_ = &config;
}

void lipe::Visualizer::SetMarkerParams(visualization_msgs::Marker* marker){
  marker->header.frame_id = "vehicle_origin";
  marker->header.stamp = ros::Time();
  marker->action = visualization_msgs::Marker::ADD;
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;  
}

void lipe::Visualizer::FillMarkerArray(Blob* tracked_objects, int max_tracked_objects_len, visualization_msgs::MarkerArray* arr){ 
  // resize
  int object_list_len = 0;
  
  // show ghosts: show objects that were verified (has ID) and visible in the past, but not currently
  if(config_->show_ghosts){
    for( int i = 0; i < max_tracked_objects_len; ++i ){
      if( tracked_objects[i].status_ == IN_TRACKING )
        ++object_list_len;
    }
  } else {
    for( int i = 0; i < max_tracked_objects_len; ++i ){
      if( tracked_objects[i].status_ == IN_TRACKING )
        // these are the valid objects: lived long enough to be visualized, and is currently active
        if( tracked_objects[i].visualized_ && tracked_objects[i].is_active_ )
          ++object_list_len;
    }
  }
  
  if( config_->visualize_bounding_boxes )  arr->markers.resize(object_list_len*3);
  else arr->markers.resize(object_list_len*2);
  
  int marker_idx = 0;
  for(int i = 0; i < max_tracked_objects_len; i++){
    Blob* tmp_blob = &(tracked_objects[i]);
    
    if( tmp_blob->status_ == IN_TRACKING){
        
      float alpha = 1.0;
      
      // if it is not validated and active, we only show it as a ghost
      if ( !(tmp_blob->visualized_ && tmp_blob->is_active_ )){
        if( config_->show_ghosts ){ alpha = 0.25; }
        else { continue; }
      }
            
      // else
      // add center marker
      SetMarkerParams(&arr->markers[marker_idx]);
      arr->markers[marker_idx].ns = "center_marker_"+std::to_string(marker_idx);
      arr->markers[marker_idx].id = marker_idx;
      arr->markers[marker_idx].type = visualization_msgs::Marker::CUBE;
      arr->markers[marker_idx].pose.position.x = tmp_blob->center_x_+1;
      arr->markers[marker_idx].pose.position.y = tmp_blob->center_y_;
      arr->markers[marker_idx].pose.position.z = 2.0;
      arr->markers[marker_idx].scale.x = 0.5;
      arr->markers[marker_idx].scale.y = 0.5;
      arr->markers[marker_idx].scale.z = 0.0001;
      arr->markers[marker_idx].color.a = alpha;
      arr->markers[marker_idx].color.r = 1.0;
      arr->markers[marker_idx].color.g = 0;
      arr->markers[marker_idx].color.b = 0;
      ++marker_idx;
      
      // add text marker
      SetMarkerParams(&arr->markers[marker_idx]);
      arr->markers[marker_idx].ns = "text_marker"+std::to_string(marker_idx);
      arr->markers[marker_idx].id = marker_idx;
      arr->markers[marker_idx].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      arr->markers[marker_idx].pose.position.x = tmp_blob->center_x_+1;
      arr->markers[marker_idx].pose.position.y = tmp_blob->center_y_;
      arr->markers[marker_idx].pose.position.z = 3.0;
      arr->markers[marker_idx].scale.x = 0.5;
      arr->markers[marker_idx].scale.y = 0.5;
      arr->markers[marker_idx].scale.z = 0.5;
      arr->markers[marker_idx].color.a = alpha;
      arr->markers[marker_idx].color.r = 1.0;
      arr->markers[marker_idx].color.g = 1.0;
      arr->markers[marker_idx].color.b = 1.0;

      std::string marker_text = "ID: "+std::to_string(tmp_blob->ID_)+"\n";
      marker_text += "CX: "+std::to_string(tmp_blob->center_x_)+"  ";
      marker_text += "CY: "+std::to_string(tmp_blob->center_y_)+"\n";
      marker_text += "last_seen: "+std::to_string(tmp_blob->last_seen_)+"\n";
      marker_text += "cont_seen: "+std::to_string(tmp_blob->continously_seen_)+"\n";
      marker_text += "max_cs: "+std::to_string(tmp_blob->max_continously_seen_)+"\n";
      if((tmp_blob->visualized_ && tmp_blob->is_active_)) marker_text += "visualized: true\n";
      else marker_text += "visualized: false\n";
      arr->markers[marker_idx].text = marker_text;
      ++marker_idx;
      
      if( config_->visualize_bounding_boxes ){
        // add bbox marker
        SetMarkerParams(&arr->markers[marker_idx]);
        arr->markers[marker_idx].ns = "bbox_marker"+std::to_string(marker_idx);
        arr->markers[marker_idx].id = marker_idx;
        arr->markers[marker_idx].type = visualization_msgs::Marker::LINE_STRIP;
        arr->markers[marker_idx].pose.position.x = 0;
        arr->markers[marker_idx].pose.position.y = 0;
        arr->markers[marker_idx].pose.position.z = 0;
        arr->markers[marker_idx].scale.x = 0.05;
        arr->markers[marker_idx].scale.y = 0.05;
        arr->markers[marker_idx].scale.z = 0.0001;
        arr->markers[marker_idx].color.a = alpha;
        arr->markers[marker_idx].color.r = 0.0;
        arr->markers[marker_idx].color.g = 1.0;
        arr->markers[marker_idx].color.b = 0.0;
            
        // adding bbox
        geometry_msgs::Point bb_point = geometry_msgs::Point();
        bb_point.x = tmp_blob->min_x_+1;
        bb_point.y = tmp_blob->min_y_;
        arr->markers[marker_idx].points.push_back(bb_point);
        bb_point.x = tmp_blob->min_x_+1;
        bb_point.y = tmp_blob->max_y_;
        arr->markers[marker_idx].points.push_back(bb_point);
        bb_point.x = tmp_blob->max_x_+1;
        bb_point.y = tmp_blob->max_y_;
        arr->markers[marker_idx].points.push_back(bb_point);
        bb_point.x = tmp_blob->max_x_+1;
        bb_point.y = tmp_blob->min_y_;
        arr->markers[marker_idx].points.push_back(bb_point);
        bb_point.x = tmp_blob->min_x_+1;
        bb_point.y = tmp_blob->min_y_;
        arr->markers[marker_idx].points.push_back(bb_point);
        ++marker_idx;
      }
    }
  }
}


void lipe::Visualizer::ColorPointSegmentation(lipe::Cell* cell, PCLMyPointType* point) { 
  if(!cell || cell->label == SPARSE) point->rgb = GetNoiseColor().rgb;
  else if(cell->label == ROAD) point->rgb = GetGroundColor().rgb;
  else point->rgb = GetUnknownColor().rgb;
}

// TODO: some refactoring to make this function, and maybe the Get...Color type of functions more elegant
void lipe::Visualizer::ColorPointDetection(lipe::Cell* cell, PCLMyPointType* point) { // do i need the cell as well??
  if(!cell || cell->label == SPARSE){
    point->rgb = GetNoiseColor().rgb;
    return;
  }
  if( cell->label == ROAD ){
    point->rgb = GetGroundColor().rgb;
    return;
  }
  if( cell->label == NOT_CLASSIFIED){
    if(!cell->cont_blob){
      point->rgb = GetUnknownColor().rgb;
      return;
    }
    if(cell->cont_blob->label_ == WALL ){
      point->rgb = GetWallColor().rgb;
      return;
    }
    if(cell->cont_blob->label_ == OBJECT ){
      if(cell->cont_blob->tracked_version_->visualized_ && cell->cont_blob->tracked_version_->is_active_)
        point->rgb = IDToColor(cell->cont_blob->tracked_version_->ID_).rgb;
      else
        point->rgb = GetUnvisualizedObjectColor().rgb;
      return;
    }
    else{
      point->rgb = GetUnknownColor().rgb;
      return;
    }
  }
}

void lipe::Visualizer::ColorCloud(const lipe::Grid* grid, lipe::Cloud* cloud, long int cloud_size){
  
  for( int i=0; i<cloud_size; i++ ){
    Cell* act_cell = grid->GetCellForPoint(cloud->points[i]);
    if (config_->visualize_tracking)
      ColorPointDetection(act_cell, &cloud->points[i]);
    else
      ColorPointSegmentation(act_cell, &cloud->points[i]);
  }
}