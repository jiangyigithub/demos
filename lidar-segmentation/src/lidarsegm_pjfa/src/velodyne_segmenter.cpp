#include <velodyne_segmenter.h>
#include <lipe_interface.h>

#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>


namespace lipe {
  
  //constructor of publisher class
  VelodyneSegmenter::VelodyneSegmenter():
    visualizer(visualizer_config) {}

  //init interfaces and advertise
  void VelodyneSegmenter::onInit() {
    // get the handle
    ros::NodeHandle private_nh = getPrivateNodeHandle();
    // get the input parameters:
    readParams(private_nh);
        
    // set the log level:
    logger_.SetSeverityLevel(config.log_level);  
    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenter] [onInit] start";    
    BOOST_LOG_TRIVIAL(info) << "VelodyneSegmenter log level is " << logger_.GetSeverityLevelAsString();
    
    // subscribe to the input:
    cloud_sub_ = private_nh.subscribe<pcl::PointCloud<pcl::PointXYZI> > (config.input_topic, 0, &VelodyneSegmenter::spinCallback, this);

    // create cloud output:
    cloud_pub_ = private_nh.advertise<PointCloudRGB>(config.output_cloud_topic, 0);
    
    // marker outputs:
    marker_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(config.output_markers_topic, 100);
    
    // msg outputs
    free_space_pub_= private_nh.advertise<lidarsegm_pjfa::FreeSpace>(config.output_freespace_topic, 10);
    object_list_pub_ = private_nh.advertise<lidarsegm_pjfa::ObjectList>(config.output_object_list_topic, 10);
    
    // init algo members:
    algo.onInit();
    
    // init cloud raw
    cloud_raw.height = 1;
    cloud_raw.width = algo.getMaxNumberOfLidarPoints();
    cloud_raw.points.resize(cloud_raw.width);         
    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenter] [onInit] cloud_raw.points.size = " << cloud_raw.points.size();

    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenter] [onInit] finished";
  }

  void VelodyneSegmenter::readParams(const ros::NodeHandle& private_nh)
  {
    private_nh.param<std::string>("log_level", config.log_level, default_values::ros_node_config::log_level);
    private_nh.param<std::string>("input_topic", config.input_topic, default_values::ros_node_config::input_topic);
    private_nh.param<std::string>("output_cloud_topic", config.output_cloud_topic, default_values::ros_node_config::output_cloud_topic);
    private_nh.param<std::string>("output_freespace_topic", config.output_freespace_topic, default_values::ros_node_config::output_freespace_topic);
    private_nh.param<bool>("output_cloud_enabled", config.output_cloud_enabled, default_values::ros_node_config::output_cloud_enabled);
    private_nh.param<bool>("output_freespace_enabled", config.output_freespace_enabled, default_values::ros_node_config::output_freespace_enabled);
  }

  void VelodyneSegmenter::spinCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud_msg)
  {    
    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenter] [spinCallback] start";

    const ros::WallTime start_ = ros::WallTime::now();

    preprocessData(input_cloud_msg);
   
    // segmentation and detection
    algo.run(cloud_raw, nr_valid_points, marker_arr);
   
    // publish
    const std_msgs::Header header_to_publish = pcl_conversions::fromPCL(input_cloud_msg->header);
    publish(header_to_publish);

    // log runtime
    const ros::WallTime end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    BOOST_LOG_TRIVIAL(info) << "[lipe] [VelodyneSegmenter] [spinCallback] Exectution time: " << execution_time << " ms";

    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenter] [spinCallback] finished";
  }


  // PJFA
  void VelodyneSegmenter::preprocessData(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud_msg)
  {
      
    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenter] [preprocessData] start";
    BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenter] [preprocessData] " << "Input cloud size: " << input_cloud_msg->points.size();
    const ros::WallTime start_ = ros::WallTime::now();

    // init
    nr_valid_points = 0;
 
    // clear cloud
    cloud_raw.clear();
    cloud_raw.width =  algo.getMaxNumberOfLidarPoints();
    cloud_raw.height = 1;
    cloud_raw.points.resize(cloud_raw.width);

    // convert to a pcl 
    nr_valid_points = 0;
    for( int i = 0; i < input_cloud_msg->points.size(); ++i )
    {

      if(nr_valid_points >= algo.getMaxNumberOfLidarPoints()) 
      {
        BOOST_LOG_TRIVIAL(error) << "[lipe] [VelodyneSegmenter] [preprocessData] Number of the point in the frame is greater, then the maximum points defined for the function!";
        break;
      }     

      const pcl::PointXYZI& point = input_cloud_msg->points[i];

      if( false == algo.checkInputPoint(point.x, point.y, point.z) )
      {
        continue;
      }
      if( point.x == 0.0 && point.y == 0.0 && point.z == 0.0 )
      {
        continue;
      }
              
      cloud_raw.points[nr_valid_points].x = point.x;
      cloud_raw.points[nr_valid_points].y = point.y;
      cloud_raw.points[nr_valid_points].z = point.z;
      ++nr_valid_points;   
    }
        
    const ros::WallTime end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenter] [preprocessData] Exectution time: " << execution_time << " ms";
    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenter] [preprocessData] finished";    
  }

  
  void VelodyneSegmenter::publish(const std_msgs::Header header_to_publish) {
    float start_publish_time = clock();
    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenter] [publish] start";

    float time; 
    double execution_time;

    // publish cloud
    if( true == config.output_cloud_enabled )
    {  
      time = clock(); 
      cloud_raw.header = pcl_conversions::toPCL(header_to_publish);      
      // color the cloud
      visualizer.ColorCloud(&algo.getGrid(), &cloud_raw, nr_valid_points);
      cloud_pub_.publish(cloud_raw);
      execution_time = (((float) clock() - time) / CLOCKS_PER_SEC)*1000;
      BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenter] [publish]" << " Cloud coloring and pub. runtime: " << execution_time << " ms";  
    }    
    
    // publish markers
    if( true == config.output_markers_enabled )
    {
      time = clock(); 
      // fill the marker arrays
      float viz_markers_beg_time = clock();
      // clean the markers
      clearMarkers();
      visualizer.FillMarkerArray(algo.GetTrackedObjectList(), algo.GetMaxTrackedLen(), &marker_arr);
      marker_pub_.publish(marker_arr);   
      execution_time = (((float) clock() - time) / CLOCKS_PER_SEC)*1000;
      BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenter] [publish]" << " Marker updating and pub. runtime: " << execution_time << " ms";     
    }   
    
    // publish free space
    if( true == config.output_freespace_enabled )
    { 
      time = clock();
      // calculate and convert freespace:
      lidarsegm_pjfa::FreeSpace free_space_msg;
      convertFreeSpace(free_space_msg);

      free_space_msg.header = header_to_publish;
      free_space_pub_.publish(free_space_msg);
      
      execution_time = (((float) clock() - time) / CLOCKS_PER_SEC)*1000;
      BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenter] [publish]" << " Freespace conv. and pub. runtime: " << execution_time << " ms";     
      
    }
    
    // publish object list
    if( true == config.output_object_list_enabled )
    { 
      time = clock();
      // calculate object list
      lidarsegm_pjfa::ObjectList object_list_msg;
      
      FillObjectListMsg(object_list_msg);     
      object_list_msg.header = header_to_publish;
      object_list_pub_.publish(object_list_msg);
      
      execution_time = (((float) clock() - time) / CLOCKS_PER_SEC)*1000;
      BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenter] [publish]" << " ObjectList.msg fill and pub. runtime: " << execution_time << " ms";   
    }
    
    
    double publish_time = (((float) clock() - start_publish_time) / CLOCKS_PER_SEC)*1000;
    BOOST_LOG_TRIVIAL(debug) << "[lipe] [VelodyneSegmenter] [publish] " << "Full publish runtime: " << publish_time << " ms";  
    BOOST_LOG_TRIVIAL(trace) << "[lipe] [VelodyneSegmenter] [publish] finished.";

  }
  
  
  void VelodyneSegmenter::FillObjectListMsg(lidarsegm_pjfa::ObjectList& object_list_msg) 
  {
    Blob* tracked_objects = algo.GetTrackedObjectList();
    int max_tracked_len = algo.GetMaxTrackedLen();    
    
    // count objects to be published
    int object_list_len = 0;
    
    for( int i = 0; i < max_tracked_len; ++i ){
      if( tracked_objects[i].ID_ > 0 ) ++object_list_len;
    } 
        
    int size = std::min(static_cast<int>(object_list_msg.MAX_NR_OBJECT), static_cast<int>(object_list_len));
    int msg_idx = 0;
    for( int i = 0; i < max_tracked_len; ++i ){
      
      Blob* tmp_blob = &tracked_objects[i];
      if( tmp_blob->ID_ <= 0 ) continue;      
      
      if( msg_idx > size ){
        break;
      }
      if(tracked_objects[i].status_ == IN_TRACKING){
        
        object_list_msg.objects[msg_idx].x = tmp_blob->center_x_;
        object_list_msg.objects[msg_idx].y = tmp_blob->center_y_;
        object_list_msg.objects[msg_idx].v_x = 0.0;
        object_list_msg.objects[msg_idx].v_y = 0.0;
        object_list_msg.objects[msg_idx].orientation = 0.0;
        object_list_msg.objects[msg_idx].id = tmp_blob->ID_;

        object_list_msg.objects[msg_idx].last_seen = tmp_blob->last_seen_;
        object_list_msg.objects[msg_idx].continously_seen = tmp_blob->continously_seen_;
        object_list_msg.objects[msg_idx].max_continously_seen = tmp_blob->max_continously_seen_;
        object_list_msg.objects[msg_idx].visualized = (tmp_blob->is_active_ || tmp_blob->visualized_);
        msg_idx++;
      }
    }
  }
  
  void VelodyneSegmenter::convertFreeSpace(lidarsegm_pjfa::FreeSpace& free_space_msg) const 
  {
    const FreeSpace& free_space = algo.getFreeSpace();

    for( int i = 0; i < 360; ++i )
    {
      free_space_msg.free_dist_from_ground[i] = free_space.free_space_in_polar_[FROM_GROUND][i];
      free_space_msg.free_dist_from_objects[i] = free_space.free_space_in_polar_[FROM_OBJECTS][i];
      free_space_msg.free_dist_combined[i] = free_space.free_space_in_polar_[COMBINED][i];
    }
  }
  
  void VelodyneSegmenter::clearMarkers(){
    for(int i=0;i<marker_arr.markers.size();i++){
      marker_arr.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    marker_pub_.publish(marker_arr);
    marker_arr.markers.clear();
  }

} // namespace

// declare this class as a nodelet plugin
PLUGINLIB_EXPORT_CLASS(lipe::VelodyneSegmenter, nodelet::Nodelet)
