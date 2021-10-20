/*
 * lidar_config.h
 *
 *  Created on: Nov 19, 2018
 *      Author: kvv2bp
 */

#ifndef LIPE_CONFIG_H_
#define LIPE_CONFIG_H_

#include "GlobalHeader.h"

namespace lipe
{

  #define LAYERS_IN_IBEO 4

  enum LidarTypes{
    VELODYNE = 0,
    IBEO     = 1,
    MAX_LIDAR_TYPES
  };

  /**
   * The default values of the config values for the ros node.
   * 
   * For description, see the VelodyneSegmenter::readParams() function.
   */
  namespace default_values{
    namespace ros_node_config{
      const std::string log_level = "error";
      const std::string input_topic = "/driving/velodyne/processed_pointcloud";
      const std::string output_freespace_topic = "/driving/velodyne/free_space";
      const std::string output_object_list_topic = "/driving/velodyne/object_list";      
      const std::string output_cloud_topic = input_topic + "/segmented"; 
      const std::string output_markers_topic = "visualization_marker_array";
      const bool output_cloud_enabled = true;
      const bool output_markers_enabled = true;
      const bool output_freespace_enabled = true;
      const bool output_object_list_enabled = true;
    }
  }


  /**
   * Config values for the ros node.
   * 
   * For description, see the VelodyneSegmenter::readParams() function.
   */
  struct RosNodeConfig{
  
  public:
    RosNodeConfig(
      const std::string& log_level_ = default_values::ros_node_config::log_level,
      const std::string& input_topic_ = default_values::ros_node_config::input_topic,
      const std::string& output_freespace_topic_ = default_values::ros_node_config::output_freespace_topic,
      const std::string& output_object_list_topic_ = default_values::ros_node_config::output_object_list_topic,
      const std::string& output_cloud_topic_ = default_values::ros_node_config::output_cloud_topic,
      const std::string& output_markers_topic_ = default_values::ros_node_config::output_markers_topic,
      const bool output_cloud_enabled_ = default_values::ros_node_config::output_cloud_enabled,
      const bool output_markers_enabled_ = default_values::ros_node_config::output_markers_enabled,
      const bool output_freespace_enabled_ = default_values::ros_node_config::output_freespace_enabled,
      const bool output_object_list_enabled_ = default_values::ros_node_config::output_object_list_enabled
      
    ):
    log_level(log_level_),
    input_topic(input_topic_),
    output_cloud_topic(output_cloud_topic_),
    output_markers_topic(output_markers_topic_),
    output_freespace_topic(output_freespace_topic_),
    output_object_list_topic(output_object_list_topic_),
    output_cloud_enabled(output_cloud_enabled_),
    output_markers_enabled(output_markers_enabled_),
    output_freespace_enabled(output_freespace_enabled_),
    output_object_list_enabled(output_object_list_enabled_)

    {};

  public:
    std::string log_level;
    std::string input_topic;
    std::string output_cloud_topic;
    std::string output_markers_topic;
    std::string output_freespace_topic;
    std::string output_object_list_topic;
    bool output_cloud_enabled;
    bool output_markers_enabled;
    bool output_freespace_enabled;
    bool output_object_list_enabled;
  };

  namespace default_values{
    namespace lidar_config{
      const uint velodyne_layers = 0;
      const float velodyne_range = 150.0f;
      const float velodyne_height = 1.75f; //2.0f;
      const enum LidarTypes velodyne_lidar_type = LidarTypes::VELODYNE;
      const long int max_number_of_lidar_points = 200000UL;
    }
  }

  struct LidarConfig{
  
  public:

    LidarConfig(
      uint layers = default_values::lidar_config::velodyne_layers, 
      float r = default_values::lidar_config::velodyne_range, 
      float h = default_values::lidar_config::velodyne_height, 
      enum LidarTypes t = default_values::lidar_config::velodyne_lidar_type,
      long int max_number_of_lidar_points_ = default_values::lidar_config::max_number_of_lidar_points
      ):
      nr_of_layers(layers), 
      range(r), 
      height(h), 
      type(t),
      max_number_of_lidar_points(max_number_of_lidar_points_)
    {
    };
    
    const uint nr_of_layers;
    const float range; /* in meters */
    const float height; /* Lidar relative height from ground in meter */
    const enum LidarTypes type;
    const long int max_number_of_lidar_points;
  };


  namespace default_values{
    namespace grid_config{
      const float cell_size = 0.5;
      const float grid_range = default_values::lidar_config::velodyne_range * 0.5f;   // sic! This is smaller than the velodyne range, since from those outer area we don't get enough lidar points anyway.
      const uint nr_rows = 2 * static_cast<uint>(grid_range / cell_size) + 1;
      const uint nr_columns = 2 * static_cast<uint>(grid_range / cell_size) + 1;
      const uint dist_for_extra_min_points = 0;
      const float input_point_lower_threshold_relative_to_ground = - 0.2f;   
      const float input_point_higher_threshold_relative_to_ground = 100.0f;  
    }
  }

  struct GridConfig
   {
    public:
       GridConfig(
         uint rows = default_values::grid_config::nr_rows, 
         uint columns = default_values::grid_config::nr_columns, 
         uint dist = default_values::grid_config::dist_for_extra_min_points, 
         float c_size = default_values::grid_config::cell_size,
         float input_point_lower_threshold_relative_to_ground_ = default_values::grid_config::input_point_lower_threshold_relative_to_ground,
         float input_point_higher_threshold_relative_to_ground_ = default_values::grid_config::input_point_higher_threshold_relative_to_ground
         ):
         nr_rows(rows), 
         nr_columns(columns), 
         dist_for_extra_min_points(dist), 
         cell_size(c_size),
         input_point_lower_threshold_relative_to_ground(input_point_lower_threshold_relative_to_ground_),
         input_point_higher_threshold_relative_to_ground(input_point_higher_threshold_relative_to_ground_)
         {};

       const uint nr_rows;
       const uint nr_columns;
       const uint dist_for_extra_min_points; /*with distance the number of the minimum number of points to classify a cell as an object can be decreased. This a parameter for after how many meters can the minimum number can be decreased [point / meter] */
       const float cell_size; /*in meter*/
       const float input_point_lower_threshold_relative_to_ground;
       const float input_point_higher_threshold_relative_to_ground;
   };

  namespace default_values{
    namespace segmenter_config{  
      const float ignore_dist_x = 3.0;
      const float ignore_dist_y = 1.2;  
      const uint min_point_num = 3;
      const float ground_height = -1.25f;//default_values::lidar_config::velodyne_height;
      const float ground_height_difference = 0.25f; //1.0f;
    }
  }

  struct SegmenterConfig
  {
    const float ignore_dist_x;
    const float ignore_dist_y;
    const uint min_point_num;
    const float ground_height;
    const float ground_height_difference;
    
    SegmenterConfig(
      float ignore_d_x = default_values::segmenter_config::ignore_dist_x,
      float ignore_d_y = default_values::segmenter_config::ignore_dist_y,
      uint min_points = default_values::segmenter_config::min_point_num,
      float gr_h = default_values::segmenter_config::ground_height, 
      float gr_h_diff = default_values::segmenter_config::ground_height_difference
      ) :
      ignore_dist_x(ignore_d_x), ignore_dist_y(ignore_d_y), min_point_num(min_points), ground_height(gr_h), ground_height_difference(gr_h_diff) {};
  };

  namespace default_values{
    namespace kalman_filter_config{
      const float process_noise_cov_scalar=1e-4;
      const float measurement_noise_cov_scalar=1e-1;
      const float error_cov_post_scalar=1e-1;
    }
  }
  
  struct KalmanFilterConfig{
    const float process_noise_cov_scalar;
    const float measurement_noise_cov_scalar;
    const float error_cov_post_scalar;
    
    KalmanFilterConfig(
      float proc_noise_scal = default_values::kalman_filter_config::process_noise_cov_scalar, 
      float meas_noise_scal = default_values::kalman_filter_config::measurement_noise_cov_scalar, 
      float error_scal = default_values::kalman_filter_config::error_cov_post_scalar
      ): 
      process_noise_cov_scalar(proc_noise_scal), measurement_noise_cov_scalar(meas_noise_scal), error_cov_post_scalar(error_scal) {};
  };
  
  
  namespace default_values{
    namespace detector_config{
      const uint max_blob_list_len = 500;
      const uint max_object_list_len = 80;
      const uint max_other_list_len = 200;
      const uint cells_per_blob = 500;
      const bool use_tesla_demo_params = true;
      const uint min_points_per_blob = 30;
      
      // params for pointnet usecase
      const float ego_circle_radius = 10.0;
      const float overall_height_threshold = 0.8;
      const float overall_height_lower_threshold = -0.8;
      const float overall_area_threshold = 30.25;
      const float inside_ego_circle_side_length_threshold = 8.0;
      const float inside_ego_circle_area_threshold = 30.25;
      const float outside_ego_circle_wall_height = 0.6;
      const float outside_ego_circle_wall_height_difference = 2.2;
      
      // params from tesla demo
      const float lidar_height = 1.75;
      const float floor_threshold = 0.4;
      const float roof_threshold = 0.5;
      const float height_lower_bound = 0.5;
      const float height_upper_bound = 2.3;
      const float x_lower_bound = 0.6;
      const float x_upper_bound = 7.5;
      const float y_lower_bound = 1.5;
      const float y_upper_bound = 7.5;
      const float area_lower_bound = 1.2;
      const float area_upper_bound = 25.0;
      const float shape_lower_bound = 0.18;
      const float shape_upper_bound = 5.5;     
    }
  }

  struct DetectorConfig{
    const uint max_blob_list_len;
    const uint max_object_list_len;
    const uint max_other_list_len;
    const uint cells_per_blob;
    const bool use_tesla_demo_params;
    
    // params for pointnet usecase
    const float ego_circle_radius;
    const uint min_points_per_blob;
    const float overall_height_threshold;
    const float overall_height_lower_threshold;
    const float overall_area_threshold;
    const float inside_ego_circle_side_length_threshold;
    const float inside_ego_circle_area_threshold;
    const float outside_ego_circle_wall_height;
    const float outside_ego_circle_wall_height_difference;
    
    
    // params from tesla demo
    const float lidar_height;
    const float floor_threshold;
    const float roof_threshold;
    const float height_lower_bound;
    const float height_upper_bound;
    const float x_lower_bound;
    const float x_upper_bound;
    const float y_lower_bound;
    const float y_upper_bound;
    const float area_lower_bound;
    const float area_upper_bound;
    const float shape_lower_bound;
    const float shape_upper_bound;   
    
    DetectorConfig(
      uint max_b_list_len = default_values::detector_config::max_blob_list_len, 
      uint max_obj_list_len = default_values::detector_config::max_object_list_len, 
      uint max_oth_list_len = default_values::detector_config::max_other_list_len,
      uint cells_p_blob = default_values::detector_config::cells_per_blob, 
      bool use_tesla_demo_params_ = default_values::detector_config::use_tesla_demo_params,
      
      float ego_c_rad = default_values::detector_config::ego_circle_radius,
      uint min_p_p_blob = default_values::detector_config::min_points_per_blob, 
      float overall_h_th = default_values::detector_config::overall_height_threshold,
      float overall_h_low_th = default_values::detector_config::overall_height_lower_threshold,
      float overall_a_th = default_values::detector_config::overall_area_threshold, 
      float inside_ego_c_s_th = default_values::detector_config::inside_ego_circle_side_length_threshold, 
      float inside_ego_c_a_th = default_values::detector_config::inside_ego_circle_area_threshold,
      float outside_ego_c_w_h = default_values::detector_config::outside_ego_circle_wall_height, 
      float outside_ego_c_w_h_d = default_values::detector_config::outside_ego_circle_wall_height_difference,
      
      const float lidar_height_ = default_values::detector_config::lidar_height,
      const float floor_threshold_ = default_values::detector_config::floor_threshold,
      const float roof_threshold_ = default_values::detector_config::roof_threshold,
      const float height_lower_bound_ = default_values::detector_config::height_lower_bound,
      const float height_upper_bound_ = default_values::detector_config::height_upper_bound,
      const float x_lower_bound_ = default_values::detector_config::x_lower_bound,
      const float x_upper_bound_ = default_values::detector_config::x_upper_bound,
      const float y_lower_bound_ = default_values::detector_config::y_lower_bound,
      const float y_upper_bound_ = default_values::detector_config::y_upper_bound,
      const float area_lower_bound_ = default_values::detector_config::area_lower_bound,
      const float area_upper_bound_ = default_values::detector_config::area_upper_bound,
      const float shape_lower_bound_ = default_values::detector_config::shape_lower_bound,
      const float shape_upper_bound_ = default_values::detector_config::shape_upper_bound   

      ) :
      max_blob_list_len(max_b_list_len), max_object_list_len(max_obj_list_len), max_other_list_len(max_oth_list_len), cells_per_blob(cells_p_blob), 
      use_tesla_demo_params(use_tesla_demo_params_),

      ego_circle_radius(ego_c_rad), min_points_per_blob(min_p_p_blob), overall_height_threshold(overall_h_th),
      overall_height_lower_threshold(overall_h_low_th),
      overall_area_threshold(overall_a_th), inside_ego_circle_side_length_threshold(inside_ego_c_s_th), inside_ego_circle_area_threshold(inside_ego_c_a_th),
      outside_ego_circle_wall_height(outside_ego_c_w_h), outside_ego_circle_wall_height_difference(outside_ego_c_w_h_d),
      
      lidar_height(lidar_height_), floor_threshold(floor_threshold_), roof_threshold(roof_threshold_),
      height_lower_bound(height_lower_bound_), height_upper_bound(height_upper_bound_), x_lower_bound(x_lower_bound_),
      x_upper_bound(x_upper_bound_), y_lower_bound(y_lower_bound_), y_upper_bound(y_upper_bound_), 
      area_lower_bound(area_lower_bound_), area_upper_bound(area_upper_bound_), 
      shape_lower_bound(shape_lower_bound_), shape_upper_bound(shape_upper_bound_)
      {};       
  };
  
  
  namespace default_values{
    namespace tracker_config{
      const uint max_tracked_objects = 100;
      const uint forget_time = 10;
      const uint validation_time = 2;
      const uint visualization_threshold = 5;
      const float center_dist_limit = 3.0;
      const float IoU_weight = 0.9;
      const float center_dist_weight = 0.1;
      const float time_value_limit = 0.9;
      const float time_value_ignore_threshold = 10.0;
      
    }
  }
  
  struct TrackerConfig{
    const uint max_tracked_objects;
    const uint forget_time;
    const uint validation_time;
    const uint visualization_threshold;
    const float center_dist_limit;
    const float IoU_weight;
    const float center_dist_weight;
    const float time_value_limit;
    const float time_value_ignore_threshold;
    
    TrackerConfig(
      uint max_t_o = default_values::tracker_config::max_tracked_objects,
      uint forget_t = default_values::tracker_config::forget_time,
      uint validation_t = default_values::tracker_config::validation_time,
      uint visualization_threshold_ = default_values::tracker_config::visualization_threshold,
      float center_d_l = default_values::tracker_config::center_dist_limit,
      float IoU_w = default_values::tracker_config::IoU_weight,
      float center_d_w = default_values::tracker_config::center_dist_weight,
      float time_v_l = default_values::tracker_config::time_value_limit,
      float time_v_i_t = default_values::tracker_config::time_value_ignore_threshold    
    ) :
    max_tracked_objects(max_t_o), forget_time(forget_t), validation_time(validation_t),
    visualization_threshold(visualization_threshold_),
    center_dist_limit(center_d_l), IoU_weight(IoU_w), center_dist_weight(center_d_w), 
    time_value_limit(time_v_l), time_value_ignore_threshold(time_v_i_t) {};    
  };
  
  
  namespace default_values{
    namespace visualizer_config{
      const bool visualize_tracking = true;
      const bool show_ghosts = true;
      const bool visualize_bounding_boxes = false;
      
      // colors for the cloud coloring
      const int wall_color[3] = {153, 15, 15};
      const int ground_color[3] = {0, 100, 50}; 
      const int noise_color[3] = {0, 100, 100}; 
      const int object_color[3] = {200, 200, 25}; // this is only used if visualize_tracking == false
      const int unknown_color[3] = {70, 70, 70};
      const int unvisualized_object_color[3] = {100, 100, 100};
    }
  }
  
  struct VisualizerConfig{
    const bool show_ghosts;
    const bool visualize_tracking;
    const bool visualize_bounding_boxes;
    const int* wall_color;
    const int* ground_color; 
    const int* noise_color; 
    const int* object_color; // this is only used if visualize_tracking == false
    const int* unknown_color;
    const int* unvisualized_object_color;

    VisualizerConfig(
      const bool s_ghosts = default_values::visualizer_config::show_ghosts,
      const bool visualize_tracking_ = default_values::visualizer_config::visualize_tracking,
      const bool visualize_bounding_boxes_ = default_values::visualizer_config::visualize_bounding_boxes,
      const int* wall_color_ = default_values::visualizer_config::wall_color,
      const int* ground_color_ = default_values::visualizer_config::ground_color,
      const int* noise_color_ = default_values::visualizer_config::noise_color,
      const int* object_color_ = default_values::visualizer_config::object_color,
      const int* unknown_color_ = default_values::visualizer_config::unknown_color,
      const int* unvisualized_object_color_ = default_values::visualizer_config::unvisualized_object_color
      ): 
      show_ghosts(s_ghosts),
      visualize_tracking(visualize_tracking_),
      visualize_bounding_boxes(visualize_bounding_boxes_),
      wall_color(wall_color_),
      ground_color(ground_color_),
      noise_color(noise_color_),
      object_color(object_color_),
      unknown_color(unknown_color_),
      unvisualized_object_color(unvisualized_object_color_)
      {};
  }; 
  
}


#endif /* INCLUDE_CONFIG_H_ */
