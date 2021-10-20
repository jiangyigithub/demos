/**
 * The velodnye processing ROS node for "lipe". 
 * 
 * It deskewing and odometry compensating the incoming velodyne point cloud, than uses a VelodyneSegmenterAlgo for segmenteing, clustering and tracking.
 * 
 * # Input topics:
 * - Velodyne Messages
 * - Transformation
 * 
 * # Output topics:
 * - spin_pub_: The deskewed and odometry compensated velodyne point cloud. (Topic name is set by parameter "out_topic".)
 * - cloud_pub_ : The segmented and labeled point cloud. (Topic name is "<out_topic>/bp_freespace" where <out_topic> is a parameter.)
 * - free_space_pub_: The free space distances. (Topic name is set by parameter "freespace_topic".)
 * 
 * # Parameters:
 * - log_level: logging severity level. See logger.h for details. Default value is "error".
 * - input_topic: the name of topic where the velodyne messages are arriving. /driving/velodyne/projected_spin".
 * - out_topic: the name base of the point cloud output topics. Default value is "/driving/velodyne/projected_spin/sync".
 * - freespace_topic: the name of the output topic, where the free space distances are sent out. Default value is "/driving/velodyne/free_space".
 * 
 * @todo add license information here. In the meantime this is the property of Robert Bosch Kft, all rights reserved, use your own risk, etc.
 * 
 * @author Attila Börcs (CC-AD/ENG1-Bp) 
 * @author Károly Harsányi (CC-AD/ENG1-Bp) / (CC-AD/EAU-Bp)
 * @author Viktor Kövesd (CC-AD/ENG1-Bp) / (CC-AD/EAU-Bp)
 * @author Péter Lakatos (CC-AD/ENG1-Bp)
 */

#ifndef LIPE_VELODYNE_SEGMENTER_H
#define LIPE_VELODYNE_SEGMENTER_H

#include <nodelet/nodelet.h>

#include <lidarsegm_pjfa/FreeSpace.h>
#include <velodyne_segmenter_algo.h>
#include <logger.h>

#include <visualization_msgs/MarkerArray.h>
#include <lidarsegm_pjfa/ObjectList.h>

namespace lipe {

  class VelodyneSegmenter : public nodelet::Nodelet
  {
   public:

    /**
     * Default constructor.
     */
    VelodyneSegmenter();

    /**
     * Input callback function for incoming velodyne messages.
     * 
     */
    void spinCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&);
    
    
   private:

    /**
     * Default initializer. 
     */
    virtual void onInit();

    /**
     * Running through the points of the input cloud and copying the ones that are inside our grid (based on the grid config) 
     */
    void preprocessData(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud_msg);

    /**
     * Converting the calculated free space into the output format.
     */
    void convertFreeSpace(lidarsegm_pjfa::FreeSpace& free_space_msg) const;

    void FillObjectListMsg(lidarsegm_pjfa::ObjectList& object_list_msg);

    /**
     * Publish every outputs.
     */
    void publish(const std_msgs::Header header_to_publish);
    
    /**
     * Deletes the markers and publishes the empty array
     */
    void clearMarkers();

    /**
     * Read the command line arguments passed to the ros node.
     * It has the following command line arguments:
     * - log_level: The severity level of the logger. For more detail see the logger.h. (std::string)
     * - input_topic: The name of the input topic. (std::string)
     * - output_spin_topic: The name of the output topic publishing the deskewed velodyne point cloud. (std::string)
     * - output_cloud_topic: The name of the output topic publishing the segmented point cloud. (std::string)
     * - output_freespace_topic: The name of the output topic publishing the resulted freespace. (std::string)
     * - transform_listener_source_frame: The name of the source frame the transform listenere listens to for the deskewing transformation data. (std::string)
     * - output_spin_enabled: Switch for the output spin topic (bool)
     * - output_cloud_enabled: Switch for the output cloud topic (bool)
     * - output_freespace_enabled: Switch for the output freespace topic (bool)
     * For default values see the config.h
     */
    void readParams(const ros::NodeHandle& private_nh);
    
    // publisher for the segmented output cloud
    ros::Publisher cloud_pub_;
    
    // visualizer for markers
    ros::Publisher marker_pub_;
    visualization_msgs::MarkerArray marker_arr;
    
    ros::Publisher free_space_pub_;
    ros::Publisher object_list_pub_;

    Cloud cloud_raw;
    long int nr_valid_points;

    lipe::VelodyneSegmenterAlgo algo; /**< The algo class doing the segmenteing, clustering and tracking. */
    VisualizerConfig visualizer_config;
    lipe::Visualizer visualizer; // colors the pointcloud and the markers

    logging::Logger logger_;

    RosNodeConfig config;

    ros::Subscriber cloud_sub_;
  };


};
#endif  // LIPE_VELODYNE_SEGMENTER_H
