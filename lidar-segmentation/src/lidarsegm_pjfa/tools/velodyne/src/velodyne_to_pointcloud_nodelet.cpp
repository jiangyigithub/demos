/*
 * velodyne_to_pointcloud_nodelet.cpp
 *
 *  Created on: 2019.01.18
 *      Author: hsk2bp
 */

// FROM VELODYNE_PROJECTOR_NODELET
#include <ros/ros.h>
#include <vlr_exception/vlrException.h>
#include <velodyne/velodyne_projector.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// NEW STUFF
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <driving_common/velodyne_transformer.h>
#include <velodyne/velodyne_raw_message_filter.h>
#include <tf/transform_listener.h>


namespace velodyne
{

/* 
 * Define PointCloud type
 */ 
//typedef pcl::PointCloud<pcl::PointXYZI> Cloud;


/*
 * Nodelet wrapper around VelodyneProjector
 */
class VelodyneToPointCloudNodelet : public nodelet::Nodelet
{
public:
  VelodyneToPointCloudNodelet();
  ~VelodyneToPointCloudNodelet();

  void onInit();

private:

  ros::NodeHandle nh_;
  message_filters::Subscriber<velodyne_msgs::VelodyneRawScan> raw_scan_sub_;
  velodyne::VelodyneProjector projector_;
  
  // NEW STUFF
  pcl::PointCloud<pcl::PointXYZI> cloud_raw;
  long int nr_valid_points;
  long int maxNumberOfPoints = 200000UL;
  ros::Publisher cloud_pub_;
  
  tf::TransformListener tf_listener_;
  std::string transform_listener_source_frame_ = "/vehicle_origin";
  
  driving_common::VelodyneRawMessageFilter* velodyne_filter_;   
  void preprocessRawVelodyneData(const lidar_msgs::ProjectedSpinConstPtr& msg, lidar_msgs::ProjectedSpin& spin_deskewed);
  void OnSpin(const lidar_msgs::ProjectedSpinConstPtr& msg);  
  
};



VelodyneToPointCloudNodelet::VelodyneToPointCloudNodelet()
{

}

VelodyneToPointCloudNodelet::~VelodyneToPointCloudNodelet()
{

}


void VelodyneToPointCloudNodelet::onInit()
{
  nh_ = ros::NodeHandle(getNodeHandle(), "/driving");

  std::string cal_filename, intensity_filename;
  bool calibrate_intensities;
  if (!nh_.getParam("velodyne/cal_file",  cal_filename))
  {
    VLR_THROW_EXCEPTION("Unable to get parameter \"velodyne/cal_file\"");
    return;
  }
  if (!nh_.getParam("velodyne/int_file",              intensity_filename))
  {
    VLR_THROW_EXCEPTION("Unable to get parameter \"velodyne/int_file\"");
    return;
  }
  if (!nh_.getParam("velodyne/calibrate_intensities", calibrate_intensities))
  {
    VLR_THROW_EXCEPTION("Unable to get parameter \"velodyne/calibrate_intensities\"");
    return;
  }

  if (!calibrate_intensities) intensity_filename = "";

  if (!projector_.init(cal_filename, intensity_filename))
  {
    VLR_THROW_EXCEPTION("Unable to initialize VelodyneProjector");
    return;
  }

  // NEW STUFF
  raw_scan_sub_.subscribe(nh_, "velodyne/raw_packets", 3);
  velodyne_filter_ = new driving_common::VelodyneRawMessageFilter(raw_scan_sub_, &projector_); 
  velodyne_filter_->setCutAngle(M_PI);  
  velodyne_filter_->registerCallback(boost::bind(&VelodyneToPointCloudNodelet::OnSpin, this, _1));
  cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("velodyne/processed_pointcloud", 0);
  
  
}

PLUGINLIB_DECLARE_CLASS(velodyne, VelodyneToPointCloudNodelet, velodyne::VelodyneToPointCloudNodelet, nodelet::Nodelet);


// NEW STUFF (from LiPe) 
// projected spin ---> PointCloud2
void VelodyneToPointCloudNodelet::preprocessRawVelodyneData(const lidar_msgs::ProjectedSpinConstPtr& msg, lidar_msgs::ProjectedSpin& spin_deskewed)
{
	// init
	nr_valid_points = 0;

	// deskew and transform spin
	try {
	  tf::StampedTransform odom_to_veh_origin;
	  tf_listener_.waitForTransform(msg->header.frame_id, transform_listener_source_frame_,  msg->header.stamp, ros::Duration(0.1));

	  tf_listener_.lookupTransform(msg->header.frame_id, transform_listener_source_frame_, msg->header.stamp, odom_to_veh_origin);

	  driving_common::deskewVelodyneScan(transform_listener_source_frame_, tf_listener_, odom_to_veh_origin, *msg, spin_deskewed);
	}
	catch (tf::TransformException ex) {
	  return;        
	}

	const ros::WallTime start_ = ros::WallTime::now();

	// clear cloud
	cloud_raw.clear();
	cloud_raw.width = maxNumberOfPoints;
	cloud_raw.height = 1;
	cloud_raw.points.resize(cloud_raw.width);

	// convert to a pcl 
	nr_valid_points = 0;
	for( int i = 0; i < spin_deskewed.scan.size(); ++i )
	{
	  for( int j = 0; j < spin_deskewed.scan[i].point.size(); ++j )
	  {
		if(nr_valid_points >= maxNumberOfPoints) 
		{
		  break;
		}     

		const geometry_msgs::Point32& point = spin_deskewed.scan[i].point[j];
	
		cloud_raw.points[nr_valid_points].x = point.x;
		cloud_raw.points[nr_valid_points].y = point.y;
		cloud_raw.points[nr_valid_points].z = point.z;
		cloud_raw.points[nr_valid_points].intensity = spin_deskewed.scan[i].laser[j].intensity;
		++nr_valid_points;   
	  }
	}	
}

// calls preprocess when its signaled by the velodyne_filter (see line 125-130)
void VelodyneToPointCloudNodelet::OnSpin(const lidar_msgs::ProjectedSpinConstPtr& msg)
{
  
  lidar_msgs::ProjectedSpin spin_deskewed;
  preprocessRawVelodyneData(msg, spin_deskewed);
  
  cloud_raw.header = pcl_conversions::toPCL(msg->header); 
  cloud_pub_.publish(cloud_raw);
}


} // namespace

