#include "../include/ibeo_segmenter.h"
#include "pluginlib/class_list_macros.h"
#include <lidarsegm_pjfa/FreeSpace.h>

namespace lipe {
#define MAX_NR_OF_IBEO_POINTS  200000UL
IbeoSegmenter::IbeoSegmenter()
{
}

void IbeoSegmenter::onInit()
{
  ros::NodeHandle multi_nh = getMTNodeHandle();
  ros::NodeHandle private_nh = getPrivateNodeHandle();

  std::string in_topic;
  std::string out_topic;
  std::string free_space_topic;

  private_nh.param<std::string>("input_topic", in_topic, "/driving/ibeo/spin");
  private_nh.param<std::string>("output_topic", out_topic, "/driving/ibeo/cloud");
  private_nh.param<std::string>("freespace_topic", free_space_topic, "/driving/ibeo/free_space");

  sensor_sub_ = multi_nh.subscribe(in_topic, 10, &IbeoSegmenter::sensorCallback, this);
  cloud_pub_ = multi_nh.advertise<PointCloudRGB>(out_topic, 10);
  free_space_pub_ = multi_nh.advertise<lidarsegm_pjfa::FreeSpace>(free_space_topic, 10);

  grid_segm.initGrid();
  grid_segm.multiLayerLidar = true;
  cloud_raw.height = 1;
  cloud_raw.width = MAX_NR_OF_IBEO_POINTS;
  cloud_raw.points.resize(cloud_raw.width);
}

void IbeoSegmenter::sensorCallback(const ibeo_msgs::IbeoSpinConstPtr& msg)
{
  long int nr_valid_points = 0;
  ibeo_msgs::IbeoSpin deskewed_spin;
  lidarsegm_pjfa::FreeSpace free_space_msg;
  // try to deskew the spin
  try {
    tf_listener_.waitForTransform(msg->header.frame_id, "/odom", msg->header.stamp,
          ros::Duration(0.1));
    common::deskewIbeoSpin("/odom", tf_listener_, msg->header.frame_id, *msg, deskewed_spin);
  }

  catch (tf::TransformException& ex) {
    NODELET_ERROR("%s", ex.what());
  }
  // convert to point cloud
  Cloud cloud_spin;
  ibeoSpinToPointCloud(deskewed_spin, &cloud_spin);

  BOOST_FOREACH(const PCLMyPointType& point, cloud_spin.points)
  {
    //filter out NAN from the point cloud
    if(!(
        std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)
       || point.x > 125 ||  point.y > 125 || point.z > 125
       || point.x < -125 || point.y < -125 || point.z < -125
        )
      )
    {
      cloud_raw.points[nr_valid_points].x = point.x;
      cloud_raw.points[nr_valid_points].y = point.y;
      cloud_raw.points[nr_valid_points].z = point.z;
      cloud_raw.points[nr_valid_points].channel = point.channel;

      nr_valid_points++;

      /*if(nr_valid_points > MAX_NR_OF_VELODYNE_POINTS)
      {
        throw(ROS_ERROR("%d points are in the frame and the maximum number is %s", %nr_valid_points %MAX_NR_OF_VELODYNE_POINTS));
      }*/
    }
  }
  //std::cout<<"Nr point "<<nr_valid_points<<std::endl;


     //cv::Mat heatmap = grid_segm.getHeatmap();
     //cv::imwrite("heatmaps/obj_mask.jpg", heatmap);

  //grid_segm.executeSegmentation(&cloud_raw, nr_valid_points);
  /*for(int i = 0; i < 360; i++)
  {
    free_space_msg.free_dist[i] = i;
  }*/
  grid_segm.executeDetection(&cloud_raw, nr_valid_points);
  cloud_raw.header = cloud_spin.header;
  cloud_pub_.publish(cloud_raw);
  free_space_pub_.publish(free_space_msg);
}

void IbeoSegmenter::ibeoSpinToPointCloud(const ibeo_msgs::IbeoSpin& ibeo_spin,
                                          Cloud* ibeo_cloud)
{
  assert(ibeo_cloud != NULL);

  // clear the output
  ibeo_cloud->clear();

  // iterate over all scan msgs
  BOOST_FOREACH(const ibeo_msgs::IbeoScan& scan, ibeo_spin.scans)
  {
    // iterate over all points within a scan
    BOOST_FOREACH(const ibeo_msgs::IbeoScanPoint& scan_point, scan.points)
    {
      PCLMyPointType cloud_point;
      cloud_point.x = scan_point.x;
      cloud_point.y = scan_point.y;
      cloud_point.z = scan_point.z;
      cloud_point.channel = scan_point.channel;
      ibeo_cloud->push_back(cloud_point);
    }
  }

  // set header information
  ibeo_cloud->header = pcl_conversions::toPCL(ibeo_spin.header);
}

}  // namespace ground_truth_labeler

// declare this class as a nodelet plugin
PLUGINLIB_EXPORT_CLASS(lipe::IbeoSegmenter, nodelet::Nodelet)
