#ifndef IBEO_SEGMENTER_H
#define IBEO_SEGMENTER_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>

#include <ibeo_synchronizer/ibeo_transformer.h>
#include <ibeo_msgs/IbeoSpin.h>
#include <sensor_msgs/PointCloud2.h>

#include "segmenter.h"

namespace lipe {

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class IbeoSegmenter: public nodelet::Nodelet
{
 public:
  IbeoSegmenter();
  void sensorCallback(const ibeo_msgs::IbeoSpinConstPtr& msg);

 private:
  virtual void onInit();
  void ibeoSpinToPointCloud(const ibeo_msgs::IbeoSpin& ibeo_spin,
                            Cloud* ibeo_cloud);

 private:
  tf::TransformListener tf_listener_;
  ros::Subscriber sensor_sub_;
  ros::Publisher cloud_pub_;
  ros::Publisher free_space_pub_;
  segmenterIbeoParameter params;
  segmenterBasic grid_segm;
  Cloud cloud_raw;
};

}  // namespace ground_truth_labeler

#endif  // IBEO_SEGMENTER
