/*
 * velodyne_projector_nodelet.cpp
 *
 *  Created on: Oct 29, 2012
 *      Author: wak1pal
 */


#include <ros/ros.h>
#include <vlr_exception/vlrException.h>
#include <velodyne/velodyne_projector.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace velodyne
{

/*
 * Nodelet wrapper around VelodyneProjector
 */
class VelodyneProjectorNodelet : public nodelet::Nodelet
{
public:
  VelodyneProjectorNodelet();
  ~VelodyneProjectorNodelet();

  void onInit();

private:
  void rawScanCB(velodyne_msgs::VelodyneRawScan::ConstPtr const &raw_scan);

  ros::NodeHandle nh_;
  ros::Publisher projected_spin_pub_;
  ros::Subscriber raw_scan_sub_;

  velodyne::VelodyneProjector projector_;
};



VelodyneProjectorNodelet::VelodyneProjectorNodelet()
{

}

VelodyneProjectorNodelet::~VelodyneProjectorNodelet()
{

}

void VelodyneProjectorNodelet::rawScanCB(velodyne_msgs::VelodyneRawScan::ConstPtr const &raw_scan)
{
  lidar_msgs::ProjectedSpinPtr projected_spin(new lidar_msgs::ProjectedSpin);
  projector_.projectVelodynePackets(*raw_scan, *projected_spin);

  projected_spin_pub_.publish(projected_spin);
}

void VelodyneProjectorNodelet::onInit()
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

  raw_scan_sub_       = nh_.subscribe("velodyne/raw_packets", 3, &VelodyneProjectorNodelet::rawScanCB, this);
  projected_spin_pub_ = nh_.advertise<lidar_msgs::ProjectedSpin>("velodyne/projected_spin", 5);
}

PLUGINLIB_DECLARE_CLASS(velodyne, VelodyneProjectorNodelet, velodyne::VelodyneProjectorNodelet, nodelet::Nodelet);

} // namespace

