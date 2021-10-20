/*
 * velodyne_projector_filter.cpp
 *
 *  Created on: Jan 31, 2014
 *      Author: wak1pal
 */

#include <velodyne/velodyne_projector_filter.h>
#include <vlr_exception/vlrException.h>

velodyne::RawVelodyneProjectorFilter::RawVelodyneProjectorFilter() :
  has_init_(false),
  projector_(NULL)
{

}


velodyne::RawVelodyneProjectorFilter::~RawVelodyneProjectorFilter()
{
  delete projector_;
}


bool velodyne::RawVelodyneProjectorFilter::init(std::string const &cal_filename, std::string const &intensity_filename)
{
  projector_ = new velodyne::VelodyneProjector();
  has_init_ = projector_->init(cal_filename, intensity_filename);
  return has_init_;
}

void velodyne::RawVelodyneProjectorFilter::update(velodyne_msgs::VelodyneRawScan::ConstPtr const& raw_scan)
{
  if (!has_init_) {
    VLR_THROW_EXCEPTION("RawVelodyneProjectorFilter has not initialized!");
  }

  lidar_msgs::ProjectedSpinPtr projected_spin(new lidar_msgs::ProjectedSpin);
  projector_->projectVelodynePackets(*raw_scan, *projected_spin);

  ROS_ASSERT_MSG(projected_spin->header.stamp == raw_scan->header.stamp,
      "Raw scan and projected spin stamps don't match. Raw spin: %.6f, projected: %.6f", raw_scan->header.stamp.toSec(),
      projected_spin->header.stamp.toSec());

  signalMessage(projected_spin);
}

// time compensation function
void velodyne::RawVelodyneProjectorFilter::setProjectorJitterCompensationEnable(bool enable){
    if (!has_init_) {
      VLR_THROW_EXCEPTION("RawVelodyneProjectorFilter has not initialized!");
    }

    projector_->setJitterCompensationEnable(enable);
}

void velodyne::RawVelodyneProjectorFilter::initProjectorOfflineJitterCompensation(const rosbag::Bag & bag){
    if (!has_init_) {
      VLR_THROW_EXCEPTION("RawVelodyneProjectorFilter has not initialized!");
    }

    projector_->initOfflineJitterCompensation(bag);
}

void velodyne::RawVelodyneProjectorFilter::setProjectorConstantTimeDelay(const double constant_time_delay){
    if (!has_init_) {
      VLR_THROW_EXCEPTION("RawVelodyneProjectorFilter has not initialized!");
    }

    projector_->setConstantTimeDelay(constant_time_delay);
}


