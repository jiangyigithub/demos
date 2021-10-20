// MODIFIED VERSION OF pjfa/src/common/driving_common/include/driving_common/velodyne_message_filter.h

#ifndef VELODYNE_MESSAGE_FILTER_H_
#define VELODYNE_MESSAGE_FILTER_H_

#include <string>
#include <lidar_msgs/ProjectedSpin.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>

// NEW STUFF
#include <velodyne_msgs/VelodyneRawScan.h>
#include <velodyne/velodyne_projector.h>
#include <nodelet/nodelet.h>




namespace driving_common
{

/**
 * VelodyneClient makes sure clients get a complete spin from a velodyne sensor.
 *
 * Integrates any partial scans until complete scan is assembled
 *
 * To use (example):

  // Message filter for ROS subscription
  velodyne_sub_ = new message_filters::Subscriber<lidar_msgs::ProjectedSpin>();
  velodyne_sub_->subscribe(nh_, "velodyne/scan", 10);
  // Velodyne filter for integrating scan data
  velodyne_filter_ = new VelodyneScanMessageFilter(*velodyne_sub_);
  velodyne_filter_->registerCallback(boost::bind(&VelodyneLocalizer::velodyneCB, this, _1));

 *
 */
class VelodyneRawMessageFilter : public message_filters::SimpleFilter<lidar_msgs::ProjectedSpin>
{
 public:
    VelodyneRawMessageFilter();

    template<class F>
    VelodyneRawMessageFilter(F& f, velodyne::VelodyneProjector* projector) :
      cut_angle_(0.0),
      projector_(projector)
    {
        connectInput(f);
    }

    /**
     * \brief Connect this filter's input to another filter's output.  If this filter is already connected, disconnects first.
     */
    template<class F>
    void connectInput(F& f)
    {
      message_connection_.disconnect();
      message_connection_ = f.registerCallback(&VelodyneRawMessageFilter::update, this);
    }

    // Cut angle is in radians. Determines where in the scan we will "cut" our next slice
    void setCutAngle(double angle);

    // Reset the "state" of the message filter
    void clearFilter()
    {
      scans_.scan.clear();
    }

private:
	void update(velodyne_msgs::VelodyneRawScan::ConstPtr const &raw_scan);

    double cut_angle_;

    message_filters::Connection message_connection_;
    lidar_msgs::ProjectedSpin scans_;
    
    velodyne::VelodyneProjector* projector_;
};

} // namespace

#endif /* VELODYNE_MESSAGE_FILTER_H_ */
