/*
 * velodyne_projector_filter.h
 *
 *  Created on: Jan 31, 2014
 *      Author: wak1pal
 */

#ifndef VELODYNE_PROJECTOR_FILTER_H_
#define VELODYNE_PROJECTOR_FILTER_H_

#include <velodyne_msgs/VelodyneRawScan.h>
#include <lidar_msgs/ProjectedSpin.h>

#include <velodyne/velodyne_projector.h>

#include <message_filters/simple_filter.h>
#include <message_filters/connection.h>

namespace velodyne {

/*
 * VelodyneProjector can be used in a message filter
 *
 * velodyne::RawVelodyneProjectorFilter raw_velodyne_projector;
 * raw_velodyne_projector.connectInput(velo_raw_sub); // message_filters::Subscriber to raw topic
 * raw_velodyne_projector.init(velo_projection_cal, velo_intensity_cal))
 *
 *
 */
class RawVelodyneProjectorFilter : public message_filters::SimpleFilter<lidar_msgs::ProjectedSpin>
{
 public:
    RawVelodyneProjectorFilter();

    template<class F>
    RawVelodyneProjectorFilter(F& f) :
      has_init_(false),
      projector_(NULL)
    {
        connectInput(f);
    }

    ~RawVelodyneProjectorFilter();

    /**
     * \brief Connect this filter's input to another filter's output.  If this filter is
     * already connected, disconnects first.
     */
    template<class F>
    void connectInput(F& f)
    {
      message_connection_.disconnect();
      message_connection_ = f.registerCallback(&RawVelodyneProjectorFilter::update, this);
    }

    bool init(std::string const &cal_filename, std::string const &intensity_filename);

    // time compensation function
    void setProjectorJitterCompensationEnable(bool enable);
    void initProjectorOfflineJitterCompensation(const rosbag::Bag & bag);
    void setProjectorConstantTimeDelay(const double constant_time_delay);
private:
    void update(velodyne_msgs::VelodyneRawScan::ConstPtr const& raw_scan);

    message_filters::Connection message_connection_;

    bool has_init_;
    velodyne::VelodyneProjector *projector_;
};


} // velodyne



#endif /* VELODYNE_PROJECTOR_FILTER_H_ */
