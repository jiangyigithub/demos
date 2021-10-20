// MODIFIED VERSION OF pjfa/src/common/driving_common/src/velodyne_message_filter.cpp

//#include <driving_common/velodyne_message_filter.h>
#include <velodyne/velodyne_raw_message_filter.h>

#include <angles/angles.h>

namespace {

static const double ENCODER_TICKS_PER_DEGREE = 100.0;

static inline double veloEncoderToRadian(uint16_t encoder, double cut_angle)
{
  double rv = angles::from_degrees(encoder / ENCODER_TICKS_PER_DEGREE); // Will be from [0,2pi] for [0,3600] input

  return angles::normalize_angle_positive(rv - cut_angle);
}


} // static

namespace driving_common
{

VelodyneRawMessageFilter::VelodyneRawMessageFilter(){
    cut_angle_ = 0.0;
}

void VelodyneRawMessageFilter::setCutAngle(double angle)
{
  cut_angle_ = angles::normalize_angle_positive(angle);

  scans_.scan.clear();
}

void VelodyneRawMessageFilter::update(velodyne_msgs::VelodyneRawScan::ConstPtr const &raw_scan)
{
	scans_.header = raw_scan->header;
	
	lidar_msgs::ProjectedSpinPtr packet(new lidar_msgs::ProjectedSpin);
    projector_->projectVelodynePackets(*raw_scan, *packet);


    // Last angle determine when we trigger for a new scan
    // If we see a decreasing angle -> trigger
    double last_angle = 0.0; // last_angle inits to 0 -> always add first packet
    if (!scans_.scan.empty())
    {
      last_angle = veloEncoderToRadian(scans_.scan.rbegin()->encoder, cut_angle_);
    }

    for (uint32_t i = 0; i < packet->scan.size(); i++)
    {
        if (!scans_.scan.empty() && (veloEncoderToRadian(packet->scan[i].encoder, cut_angle_) < last_angle))
        {
            // Set stamp with last stamp from scan
            scans_.header.stamp = scans_.scan.rbegin()->stamp;

            ROS_DEBUG_NAMED("velodyne_message_filter", "Built up scan with %zu spins, original frame %s, stamp %.3f. Current encoder: %d, last angle: %.2f. Cut angle: %.2f",
                            scans_.scan.size(), scans_.header.frame_id.c_str(), scans_.header.stamp.toSec(), packet->scan[i].encoder, last_angle, cut_angle_);

            // Copy data for publishing
            signalMessage(lidar_msgs::ProjectedSpinConstPtr(new lidar_msgs::ProjectedSpin(scans_)));

            scans_.scan.clear();
        }

        last_angle = veloEncoderToRadian(packet->scan[i].encoder, cut_angle_);
        scans_.scan.push_back(packet->scan[i]);
    }
}


} // namespace

