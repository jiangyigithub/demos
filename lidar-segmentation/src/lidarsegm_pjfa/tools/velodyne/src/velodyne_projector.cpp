/*
 * velodyne_projector.cpp
 *
 *  Created on: Oct 29, 2012
 *      Author: wak1pal
 */

#include <velodyne/velodyne_projector.h>

#include <velodyne/velodyneConfig.h>

#include <ros/console.h>
#include <rosbag/view.h>
#include <limits>

using namespace velodyne;

static const double ECU_TIME_MAX = (std::pow(2, 32) - 1)/1000000.0;

VelodyneProjector::VelodyneProjector() :
    config_(NULL),
    enable_jitter_compensation_(false),
    is_compensation_mode_offline_(false),
    constant_time_delay_(0.0),
    overflow_count_(0)
{

}


bool VelodyneProjector::init(std::string const &cal_filename, std::string const &intensity_filename)
{
  if (config_)
  {
    ROS_ERROR("VelodyneProjector has already been initialized!");
    return false;
  }

  config_ = std::make_shared<velodyne::Config>();

  if (!config_->readCalibration(cal_filename))
  {
    ROS_ERROR("VelodyneProjector could not open calibration file \"%s\"", cal_filename.c_str());
    return false;
  }

  if (!config_->readIntensity(intensity_filename))
  {
    ROS_ERROR("VelodyneProjector could not open intensity calibration file \"%s\"", intensity_filename.c_str());
    return false;
  }

  return true;
}

std::shared_ptr<Config> VelodyneProjector::getConfig() const
{
  return config_;
}

void VelodyneProjector::projectVelodynePackets(velodyne_msgs::VelodyneRawScan const &raw_scan, lidar_msgs::ProjectedSpin &projected_spin)
{
  projected_spin.header = raw_scan.header;
  UDPPacket velo_udp_packet;
  for (size_t i = 0; i < raw_scan.packets.size(); ++i)
  {
    if (!parsePacket(raw_scan.packets[i], &velo_udp_packet))
      continue;

    velodynePacketToScans(&velo_udp_packet, projected_spin.scan);
  }

  // correct scan timestamp
  correctScanTimestamp(projected_spin.scan);
}

// Pushes scan into vector. Does not clear scan data
void VelodyneProjector::velodynePacketToScans(UDPPacket* const pkt, std::vector<lidar_msgs::Scan> &scans) const
{
  for (size_t i = 0; i < NUM_SCANS_IN_UDP_PACKET; i++)
  {
    lidar_msgs::Scan scan;

    scan.encoder  = pkt->scan[i].encoder;
    scan.block_id = pkt->scan[i].block;
    scan.stamp    = pkt->stamp;
    scan.ecu_time_sec = pkt->ecu_time_sec;

    projectMeasurement(pkt->scan[i], scan);

    scans.push_back(scan);
  }
}

// Pushes scan into vector. Does not clear scan data
void VelodyneProjector::velodyneMeasurementToScan(const Measurement& mmt, lidar_msgs::Scan& scan) const
{
  scan.encoder  = mmt.encoder;
  scan.block_id = mmt.block;

  projectMeasurement(mmt, scan);
}


bool VelodyneProjector::parsePacket(velodyne_msgs::VelodyneRawPacket const &raw_packet, UDPPacket* p)
{
  int ptr = 0;

  const uint8_t* const pkt = &raw_packet.data[0];
  p->stamp = raw_packet.stamp;

  for (size_t i = 0; i < NUM_SCANS_IN_UDP_PACKET; i++)
  {
    memcpy(&(p->scan[i].block), &(pkt[ptr]), sizeof(uint16_t));
    ptr += sizeof(uint16_t);
    switch (p->scan[i].block)
    {
      case 0xeeff: // upper block
        p->scan[i].block = 0;
        break;
      case 0xddff: // lower block
        p->scan[i].block = 1;
        break;
      default:
        ROS_ERROR("VelodyneProjector: Unknown block id (%u) in the velodyne packet, unable to parse.", p->scan[i].block);
        return false;
    }

    memcpy(&(p->scan[i].encoder), &(pkt[ptr]), sizeof(uint16_t));
    ptr += sizeof(uint16_t);

    for (size_t j = 0; j < NUM_VELO_BEAMS; j++)
    {
      memcpy(&(p->scan[i].range[j]), &(pkt[ptr]), sizeof(uint16_t));
      ptr += sizeof(uint16_t);
      p->scan[i].intensity[j] = pkt[ptr];
      ptr++;
    }
  }

  memcpy(p->status, &(pkt[ptr]), 6);
  ptr += 6;

  // decode ecu_time_sec from status
  uint32_t ecu_time_usec;
  memcpy(&(ecu_time_usec), p->status, sizeof(uint32_t));
  p->ecu_time_sec = static_cast<double>(ecu_time_usec)/1000000;

  return true;
}

/*
 * Projects a velodyne range measurement to points in the velodyne frame, using the velodyne's calibration
 */
void VelodyneProjector::projectMeasurement(const Measurement& msrm, lidar_msgs::Scan& scan) const
{
  double distance, distance1, cosVertAngle, sinVertAngle, cosRotAngle, sinRotAngle, hOffsetCorr, vOffsetCorr;
  double xyDistance;

  static const float quiet_nan = std::numeric_limits<float>::quiet_NaN();
  for (size_t j = 0; j < NUM_VELO_BEAMS; j++) {
    int32_t n = j + NUM_VELO_BEAMS * msrm.block;
    // use calibrated intensity!
    scan.laser[j].intensity = config_->intensity_map[config_->inv_beam_order[n]][msrm.intensity[j]];

    if ((msrm.range[j] == 0 || msrm.range[j] * VELODYNE_TICKS_TO_METER > 110 || std::isnan(config_->sin_rot_angle[msrm.encoder][n]))) {
      scan.point[j].x = quiet_nan;
      scan.point[j].y = quiet_nan;
      scan.point[j].z = quiet_nan;
      scan.laser[j].distance = USHRT_MAX;
    }
    else {
      distance1 = msrm.range[j] * VELODYNE_TICKS_TO_METER;
      distance = distance1 + config_->range_offset[n]; // scan distance in meters

      cosVertAngle = config_->cos_vert_angle[n];
      sinVertAngle = config_->sin_vert_angle[n];
      cosRotAngle = config_->cos_rot_angle[msrm.encoder][n];
      sinRotAngle = config_->sin_rot_angle[msrm.encoder][n];
      hOffsetCorr = config_->h_offset[n];
      vOffsetCorr = config_->v_offset[n];

      xyDistance = distance * cosVertAngle;

      scan.point[j].x = xyDistance * cosRotAngle - hOffsetCorr * sinRotAngle;
      scan.point[j].y = xyDistance * sinRotAngle + hOffsetCorr * cosRotAngle;
      scan.point[j].z = (xyDistance / cosVertAngle) * sinVertAngle + vOffsetCorr;

      scan.laser[j].distance = distance; // distance in meters
    }
  }
}

void VelodyneProjector::correctScanTimestamp(std::vector<lidar_msgs::Scan> &scans){
    for(auto& scan: scans){
        if(enable_jitter_compensation_)
        {
            if (is_compensation_mode_offline_) {
                // OFFLINE
                auto key = scan.stamp.toNSec();
                if (timestamp_map_jitter_compensation_.count(key)){
                    scan.stamp = ros::Time(timestamp_map_jitter_compensation_[key]);
                } else {
                    ROS_WARN("[VelodyneProjector]: could not find jitter compensated timestamp in offline mode!");
                }
            } else {
                // ONLINE
                static double ecu_time_sec_first = -1;
                static double ecu_time_sec_last = -1;

                // get ecu timestamp
                double ecu_time_sec = scan.ecu_time_sec;

                // correct for overflow
                if ((ecu_time_sec - ecu_time_sec_last) < 0)
                {
                    overflow_count_++;
                }
                ecu_time_sec += overflow_count_ * ECU_TIME_MAX;

                // set the first ecu time
                if(ecu_time_sec_first < 0){
                    ecu_time_sec_first = ecu_time_sec;
                }

                // set the last ecu time
                ecu_time_sec_last = ecu_time_sec;

                // estimate compensation parameters
                jitter_estimator_.onlineMode(ecu_time_sec, scan.stamp.toSec());

                // Only do jitter compensation, when records at least 1 minutes && 1000 samples
                if (((ecu_time_sec_last - ecu_time_sec_first) > 60) &&
                        (jitter_estimator_.getCounter() > 1000))
                {
                    scan.stamp = ros::Time(ecu_time_sec * jitter_estimator_.getClockDrift()  + jitter_estimator_.getClockOffset());
                }
            }
        }

        // correct for the constant time delay
        scan.stamp = scan.stamp - ros::Duration(constant_time_delay_);
    }
}

void VelodyneProjector::setJitterCompensationEnable(bool enable){
    enable_jitter_compensation_ = enable;
}

void VelodyneProjector::initOfflineJitterCompensation(const rosbag::Bag &bag){
    if (!enable_jitter_compensation_)
        return;

    is_compensation_mode_offline_ = true;
    timestamp_map_jitter_compensation_.clear();

    std::vector<double> timestamps_ecu, timestamps_ros;
    std::vector<uint64_t> timestamps_ros_nsec;
    std::vector<std::string> topic{"/driving/velodyne/raw_packets"};
    rosbag::View view(bag, rosbag::TopicQuery(topic));

    // accesing both ecu and ros timestamps
    for (const rosbag::MessageInstance &m : view) {
        velodyne_msgs::VelodyneRawScan::ConstPtr velodyne_ptr = m.instantiate<velodyne_msgs::VelodyneRawScan>();
        if (velodyne_ptr) {
            for (const auto& packet: velodyne_ptr->packets){
                // get ecu timestamp
                uint32_t ecu_time_usec;
                memcpy(&(ecu_time_usec), &(packet.data[1200]), sizeof(uint32_t));
                double ecu_time_sec = static_cast<double>(ecu_time_usec)/1000000;

                // put to the record
                timestamps_ros.push_back(packet.stamp.toSec());
                timestamps_ros_nsec.push_back(packet.stamp.toNSec());
                timestamps_ecu.push_back(ecu_time_sec);
            }
        }
    }

    // fix clock jump due to data overflow
    for (size_t i = 0; i < timestamps_ecu.size(); i++) {
        if (i > 1 && (timestamps_ecu[i]-timestamps_ecu[i-1]) < 0) {
            std::for_each(timestamps_ecu.begin()+i, timestamps_ecu.end(), [](double& value) { value += ECU_TIME_MAX;});
        }
    }

    // construct jitter compensate map
    jitter_estimator_.offlineMode(timestamps_ecu, timestamps_ros);
    auto drift = jitter_estimator_.getClockDrift();
    auto offset = jitter_estimator_.getClockOffset();
    for (size_t i = 0; i < timestamps_ecu.size(); i++) {
      timestamp_map_jitter_compensation_[timestamps_ros_nsec[i]] = timestamps_ecu[i] * drift + offset;
    }
}

void VelodyneProjector::setConstantTimeDelay(const double constant_time_delay){
    constant_time_delay_ = constant_time_delay;
}

const double VelodyneProjector::VELODYNE_TICKS_TO_METER = 0.002;

