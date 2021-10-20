/*
 * velodyne_projector.h
 *
 *  Created on: Oct 29, 2012
 *      Author: wak1pal
 */

#ifndef VELODYNE_PROJECTOR_H_
#define VELODYNE_PROJECTOR_H_

#include <velodyne_msgs/VelodyneRawScan.h>
#include <lidar_msgs/ProjectedSpin.h>

#include <boost/utility.hpp>
#include <memory> // shared_ptr
#include <string>
#include <velodyne/velodyneConfig.h>

#include <time_jitter_compensation/jitter_estimator.hpp>
#include <rosbag/bag.h>

namespace velodyne
{
/*
 * VelodyneProjector projects raw packet data into XYZ and intensity data
 *
 *
 */
class VelodyneProjector : public boost::noncopyable
{

public:

  /*
    one packet is

    one scan =  NUM_VELO_BEAMS * 3 + 4 [enc+block+n*3] = 100 bytes
    packet = 32 * 100 + status (6 bytes) = 1206 bytes
  */
  static const double   VELODYNE_TICKS_TO_METER;// = 0.002;
  static const size_t   VELO_PACKET_SIZE        = 1206;
  static const uint32_t NUM_SCANS_IN_UDP_PACKET = 12;
  static const uint32_t NUM_VELO_BEAMS          = 32;

  struct Measurement
  {
    uint16_t      encoder;
    uint16_t      block;
    uint16_t      range[NUM_VELO_BEAMS];
    uint8_t       intensity[NUM_VELO_BEAMS];
  };

  struct UDPPacket
  {
    ros::Time     stamp;
    Measurement   scan[NUM_SCANS_IN_UDP_PACKET];
    uint8_t       status[6];
    double        ecu_time_sec;
  };


  VelodyneProjector();

  // Open calibration files. Intensity filename can be set empty if no intensity calibration is desired
  bool init(std::string const &cal_filename, std::string const &intensity_filename);

  // Project raw packets to XYZ/intensity data. Does NOT change message frame_id
  void projectVelodynePackets(velodyne_msgs::VelodyneRawScan const &raw_scan, lidar_msgs::ProjectedSpin &projected_spin);

  std::shared_ptr<Config> getConfig() const;

  static bool parsePacket(velodyne_msgs::VelodyneRawPacket const &raw_packet, UDPPacket* p);

  void velodyneMeasurementToScan(const Measurement& mmt, lidar_msgs::Scan& scan) const;


  // time compensation function
  void setJitterCompensationEnable(bool enable);
  void initOfflineJitterCompensation(const rosbag::Bag & bag);
  void setConstantTimeDelay(const double constant_time_delay);

private:

 void velodynePacketToScans(UDPPacket* const pkt, std::vector<lidar_msgs::Scan> &scans) const;

 void projectMeasurement(const Measurement& msrm, lidar_msgs::Scan& scan) const; // project velodyne scan into XYZ data

 std::shared_ptr<Config> config_;


 // compensation attributes
 void correctScanTimestamp(std::vector<lidar_msgs::Scan> &scans);
 time_jitter_compensation::JitterEstimator jitter_estimator_;
 bool enable_jitter_compensation_;
 bool is_compensation_mode_offline_;
 double constant_time_delay_;

 // mapping from raw ros time (nsec - uint64_t) to compensated time (sec - double)
 std::map<uint64_t, double> timestamp_map_jitter_compensation_;
 size_t overflow_count_; // for online compensation

};

}


#endif /* VELODYNE_PROJECTOR_H_ */
