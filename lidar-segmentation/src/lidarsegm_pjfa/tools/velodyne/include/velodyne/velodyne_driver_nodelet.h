/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/


#ifndef VELODYNE_VELODYNE_H_
#define VELODYNE_VELODYNE_H_

#include <stdint.h>
#include <ros/ros.h>

#include <velodyne/velodyneConfig.h>
#include <velodyne_msgs/VelodyneRawScan.h>
#include <nodelet/nodelet.h>

#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

namespace velodyne {

#define VELODYNE_TICKS_TO_METER  0.002
#define VELO_NUM_LASERS          64
#define VELO_PACKET_SIZE         1206

/*
 * Publishes on "/driving/velodyne/raw_packets" and "/driving/velodyne/stat"
 *
 */
class VelodyneDriverNodelet : public nodelet::Nodelet
{
public:
  VelodyneDriverNodelet();
  virtual ~VelodyneDriverNodelet();

  void onInit();

private:
  void run();
  void runDiagPub();

  int openSocket(uint16_t port);

  void addPacket(velodyne_msgs::VelodyneRawPacket const& pkt); // add packet to our buffer of scan data

  std::string velodyne_frame_id_;
  ros::NodeHandle nh_;
  ros::Publisher raw_scan_pub_, diag_pub_;

  // socket
  int32_t port_;
  int32_t packet_publish_count_;

  // Run asynchronously
  volatile bool quit_req_;
  boost::thread run_thread_;

  // diagnostic state (accessed in BG thread)
  boost::mutex diag_mutex_;
  boost::thread diag_thread_;
  size_t diag_total_num_bytes_;
  boost::circular_buffer<ros::Time> diag_received_data_;

  velodyne_msgs::VelodyneRawScan::Ptr raw_scan_; // buffer of scan data to publish
};

} // namespace velodyne

#endif
