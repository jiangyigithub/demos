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

#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fnmatch.h>
#include <limits>

#include <vlr_exception/vlrException.h>
#include <driving_common/file_tools.h>
#include <velodyne/velodyne_driver_nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

#define MAX_NAME_LENGTH               256

#define MAXRECVLEN                    4096
#define MAX_SENSOR_TIMEOUT            0.5
#define DEFAULT_PACKET_PUBLISH_COUNT  100

namespace drc = driving_common;

namespace velodyne {

VelodyneDriverNodelet::VelodyneDriverNodelet() :
  port_(2368),
  quit_req_(false),
  diag_total_num_bytes_(0),
  diag_received_data_(100)
{
  raw_scan_.reset(new velodyne_msgs::VelodyneRawScan);
}

VelodyneDriverNodelet::~VelodyneDriverNodelet() {
  quit_req_ = true;
  run_thread_.join();
  diag_thread_.join();
}

void VelodyneDriverNodelet::onInit()
{
  nh_ = ros::NodeHandle(getNodeHandle(), "/driving");

  nh_.param("velodyne_frame_id", velodyne_frame_id_, std::string("Velodyne"));
  nh_.param("velodyne/packet_publish_count", packet_publish_count_, DEFAULT_PACKET_PUBLISH_COUNT);
  nh_.param("velodyne/port", port_, port_);

  raw_scan_->packets.reserve(packet_publish_count_);

  raw_scan_pub_   = nh_.advertise<velodyne_msgs::VelodyneRawScan> ("velodyne/raw_packets", 10);
  diag_pub_       = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 5);

  run_thread_  = boost::thread(boost::bind(&VelodyneDriverNodelet::run,        this));
  diag_thread_ = boost::thread(boost::bind(&VelodyneDriverNodelet::runDiagPub, this));
}

void VelodyneDriverNodelet::run()
{
  unsigned char data[MAXRECVLEN];
  int len, err;
  struct timeval t;
  fd_set set;

  ROS_INFO("Opening UDP socket %d to Velodyne...", port_);
  int sock = openSocket(port_);

  velodyne_msgs::VelodyneRawPacket raw_packet_msg;

  while (ros::ok() && !quit_req_) {
    FD_ZERO(&set);
    FD_SET(sock, &set);
    t.tv_sec = 1;
    t.tv_usec = 100000;
    err = select(sock + 1, &set, NULL, NULL, &t);

    if (err == 1) {
      len = recv(sock, &(data[0]), MAXRECVLEN, MSG_WAITALL);
      if (len < 0) {
        VLR_THROW_EXCEPTION("recvfrom() failed");
      }

      if (len != (int)raw_packet_msg.data.size()) // raw_packet_msg.data size is fixed, so we can check against it here
        continue;

      raw_packet_msg.stamp = ros::Time::now();
      memcpy(&raw_packet_msg.data[0], &data[0], len);
      addPacket(raw_packet_msg);

      {
        boost::lock_guard<boost::mutex> lock(diag_mutex_);
        diag_received_data_.push_back(raw_packet_msg.stamp);
        diag_total_num_bytes_ += len;
      }
    }
  }

  close(sock);
}

void VelodyneDriverNodelet::runDiagPub()
{
  ros::Rate r(1.0);

  //
  boost::circular_buffer<ros::Time> received_data;

  while (ros::ok() && !quit_req_)
  {
    r.sleep();

    // Copy diag data
    {
      boost::lock_guard<boost::mutex> lock(diag_mutex_);
      received_data   = diag_received_data_;
    }

    diagnostic_updater::DiagnosticStatusWrapper diag_stat;
    diag_stat.name        = "Velodyne";
    diag_stat.hardware_id = "Velodyne";
    diag_stat.message     = "OK";

    int num_updates = received_data.size();
    if (num_updates == 0)
    {
      diag_stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "No data from Velodyne");
      diag_stat.add("Time Since Update", "N/A");
      diag_stat.add("Update Frequency",  "N/A");
    }
    else
    {
      double calc_update_freq = 0.0;

      ros::Time first_stamp = received_data.front();
      ros::Time last_stamp  = received_data.back();
      if (num_updates > 1)
      {
        ros::Duration dt = (last_stamp - first_stamp);
        // freq calc: Remember that we have observed (n-1) "gaps" between messages if we have n stamps
        calc_update_freq = (num_updates - 1) / dt.toSec();
      }

      diag_stat.addf("Time Since Update", "%.2fs",   (ros::Time::now() - last_stamp).toSec());
      diag_stat.addf("Update Frequency",  "%.2fKHz", calc_update_freq / 1000.0);

      if (ros::Time::now() - last_stamp > ros::Duration(1.0))
      {
        diag_stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Velodyne data stale!");
      }
    } // have updates

    diagnostic_msgs::DiagnosticArray diag_array;
    diag_array.status.push_back(diag_stat);
    diag_array.header.stamp = ros::Time::now();

    diag_pub_.publish(diag_array);
  } // loop
}

void VelodyneDriverNodelet::addPacket(velodyne_msgs::VelodyneRawPacket const& pkt)
{
  raw_scan_->header.stamp    = pkt.stamp;
  raw_scan_->header.frame_id = velodyne_frame_id_;
  raw_scan_->packets.push_back(pkt);

  if (raw_scan_->packets.size() == packet_publish_count_) {
    raw_scan_pub_.publish(raw_scan_);

    // Reset the projected spin pointer for next data
    raw_scan_.reset(new velodyne_msgs::VelodyneRawScan);
    raw_scan_->packets.reserve(packet_publish_count_);
  }
}

int VelodyneDriverNodelet::openSocket(uint16_t port) {
  int sock_rmem_size = 2097152;
  int sock;
  struct sockaddr_in broadcastAddr; /* Broadcast Address */

  /* Create a best-effort datagram socket using UDP */
  if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    VLR_THROW_EXCEPTION("socket() failed");
  }

  /* Request a larger receive buffer window */
  if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &sock_rmem_size, sizeof(sock_rmem_size)) == -1) {
    ROS_ERROR_STREAM("Could not increase socket receive buffer to " << sock_rmem_size << ". This may cause dropped Velodyne data!");
  }

  /* Zero out structure */
  memset(&broadcastAddr, 0, sizeof(broadcastAddr));

  /* Internet address family */
  broadcastAddr.sin_family = AF_INET;

  /* Any incoming interface */
  broadcastAddr.sin_addr.s_addr = htonl(INADDR_ANY);

  /* Broadcast port */
  broadcastAddr.sin_port = htons(port);

  /* Bind to the broadcast port */
  if (bind(sock, (struct sockaddr *) &broadcastAddr, sizeof(broadcastAddr)) < 0) {
    VLR_THROW_EXCEPTION("bind() failed");
  }

  return sock;
}

PLUGINLIB_DECLARE_CLASS(velodyne, VelodyneDriverNodelet, velodyne::VelodyneDriverNodelet, nodelet::Nodelet);

} // namespace velodyne
