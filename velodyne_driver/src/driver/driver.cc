// Copyright (C) 2007, 2009-2012 Austin Robot Technology, Patrick Beeson, Jack O'Quin
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "velodyne_driver/driver.h"

namespace velodyne_driver
{
VelodyneDriver::VelodyneDriver(ros::NodeHandle node, const ros::NodeHandle& private_nh, std::string const& node_name)
{
  // use private node handle to get parameters
  std::string pcap_file, timestamp_method;
  double cut_angle;
  int udp_port;
  private_nh.param("sensor_frame", config_.sensor_frame, std::string("velodyne"));
  private_nh.param("timestamp_method", timestamp_method, std::string("end"));
  private_nh.param("cut_angle", cut_angle, 0.);
  private_nh.param("port", udp_port, (int)DATA_PORT_NUMBER);
  private_nh.param("pcap", pcap_file, std::string(""));
  // parse old parameters for compatibility (TODO: remove this line in a future release)
  parse_deprecated_parameters(private_nh, cut_angle, timestamp_method);

  // process, store & print parameters
  if(timestamp_method == "end") {
    config_.timestamp_method = SCAN_END;
  } else if (timestamp_method == "start") {
    config_.timestamp_method = SCAN_START;
  } else if (timestamp_method == "middle") {
    config_.timestamp_method = SCAN_MIDDLE;
  } else {
    ROS_ERROR("Invalid value for parameter 'timestamp_method'. Should be one of 'end', 'start', or 'middle'.");
    config_.timestamp_method = SCAN_END;
  }
  ROS_INFO("Setting timestamp of velodyne scan to: %s of scan", timestamp_method.c_str());
  if (cut_angle >= 0.0 && cut_angle < (2 * M_PI))
  {
    ROS_INFO_STREAM("Cutting velodyne points at " << cut_angle << " rad.");
  }
  else
  {
    ROS_ERROR_STREAM("'cut_angle' parameter is out of range. Allowed range is between 0.0 and 2*PI.");
    cut_angle = 0;
  }
  // Convert cut_angle from radian to one-hundredth degree,
  // which is used in velodyne packets
  config_.cut_angle = int((cut_angle * 360 / (2 * M_PI)) * 100);

  // Initialize dynamic reconfigure
  srv_ = std::make_shared<dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig> >(private_nh);
  srv_->setCallback([this](velodyne_driver::VelodyneNodeConfig& config, uint32_t level) { callback(config, level); });

  config_.enabled = true;

  // open Velodyne input device or file
  if (!pcap_file.empty())  // have PCAP file?
  {
    // read data from packet capture file
    input_.reset(new velodyne_driver::InputPCAP(private_nh, udp_port, pcap_file));
  }
  else
  {
    // read data from live socket
    input_.reset(new velodyne_driver::InputSocket(private_nh, udp_port));
  }

  // raw packet output topic
  output_ = node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);

  // specify invalid last azimuth
  last_azimuth_ = -1;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll()
{
  if (!config_.enabled)
  {
    // If we are not enabled exit once a second to let the caller handle
    // anything it might need to, such as if it needs to exit.
    ros::Duration(1).sleep();
    return true;
  }

  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);

  // make memory allocation more efficient by initializing with proper amount of memory
  if (config_.max_packets > 0)
    scan->packets.resize(config_.max_packets);
  else
    scan->packets.resize(512); // some initial guess about max_packets

  // read UDP packets
  size_t cur_packet_idx = 0;
  while (true)
  {
    // check if current size is to small (after one/few rotations this will not happen any more because we properly
    // resized with max_packets above)
    if (cur_packet_idx >= scan->packets.size())
    {
      scan->packets.resize(scan->packets.size() * 2);
    }

    // read single packet
    while (true)
    {
      int rc = input_->getPacket(&scan->packets[cur_packet_idx], config_.time_offset);
      if (rc == 0)
        break;  // got a full packet?
      if (rc < 0)
        return false;  // end of file reached?
    }

    // Extract base rotation of first block in packet
    std::size_t azimuth_data_pos = 100 * 0 + 2;
    int cur_azimuth = *((u_int16_t*)(&scan->packets[cur_packet_idx].data[azimuth_data_pos]));

    cur_packet_idx++;

    if (passedCutAngle(cur_azimuth))
    {
      scan->packets.resize(cur_packet_idx); // shrink size (only necessary in initialization phase)
      break;
    }
  }

  // keep track of the maximum number of packets seen so far
  config_.max_packets = std::max(config_.max_packets, scan->packets.size());

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Velodyne scan.");
  if (config_.timestamp_method == SCAN_START)
  {
    scan->header.stamp = scan->packets.front().stamp;
  }
  else if (config_.timestamp_method == SCAN_END)
  {
    scan->header.stamp = scan->packets.back().stamp;
  }
  else if (config_.timestamp_method == SCAN_MIDDLE)
  {
    auto start = scan->packets.front().stamp;
    auto end = scan->packets.back().stamp;
    scan->header.stamp = start + (end - start) * 0.5;
  }
  scan->header.frame_id = config_.sensor_frame;
  output_.publish(scan);

  return true;
}

bool VelodyneDriver::passedCutAngle(int cur_azimuth)
{
  // if first packet in scan, there is no "valid" last_azimuth_
  if (last_azimuth_ == -1)
  {
    last_azimuth_ = cur_azimuth;
    return false;
  }

  // check if velodyne passed 360 degree
  bool new_revolution = cur_azimuth < last_azimuth_;

  // check if cut angle was passed. Including corner cases where the cut angle is close to 0/360 degree.
  bool passed_cut_angle = (last_azimuth_ < config_.cut_angle && cur_azimuth >= config_.cut_angle) ||
                          (new_revolution && cur_azimuth >= config_.cut_angle) ||  // if cut_angle = 0 + small_epsilon
                          (new_revolution && last_azimuth_ < config_.cut_angle);   // if cut_angle = 360 - small_epsilon

  // store current azimuth for next iteration
  last_azimuth_ = cur_azimuth;

  return passed_cut_angle;
}

void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  if (level & 1)
  {
    config_.time_offset = config.time_offset;
  }
  if (level & 2)
  {
    config_.enabled = config.enabled;
  }
}

void VelodyneDriver::parse_deprecated_parameters(const ros::NodeHandle& private_nh, double& cut_angle,
                                                 std::string& timestamp_method)
{
  // This method is designed in a way that it can be removed without breaking the new code (using the new parameters).

  std::string frame_id;
  if (private_nh.getParam("frame_id", frame_id))
  {
    ROS_WARN("Deprecation: parameter 'frame_id' was replaced by 'sensor_frame' to emphasize that it is the frame id of "
             "the sensor, i.e. the origin of the point cloud. Please update your launch files since this "
             "compatibility code will be removed in a future release.");
    config_.sensor_frame = frame_id;
  }

  if (cut_angle < 0)
  {
    ROS_WARN("Deprecation: the 'cut_angle' is now always enabled in order to simplify configuration and to make the "
             "driver more robust. Therefore negative cut_angles are no more valid. Please update your launch files "
             "since this compatibility code will be removed in a future release (cut_angle is set to zero).");
    cut_angle = 0;
  }

  bool timestamp_first_packet;
  if (private_nh.getParam("timestamp_first_packet", timestamp_first_packet))
  {
    ROS_WARN("Deprecation: parameter 'timestamp_first_packet' was replaced by 'timestamp_method' because it also "
             "allows to specify the middle of a scan (typically the most accurate approximation). Possible vales for "
             "'timestamp_method' are 'end', 'start', 'middle'. Please update your launch files since this "
             "compatibility code will be removed in a future release.");
    timestamp_method = timestamp_first_packet ? "start" : "end";
  }

  if (private_nh.getParam("rpm", timestamp_first_packet))
  {
    ROS_WARN("Deprecation: parameter 'rpm' is no more required because individual scans are separated based on given "
             "'cut_angle' and the current packet's azimuth angle. Please remove this parameter from your launch files "
             "as it has no effect any more.");
  }

  std::string model;
  if (private_nh.getParam("model", model))
  {
    ROS_WARN("Deprecation: parameter 'model' is no more required because individual scans are separated based on given "
             "'cut_angle' and the current packet's azimuth angle. However it is required if you want to play pcap "
             "files in order to calculate the appropriate packet rate (used to simulate inter packet delay). So it was "
             "renamed from 'model' to 'pcap_velodyne_model' to emphasize that this parameter is only required for "
             "replaying pcap files. Please update your launch files since this compatibility code will be removed in a "
             "future release.");  // compatibility code is located in input.cc
  }
  bool read_once;
  if (private_nh.getParam("read_once", read_once))
  {
    ROS_WARN("Deprecation: parameter 'read_once' was renamed to 'pcap_read_once' in order to emphasize that is only "
             "required for replaying pcap files. Please update your launch files since this compatibility code will be "
             "removed in a future release.");  // compatibility code is located in input.cc
  }
  bool read_fast;
  if (private_nh.getParam("read_fast", read_fast))
  {
    ROS_WARN("Deprecation: parameter 'read_fast' was renamed to 'pcap_read_fast' in order to emphasize that is only "
             "required for replaying pcap files. Please update your launch files since this compatibility code will be "
             "removed in a future release.");  // compatibility code is located in input.cc
  }
  double repeat_delay;
  if (private_nh.getParam("repeat_delay", repeat_delay))
  {
    ROS_WARN("Deprecation: parameter 'repeat_delay' was renamed to 'pcap_repeat_delay' in order to emphasize that is "
             "only required for replaying pcap files. Please update your launch files since this compatibility code "
             "will be removed in a future release.");  // compatibility code is located in input.cc
  }
}

}  // namespace velodyne_driver
