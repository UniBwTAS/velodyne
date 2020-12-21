// Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
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

#ifndef VELODYNE_DRIVER_DRIVER_H
#define VELODYNE_DRIVER_DRIVER_H

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <velodyne_driver/input.h>
#include <velodyne_driver/VelodyneNodeConfig.h>

namespace velodyne_driver
{

class VelodyneDriver
{
public:
  VelodyneDriver(ros::NodeHandle node,
                 const ros::NodeHandle& private_nh,
                 std::string const & node_name = ros::this_node::getName());
  ~VelodyneDriver() = default;

  bool poll();

private:
  // Callback for dynamic reconfigure
  void callback(velodyne_driver::VelodyneNodeConfig &config, uint32_t level);

  // Check if velodyne has passed specified cut angle
  inline bool passedCutAngle(int cur_azimuth);

  void parse_deprecated_parameters(const ros::NodeHandle& private_nh, double& cut_angle, std::string& timestamp_method);

  // Pointer to dynamic reconfigure service srv_
  std::shared_ptr<dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig> > srv_;

  enum TIMESTAMP_METHOD
  {
    SCAN_END = 0,
    SCAN_START,
    SCAN_MIDDLE
  };

  // configuration parameters
  struct
  {
    std::string sensor_frame;           // tf frame ID of the velodyne sensor (origin of the point cloud)
    int cut_angle;                      // cutting angle in 1/100°
    double time_offset;                 // time in seconds added to each velodyne time stamp
    bool enabled;                       // polling is enabled
    TIMESTAMP_METHOD timestamp_method;  // which timestamp to use for a whole scan (end, start, middle of scan)
    std::size_t max_packets{ 0 };       // maximum number of packets within a revolution seen so far
  } config_;

  std::shared_ptr<Input> input_;
  ros::Publisher output_;
  int last_azimuth_;
};

}  // namespace velodyne_driver

#endif  // VELODYNE_DRIVER_DRIVER_H
