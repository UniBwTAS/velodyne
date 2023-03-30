// Copyright (C) 2012, 2019 Austin Robot Technology, Jack O'Quin, Joshua Whitley, Sebastian Pütz
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

#ifndef ROS_OBJECT_INSTANCES_POINTCLOUD_EXTENDED_H
#define ROS_OBJECT_INSTANCES_POINTCLOUD_EXTENDED_H


#include <velodyne_pointcloud/datacontainerbase.h>
#include <string>

namespace velodyne_pointcloud
{
class PointcloudExtended : public velodyne_rawdata::DataContainerBase
{
public:
  PointcloudExtended(const double max_range, const double min_range, const std::string& target_frame,
                   const std::string& fixed_frame, const unsigned int num_lasers,
                     const unsigned int scans_per_packet);

  virtual void newLine();

  virtual void setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg);

  void addPoint(float x, float y, float z, const uint16_t ring,
                const uint16_t azimuth, const float distance,
                const float intensity, const float time) override;

  // With the VSL 128 the ring and the rotation segment can be used to build a range image
  void addPoint(float x, float y, float z, const uint16_t ring,
                        const uint16_t azimuth, const float distance,
                        const float intensity, const float time,
                        const uint32_t sub_segment, const uint16_t  rotation_segment,
                        const uint16_t  firing_bin, const uint8_t laser_id, const uint8_t first_return_flag) override ;

  sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z,  iter_distance, iter_time;
  sensor_msgs::PointCloud2Iterator<uint32_t> iter_sub_segment;
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_rotation_segment, iter_azimuth, iter_firing_bin, iter_ring;
  sensor_msgs::PointCloud2Iterator<uint8_t>   iter_intensity, iter_laser_id, iter_first_ret;


};
}  // namespace velodyne_pointcloud



#endif // ROS_OBJECT_INSTANCES_POINTCLOUD_EXTENDED_H
