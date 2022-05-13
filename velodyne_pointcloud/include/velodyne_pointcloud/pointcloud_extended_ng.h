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

#ifndef VELODYNE_POINTCLOUD_POINTCLOUD_EXTENDED_H
#define VELODYNE_POINTCLOUD_POINTCLOUD_EXTENDED_H

#include <string>
#include <velodyne_pointcloud/datacontainerbase.h>

namespace velodyne_pointcloud
{
class PointcloudExtendedNg : public velodyne_rawdata::DataContainerBase
{
  public:
    PointcloudExtendedNg(const double max_range,
                         const double min_range,
                         const std::string& target_frame,
                         const std::string& fixed_frame,
                         const unsigned int num_lasers,
                         const unsigned int scans_per_block);

    virtual void newLine();

    virtual void setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg);

    void addPoint(float x,
                  float y,
                  float z,
                  const uint16_t ring,
                  const uint16_t azimuth,
                  const float distance,
                  const float intensity,
                  const float time) override;

    sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z, iter_x_sensor, iter_y_sensor, iter_z_sensor,
        iter_distance, iter_time;
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_rotation_segment, iter_azimuth, iter_firing_bin, iter_ring;
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity;
    int expected_height_{};
    int prev_height_{0};
    float two_pi_{static_cast<float>(M_PI * 2.)};
    float azimuth_step_{};
    tf2::Vector3 sensor_position_;
    float rad_to_deg_times_100_{static_cast<float>((180 / M_PI) * 100)};
};
} // namespace velodyne_pointcloud

#endif // VELODYNE_POINTCLOUD_POINTCLOUD_EXTENDED_H
