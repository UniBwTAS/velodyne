//
// Created by jugo on 05.04.23.
//

#ifndef VELODYNE_POINTCLOUD_POINTCLOUD_EXTENDED_CONFIDENCE_H
#define VELODYNE_POINTCLOUD_POINTCLOUD_EXTENDED_CONFIDENCE_H

#include <velodyne_pointcloud/datacontainerbase.h>
#include <string>

namespace velodyne_pointcloud
{

class PointcloudExtendedConfidence : public velodyne_rawdata::DataContainerBase
{
public:

    PointcloudExtendedConfidence(const double max_range, const double min_range, const std::string& target_frame,
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

    // With the VSL 128 and return mode dual plus confidence more information about each firing can be obtained.
    void addPoint_with_confidence(float x, float y, float z, const uint16_t ring,
                                  const uint16_t azimuth, const float distance,
                                  const float intensity, const float time,
                                  const uint32_t sub_segment, const uint16_t rotation_segment,
                                  const uint16_t firing_bin, const uint8_t laser_id, const uint8_t first_return_flag,
                                  const uint8_t drop,
                                  const uint8_t retro_shadow,
                                  const uint8_t range_limited,
                                  const uint8_t retro_ghost,
                                  const uint8_t interference,
                                  const uint8_t sun_lvl,
                                  const uint8_t confidence) override;


    sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z,  iter_distance, iter_time;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_sub_segment;
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_rotation_segment, iter_azimuth, iter_firing_bin, iter_ring;
    sensor_msgs::PointCloud2Iterator<uint8_t>   iter_intensity, iter_laser_id, iter_first_ret;

    // iterators for confidence fields


    sensor_msgs::PointCloud2Iterator<uint8_t> iter_drop, iter_retro_shadow,
            iter_range_limited,
            iter_retro_ghost,
            iter_interference,
            iter_sun_lvl,
            iter_confidence;




};

}



#endif //VELODYNE_POINTCLOUD_POINTCLOUD_EXTENDED_CONFIDENCE_H
