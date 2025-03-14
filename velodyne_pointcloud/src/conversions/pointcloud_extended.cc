


#include <velodyne_pointcloud/pointcloud_extended.h>

namespace velodyne_pointcloud 
{
PointcloudExtended::PointcloudExtended(
    const double max_range, const double min_range,
    const std::string& target_frame, const std::string& fixed_frame,
    const unsigned int num_lasers, // determines the width of the cloud
    const unsigned int points_per_packet)
    : DataContainerBase(
        max_range, min_range, target_frame, fixed_frame,
        num_lasers, 0, false, points_per_packet, 13,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "distance", 1, sensor_msgs::PointField::FLOAT32,
        "time", 1, sensor_msgs::PointField::FLOAT32,
        "sub_segment", 1, sensor_msgs::PointField::UINT32,
        "azimuth", 1, sensor_msgs::PointField::UINT16,
        "rotation_segment", 1, sensor_msgs::PointField::UINT16,
        "firing_bin", 1, sensor_msgs::PointField::UINT16,
        "ring", 1, sensor_msgs::PointField::UINT16,
        "intensity", 1, sensor_msgs::PointField::UINT8,
        "laser_id", 1, sensor_msgs::PointField::UINT8,
        "return_type", 1, sensor_msgs::PointField::UINT8
        ),

      iter_x(*cloud, "x"),
      iter_y(*cloud, "y"),
      iter_z(*cloud, "z"),
      iter_distance(*cloud, "distance"),
      iter_time(*cloud, "time"),
      iter_sub_segment(*cloud, "sub_segment"),
      iter_rotation_segment(*cloud, "rotation_segment"),
      iter_azimuth(*cloud, "azimuth"),
      iter_firing_bin(*cloud, "firing_bin"),
      iter_ring(*cloud, "ring"),
      iter_intensity(*cloud, "intensity"),
      iter_laser_id(*cloud, "laser_id"),
      iter_ret_type(*cloud, "return_type")
    {
    }

  void PointcloudExtended::setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg){
    DataContainerBase::setup(scan_msg);
    iter_x = sensor_msgs::PointCloud2Iterator<float>(*cloud, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(*cloud, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(*cloud, "z");
    iter_distance = sensor_msgs::PointCloud2Iterator<float>(*cloud,"distance");
    iter_time = sensor_msgs::PointCloud2Iterator<float>(*cloud,"time");
    iter_sub_segment = sensor_msgs::PointCloud2Iterator<uint32_t>(*cloud,"sub_segment");
    iter_azimuth = sensor_msgs::PointCloud2Iterator<uint16_t>(*cloud,"azimuth");
    iter_rotation_segment = sensor_msgs::PointCloud2Iterator<uint16_t>(*cloud,"rotation_segment");
    iter_firing_bin = sensor_msgs::PointCloud2Iterator<uint16_t>(*cloud,"firing_bin");
    iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t>(*cloud,"ring");
    iter_intensity = sensor_msgs::PointCloud2Iterator<uint8_t>(*cloud,"intensity");
    iter_laser_id = sensor_msgs::PointCloud2Iterator<uint8_t>(*cloud,"laser_id");
    iter_ret_type = sensor_msgs::PointCloud2Iterator<uint8_t>(*cloud,"return_type");

  }

  void PointcloudExtended::newLine()
  {
    ++cloud->height;
  }

void PointcloudExtended::addPoint(float x, float y, float z, const uint16_t ring,
                                  const uint16_t azimuth, const float distance,
                                  const float intensity, const float time)
{
  uint64_t  offset = ring + cloud->height * config_.init_width;
  if (!pointInRange(distance)) {
    // convert polar coordinates to Euclidean XYZ

    transformPoint(x, y, z);

    *(iter_x + offset) = x;
    *(iter_y + offset) = y;
    *(iter_z + offset) = z;
    *(iter_distance + offset) = distance;
    *(iter_time + offset) = time;
    *(iter_sub_segment + offset) = 0;
    *(iter_azimuth + offset) = azimuth;
    *(iter_rotation_segment + offset) = 0;
    *(iter_firing_bin + offset) = 0;
    *(iter_intensity + offset) = static_cast<uint8_t>(intensity);
    *(iter_ring + offset) = ring;
    *(iter_laser_id + offset) = 0;
    *(iter_ret_type + offset) = 0;
  } else {
    *(iter_x + offset) = nanf("");
    *(iter_y + offset) = nanf("");
    *(iter_z + offset) = nanf("");
    *(iter_distance + offset) = nanf("");
    *(iter_time + offset) = time;
    *(iter_sub_segment + offset) = 0;
    *(iter_azimuth + offset) = azimuth;
    *(iter_rotation_segment + offset) = 0;
    *(iter_firing_bin + offset) = 0;
    *(iter_intensity + offset) = static_cast<uint8_t>(intensity);
    *(iter_ring + offset) = ring;
    *(iter_laser_id + offset) = 0;
    *(iter_ret_type + offset) = 0;
  }
}

  void PointcloudExtended::addPoint(float x, float y, float z, const uint16_t ring,
                                    const uint16_t azimuth, const float distance,
                                    const float intensity, const float time,
                                    const uint32_t sub_segment, const uint16_t  rotation_segment,
                                    const uint16_t  firing_bin, const uint8_t laser_id,
                                    const uint8_t return_type)
  {
    const auto first_return_flag = velodyne_rawdata::is_flag_set(return_type,velodyne_rawdata::FIRST_RETURN_FLAG)?1:0;
    const uint64_t  off_sec_ret =
            (1-first_return_flag) * (packets_in_scan * config_.points_per_packet/2);
    const uint64_t  off_first_ret = ring + rotation_segment * config_.init_width;
    const uint64_t  offset = off_first_ret + off_sec_ret;

    if (pointInRange(distance)) {
        // convert polar coordinates to Euclidean XYZ
        transformPoint(x, y, z);

        *(iter_x + offset) = x;
        *(iter_y + offset) = y;
        *(iter_z + offset) = z;
        *(iter_distance + offset) = distance;
        *(iter_time + offset) = time;
        *(iter_sub_segment + offset) = sub_segment;
        *(iter_azimuth + offset) = azimuth;
        *(iter_rotation_segment + offset) = rotation_segment;
        *(iter_firing_bin + offset) = firing_bin;
        *(iter_intensity + offset) = static_cast<uint8_t>(intensity);
        *(iter_ring + offset) = ring;
        *(iter_laser_id + offset) = laser_id;
        *(iter_ret_type + offset) = return_type;
    } else {

        *(iter_x + offset) = nanf("");
        *(iter_y + offset) = nanf("");
        *(iter_z + offset) = nanf("");
        *(iter_distance + offset) = nanf("");
        *(iter_time + offset) = time;
        *(iter_sub_segment + offset) = sub_segment;
        *(iter_azimuth + offset) = azimuth;
        *(iter_rotation_segment + offset) = rotation_segment;
        *(iter_firing_bin + offset) = firing_bin;
        *(iter_intensity + offset) = static_cast<uint8_t>(intensity);
        *(iter_ring + offset) = ring;
        *(iter_laser_id + offset) = laser_id;
        *(iter_ret_type + offset) = return_type;
    }

  }
}

