


#include <velodyne_pointcloud/pointcloud_extended_confidence.h>

namespace velodyne_pointcloud {

    PointcloudExtendedConfidence::PointcloudExtendedConfidence(const double max_range, const double min_range,
                                                               const std::string &target_frame,
                                                               const std::string &fixed_frame,
                                                               const unsigned int num_lasers,
                                                               const unsigned int scans_per_packet)
                                                               : DataContainerBase(
            max_range, min_range, target_frame, fixed_frame,
            num_lasers, 0, false, scans_per_packet, 20,
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
            "first_return_flag", 1, sensor_msgs::PointField::UINT8,
            "drop", 1, sensor_msgs::PointField::UINT8,
            "retro_shadow", 1, sensor_msgs::PointField::UINT8,
            "range_limited", 1, sensor_msgs::PointField::UINT8,
            "retro_ghost", 1, sensor_msgs::PointField::UINT8,
            "interference", 1, sensor_msgs::PointField::UINT8,
            "sun_lvl", 1, sensor_msgs::PointField::UINT8,
            "confidence", 1, sensor_msgs::PointField::UINT8), iter_x(cloud, "x"),
          iter_y(cloud,"y"),
          iter_z(cloud,"z"),
          iter_distance(cloud,"distance"),
          iter_time(cloud,"time"),
          iter_sub_segment(cloud,"sub_segment"),
          iter_rotation_segment(cloud,"rotation_segment"),
          iter_azimuth(cloud,"azimuth"),
          iter_firing_bin(cloud,"firing_bin"),
          iter_ring(cloud,"ring"),
          iter_intensity(cloud,"intensity"),
          iter_laser_id(cloud,"laser_id"),
          iter_first_ret(cloud,"first_return_flag"),
          iter_drop(cloud,"drop"),
          iter_retro_shadow(cloud,"retro_shadow"),
          iter_range_limited(cloud,"range_limited"),
          iter_retro_ghost(cloud,"retro_ghost"),
          iter_interference(cloud,"interference"),
          iter_sun_lvl(cloud,"sun_lvl"),
          iter_confidence(cloud,"confidence")
          {
            }


    void PointcloudExtendedConfidence::setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg){
        DataContainerBase::setup(scan_msg);
        iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
        iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
        iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
        iter_distance = sensor_msgs::PointCloud2Iterator<float>(cloud,"distance");
        iter_time = sensor_msgs::PointCloud2Iterator<float>(cloud,"time");
        iter_sub_segment = sensor_msgs::PointCloud2Iterator<uint32_t>(cloud,"sub_segment");
        iter_azimuth = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud,"azimuth");
        iter_rotation_segment = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud,"rotation_segment");
        iter_firing_bin = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud,"firing_bin");
        iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud,"ring");
        iter_intensity = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud,"intensity");
        iter_laser_id = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud,"laser_id");
        iter_first_ret = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud,"first_return_flag");
        iter_drop = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud,"drop");
        iter_retro_shadow = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud,"retro_shadow");
        iter_range_limited = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud,"range_limited");
        iter_retro_ghost = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud,"retro_ghost");
        iter_interference = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud,"interference");
        iter_sun_lvl = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud,"sun_lvl");
        iter_confidence = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud,"confidence");

    }

    void PointcloudExtendedConfidence::newLine()
    {
        ++cloud.height;
    }

    void PointcloudExtendedConfidence::addPoint(float x, float y, float z, const uint16_t ring,
                                      const uint16_t azimuth, const float distance,
                                      const float intensity, const float time)
    {
        uint64_t  offset = ring + cloud.height * config_.init_width;
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
            *(iter_first_ret + offset) = 0;
            *(iter_drop + offset) = 77;
            *(iter_retro_shadow + offset) = 77;
            *(iter_range_limited + offset) = 77;
            *(iter_retro_ghost + offset) = 77;
            *(iter_interference + offset) = 77;
            *(iter_sun_lvl + offset) = 77;
            *(iter_confidence + offset) = 77;
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
            *(iter_first_ret + offset) = 0;
            *(iter_drop + offset) = 44;
            *(iter_retro_shadow + offset) = 44;
            *(iter_range_limited + offset) = 44;
            *(iter_retro_ghost + offset) = 44;
            *(iter_interference + offset) = 44;
            *(iter_sun_lvl + offset) = 44;
            *(iter_confidence + offset) = 44;
        }
    }

    void PointcloudExtendedConfidence::addPoint(float x, float y, float z, const uint16_t ring,
                                      const uint16_t azimuth, const float distance,
                                      const float intensity, const float time,
                                      const uint32_t sub_segment, const uint16_t  rotation_segment,
                                      const uint16_t  firing_bin, const uint8_t laser_id, const uint8_t first_return_flag)
    {

        const uint64_t  off_sec_ret =
                (1-first_return_flag) * (packets_in_scan * config_.points_per_packet);
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
            *(iter_first_ret + offset) = first_return_flag;
            *(iter_drop + offset) = 22;
            *(iter_retro_shadow + offset) = 22;
            *(iter_range_limited + offset) = 22;
            *(iter_retro_ghost + offset) = 22;
            *(iter_interference + offset) = 22;
            *(iter_sun_lvl + offset) = 22;
            *(iter_confidence + offset) = 22;
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
            *(iter_first_ret + offset) = first_return_flag;
            *(iter_drop + offset) = 33;
            *(iter_retro_shadow + offset) = 33;
            *(iter_range_limited + offset) = 33;
            *(iter_retro_ghost + offset) = 33;
            *(iter_interference + offset) = 33;
            *(iter_sun_lvl + offset) = 33;
            *(iter_confidence + offset) = 33;
        }
    }

    void PointcloudExtendedConfidence::addPoint_with_confidence(float x, float y, float z, const uint16_t ring,
                                                                const uint16_t azimuth, const float distance,
                                                                const float intensity, const float time,
                                                                const uint32_t sub_segment,
                                                                const uint16_t rotation_segment,
                                                                const uint16_t firing_bin, const uint8_t laser_id,
                                                                const uint8_t first_return_flag,
                                                                const uint8_t drop,
                                                                const uint8_t retro_shadow,
                                                                const uint8_t range_limited,
                                                                const uint8_t retro_ghost,
                                                                const uint8_t interference,
                                                                const uint8_t sun_lvl,
                                                                const uint8_t confidence)
    {

        const uint64_t  off_sec_ret =
                (1-first_return_flag) * (packets_in_scan * config_.points_per_packet);
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
            *(iter_first_ret + offset) = first_return_flag;
            *(iter_drop + offset) = drop;
            *(iter_retro_shadow + offset) = retro_shadow;
            *(iter_range_limited + offset) = range_limited;
            *(iter_retro_ghost + offset) = retro_ghost;
            *(iter_interference + offset) = interference;
            *(iter_sun_lvl + offset) = sun_lvl;
            *(iter_confidence + offset) = confidence;
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
            *(iter_first_ret + offset) = first_return_flag;
            *(iter_drop + offset) = 88;
            *(iter_retro_shadow + offset) = 88;
            *(iter_range_limited + offset) = 88;
            *(iter_retro_ghost + offset) = 88;
            *(iter_interference + offset) = 88;
            *(iter_sun_lvl + offset) = 88;
            *(iter_confidence + offset) = 88;
        }

    }
}