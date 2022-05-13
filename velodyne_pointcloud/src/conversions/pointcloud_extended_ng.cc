

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <velodyne_pointcloud/pointcloud_extended_ng.h>

namespace velodyne_pointcloud
{
PointcloudExtendedNg::PointcloudExtendedNg(const double max_range,
                                           const double min_range,
                                           const std::string& target_frame,
                                           const std::string& fixed_frame,
                                           const unsigned int num_lasers, // determines the width of the cloud
                                           const unsigned int scans_per_block)
    : DataContainerBase(max_range,
                        min_range,
                        target_frame,
                        fixed_frame,
                        num_lasers,
                        0,
                        false,
                        scans_per_block,
                        13,
                        "x",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "y",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "z",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "x_sensor",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "y_sensor",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "z_sensor",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "distance",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "time",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "azimuth",
                        1,
                        sensor_msgs::PointField::UINT16,
                        "rotation_segment",
                        1,
                        sensor_msgs::PointField::UINT16,
                        "firing_bin",
                        1,
                        sensor_msgs::PointField::UINT16,
                        "ring",
                        1,
                        sensor_msgs::PointField::UINT16,
                        "intensity",
                        1,
                        sensor_msgs::PointField::UINT8),

      iter_x(cloud, "x"),
      iter_y(cloud, "y"),
      iter_z(cloud, "z"),
      iter_x_sensor(cloud, "x_sensor"),
      iter_y_sensor(cloud, "y_sensor"),
      iter_z_sensor(cloud, "z_sensor"),
      iter_distance(cloud, "distance"),
      iter_time(cloud, "time"),
      iter_azimuth(cloud, "azimuth"),
      iter_rotation_segment(cloud, "rotation_segment"),
      iter_firing_bin(cloud, "firing_bin"),
      iter_ring(cloud, "ring"),
      iter_intensity(cloud, "intensity")
{
}

void PointcloudExtendedNg::setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg)
{
    DataContainerBase::setup(scan_msg);

    // try to approximate height (it is not known beforehand in online mode)
    if (prev_height_ == 0)
    {
        // first rotation
        expected_height_ = 1000;
    }
    else
    {
        // subsequent rotations
        if (abs(prev_height_ - expected_height_) < 100)
            expected_height_ = static_cast<int>(expected_height_ * 0.8 + prev_height_ * 0.2);
        else
            expected_height_ = prev_height_;
    }
    prev_height_ = 0;
    cloud.height = expected_height_;
    azimuth_step_ = two_pi_ / expected_height_;

    // resize buffer to the correct size
    cloud.data.resize(expected_height_ * cloud.row_step);

    iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    iter_x_sensor = sensor_msgs::PointCloud2Iterator<float>(cloud, "x_sensor");
    iter_y_sensor = sensor_msgs::PointCloud2Iterator<float>(cloud, "y_sensor");
    iter_z_sensor = sensor_msgs::PointCloud2Iterator<float>(cloud, "z_sensor");
    iter_distance = sensor_msgs::PointCloud2Iterator<float>(cloud, "distance");
    iter_time = sensor_msgs::PointCloud2Iterator<float>(cloud, "time");
    iter_azimuth = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "azimuth");
    iter_rotation_segment = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "rotation_segment");
    iter_firing_bin = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "firing_bin");
    iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "ring");
    iter_intensity = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud, "intensity");

    // fill with NaN's
    for (int firing_idx = 0; firing_idx < expected_height_; ++firing_idx)
    {
        for (int ring_idx = 0; ring_idx < config_.init_width; ++ring_idx)
        {
            int cur_data_idx = firing_idx * static_cast<int>(config_.init_width) + ring_idx;
            *(iter_x + cur_data_idx) = nanf("");
            *(iter_y + cur_data_idx) = nanf("");
            *(iter_z + cur_data_idx) = nanf("");
            *(iter_x_sensor + cur_data_idx) = nanf("");
            *(iter_y_sensor + cur_data_idx) = nanf("");
            *(iter_z_sensor + cur_data_idx) = nanf("");
            *(iter_distance + cur_data_idx) = nanf("");
            *(iter_time + cur_data_idx) = nanf("");
            *(iter_azimuth + cur_data_idx) =
                static_cast<int>((static_cast<float>(firing_idx) * azimuth_step_) * rad_to_deg_times_100_);
            *(iter_rotation_segment + cur_data_idx) = firing_idx;
            *(iter_firing_bin + cur_data_idx) = std::numeric_limits<uint16_t>::max();
            *(iter_ring + cur_data_idx) = ring_idx;
            *(iter_intensity + cur_data_idx) = 0;
        }
    }

    sensor_position_ = tf2::Vector3(0, 0, 0);
    try
    {
        geometry_msgs::TransformStamped tf =
            tf_buffer->lookupTransform(config_.fixed_frame, sensor_frame, scan_msg->header.stamp, ros::Duration(0.1));
        fromMsg(tf.transform.translation, sensor_position_);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void PointcloudExtendedNg::newLine()
{
    ++prev_height_;
}

void PointcloudExtendedNg::addPoint(float x,
                                    float y,
                                    float z,
                                    const uint16_t ring,
                                    const uint16_t azimuth,
                                    const float distance,
                                    const float intensity,
                                    const float time)
{
    if (pointInRange(distance))
    {
        float x_sensor = x;
        float y_sensor = y;
        float z_sensor = z;

        // convert polar coordinates to Euclidean XYZ
        transformPoint(x, y, z);

        float azimuth_after_transform =
            -std::atan2(y - static_cast<float>(sensor_position_.y()), x - static_cast<float>(sensor_position_.x()));
        if (azimuth_after_transform < 0)
            azimuth_after_transform = two_pi_ + azimuth_after_transform;

        int azimuth_idx = static_cast<int>(azimuth_after_transform / azimuth_step_);
        if (azimuth_idx < 0) // can happen due to numeric errors
            azimuth_idx += expected_height_;
        else if (azimuth_idx >= expected_height_)
            azimuth_idx -= expected_height_;

        int offset = ring + azimuth_idx * static_cast<int>(config_.init_width);
        *(iter_x + offset) = x;
        *(iter_y + offset) = y;
        *(iter_z + offset) = z;
        *(iter_x_sensor + offset) = x_sensor;
        *(iter_y_sensor + offset) = y_sensor;
        *(iter_z_sensor + offset) = z_sensor;
        *(iter_distance + offset) = distance;
        *(iter_time + offset) = time;
        *(iter_azimuth + offset) = azimuth;
        *(iter_rotation_segment + offset) = azimuth_idx;
        *(iter_firing_bin + offset) = prev_height_;
        *(iter_intensity + offset) = static_cast<uint8_t>(intensity);
        *(iter_ring + offset) = ring;
    }
}
} // namespace velodyne_pointcloud
