
#include <velodyne_pointcloud/streaming_cloudXYZIRT.h>

namespace velodyne_pointcloud
{
StreamingCloudXYZIRT::StreamingCloudXYZIRT(ros::Publisher* pub,
                                           const std::string& sensor_frame,
                                           const unsigned num_accumulated_firings,
                                           const unsigned int num_lasers)
    : DataContainerBase(0,
                        0,
                        "",
                        "",
                        num_accumulated_firings,
                        num_lasers,
                        true,
                        0,
                        10,
                        "x",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "y",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "z",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "distance",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "azimuth",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "intensity",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "row_index",
                        1,
                        sensor_msgs::PointField::UINT16,
                        "firing_index",
                        1,
                        sensor_msgs::PointField::UINT32,
                        "time_sec",
                        1,
                        sensor_msgs::PointField::UINT32,
                        "time_nsec",
                        1,
                        sensor_msgs::PointField::UINT32),
      iter_x(cloud, "x"),
      iter_y(cloud, "y"),
      iter_z(cloud, "z"),
      iter_distance(cloud, "distance"),
      iter_azimuth(cloud, "azimuth"),
      iter_intensity(cloud, "intensity"),
      iter_row_index(cloud, "row_index"),
      iter_firing_index(cloud, "firing_index"),
      iter_time_sec(cloud, "time_sec"),
      iter_time_nsec(cloud, "time_nsec"),
      pub(pub)
{
    cloud.header.frame_id = sensor_frame;
    cloud.width = num_accumulated_firings;
    cloud.height = num_lasers;
    cloud.is_dense = true;
    cloud.data.resize(num_lasers * num_accumulated_firings * cloud.point_step);

    // the iterators seem to require some parameters above, therefore they have to be initialized again
    iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    iter_distance = sensor_msgs::PointCloud2Iterator<float>(cloud, "distance");
    iter_azimuth = sensor_msgs::PointCloud2Iterator<float>(cloud, "azimuth");
    iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
    iter_row_index = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "row_index");
    iter_firing_index = sensor_msgs::PointCloud2Iterator<uint32_t>(cloud, "firing_index");
    iter_time_sec = sensor_msgs::PointCloud2Iterator<uint32_t>(cloud, "time_sec");
    iter_time_nsec = sensor_msgs::PointCloud2Iterator<uint32_t>(cloud, "time_nsec");
}

void StreamingCloudXYZIRT::newLine()
{
    global_firing_index++;
    local_firing_index++;

    if (local_firing_index >= cloud.width)
    {
        // publish cloud
        cloud.header.stamp.sec = *(iter_time_sec);   // use stamp of topmost laser
        cloud.header.stamp.nsec = *(iter_time_nsec); // use stamp of topmost laser
        pub->publish(cloud);

        // restart local firing index
        local_firing_index = 0;
    }
}

void StreamingCloudXYZIRT::setPacketStamp(ros::Time packet_stamp)
{
    double dt = (packet_stamp - cur_packet_stamp).toSec();
    if (ros::Time::isSimTime() && std::abs(dt) > 0.1)
    {
        ROS_WARN_STREAM("Detected jump in time: " << dt << ". Reset velodyne driver.");
        local_firing_index = 0;
        global_firing_index = 0;
    }
    cur_packet_stamp = packet_stamp;
}

void StreamingCloudXYZIRT::addPoint(float x,
                                    float y,
                                    float z,
                                    const uint16_t ring,
                                    const uint16_t azimuth,
                                    const float distance,
                                    const float intensity,
                                    const float time)
{
    // calculate time
    ros::Duration duration_in_packet;
    ros::Time stamp = cur_packet_stamp + duration_in_packet.fromSec(time);

    int row_index = static_cast<int>(cloud.height) - ring - 1;
    int data_index = row_index * static_cast<int>(cloud.width) + local_firing_index;
    *(iter_azimuth + data_index) = azimuth;
    *(iter_row_index + data_index) = row_index;
    *(iter_firing_index + data_index) = global_firing_index;
    *(iter_time_sec + data_index) = stamp.sec;
    *(iter_time_nsec + data_index) = stamp.nsec;

    if (distance > 0)
    {
        *(iter_x + data_index) = x;
        *(iter_y + data_index) = y;
        *(iter_z + data_index) = z;
        *(iter_distance + data_index) = distance;
        *(iter_intensity + data_index) = intensity;
    }
    else
    {
        *(iter_x + data_index) = nanf("");
        *(iter_y + data_index) = nanf("");
        *(iter_z + data_index) = nanf("");
        *(iter_distance + data_index) = nanf("");
        *(iter_intensity + data_index) = 0;
    }
}
} // namespace velodyne_pointcloud
