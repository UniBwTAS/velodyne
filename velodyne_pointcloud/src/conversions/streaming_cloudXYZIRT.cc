
#include <velodyne_pointcloud/streaming_cloudXYZIRT.h>

namespace velodyne_pointcloud
{
StreamingCloudXYZIRT::StreamingCloudXYZIRT(ros::Publisher* pub,
                                           const std::string& sensor_frame,
                                           const unsigned int num_lasers)
    : DataContainerBase(0,
                        0,
                        "",
                        "",
                        1,
                        num_lasers,
                        true,
                        0,
                        8,
                        "x",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "y",
                        1,
                        sensor_msgs::PointField::FLOAT32,
                        "z",
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
      iter_intensity(cloud, "intensity"),
      iter_row_index(cloud, "row_index"),
      iter_firing_index(cloud, "firing_index"),
      iter_time_sec(cloud, "time_sec"),
      iter_time_nsec(cloud, "time_nsec"),
      pub(pub)
{
    cloud.header.frame_id = sensor_frame;
    cloud.width = 1;
    cloud.height = num_lasers;
    cloud.is_dense = true;
    cloud.data.resize(num_lasers * cloud.point_step);

    // the iterators seem to require some parameters above, therefore they have to be initialized again
    iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
    iter_row_index = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "row_index");
    iter_firing_index = sensor_msgs::PointCloud2Iterator<uint32_t>(cloud, "firing_index");
    iter_time_sec = sensor_msgs::PointCloud2Iterator<uint32_t>(cloud, "time_sec");
    iter_time_nsec = sensor_msgs::PointCloud2Iterator<uint32_t>(cloud, "time_nsec");
}

void StreamingCloudXYZIRT::newLine()
{
    cloud.header.stamp.sec = *(iter_time_sec);   // use stamp of topmost laser
    cloud.header.stamp.nsec = *(iter_time_nsec); // use stamp of topmost laser

    pub->publish(cloud);

    firing_index++;
}

void StreamingCloudXYZIRT::setPacketStamp(ros::Time packet_stamp)
{
    cur_packet_stamp = packet_stamp;
}

void StreamingCloudXYZIRT::addPoint(float x,
                                    float y,
                                    float z,
                                    const uint16_t ring,
                                    const uint16_t /*azimuth*/,
                                    const float distance,
                                    const float intensity,
                                    const float time)
{
    // calculate time
    ros::Duration duration_in_packet;
    ros::Time stamp = cur_packet_stamp + duration_in_packet.fromSec(time);

    uint16_t row_index = cloud.height - ring - 1;
    *(iter_row_index + row_index) = row_index;
    *(iter_firing_index + row_index) = firing_index;
    *(iter_time_sec + row_index) = stamp.sec;
    *(iter_time_nsec + row_index) = stamp.nsec;

    if (distance > 0)
    {
        *(iter_x + row_index) = x;
        *(iter_y + row_index) = y;
        *(iter_z + row_index) = z;
        *(iter_intensity + row_index) = intensity;
    }
    else
    {
        *(iter_x + row_index) = nanf("");
        *(iter_y + row_index) = nanf("");
        *(iter_z + row_index) = nanf("");
        *(iter_intensity + row_index) = 0;
    }
}
} // namespace velodyne_pointcloud
