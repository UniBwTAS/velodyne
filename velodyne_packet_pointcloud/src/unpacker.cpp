//
// Created by jugo on 05.08.24.
//

#include "velodyne_packet_pointcloud/unpacker.h"
#include <cstdarg>

namespace velodyne_packet_pointcloud
{
    const std::string Unpacker::XYZIRT_TYPE = "XYZIRT";
    const std::string Unpacker::EXTENDED_TYPE = "EXTENDED";
    const std::string Unpacker::EXTENDEDCONF_TYPE = "EXTENDEDCONF";

    // constructor for EXTENDED_TYPE
    Unpacker::Unpacker(ros::NodeHandle &nh, ros::NodeHandle &private_nh, const std::string ethernet_msgs_topic,
                       const double max_range,
                       const double min_range,
                       const std::string &target_frame, const std::string &fixed_frame, const std::string &sensor_frame,
                       const unsigned int num_lasers, // determines the width of the cloud
                       const unsigned int points_per_packet) :
            DataContainerBase(max_range, min_range, target_frame, fixed_frame,
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
                              "first_return_flag", 1, sensor_msgs::PointField::UINT8),
            iter_x(cloud, "x"),
            iter_y(cloud, "y"),
            iter_z(cloud, "z"),
            iter_distance(cloud, "distance"),
            iter_time(cloud, "time"),
            iter_sub_segment(cloud, "sub_segment"),
            iter_rotation_segment(cloud, "rotation_segment"),
            iter_azimuth(cloud, "azimuth"),
            iter_firing_bin(cloud, "firing_bin"),
            iter_ring(cloud, "ring"),
            iter_intensity(cloud, "intensity"),
            iter_laser_id(cloud, "laser_id"),
            iter_first_ret(cloud, "first_return_flag"),
            raw_data_ptr_(std::make_shared<velodyne_rawdata::RawData>())
    {
        this->sensor_frame = sensor_frame;

        boost::optional<velodyne_pointcloud::Calibration> calibration = raw_data_ptr_->setup(private_nh);
        model = raw_data_ptr_->get_sensor_model();
        raw_data_ptr_->setParameters(min_range, max_range,
                                     0.0, M_PI * 2.0);


        start_listening(nh, ethernet_msgs_topic);
    }

    void Unpacker::start_listening(ros::NodeHandle &nh, const std::string ethernet_msgs_topic)
    {
        output_ = nh.advertise<sensor_msgs::PointCloud2>("velodyne_packets_points", 10);
        output_ret_mode_ = nh.advertise<velodyne_msgs::VelodyneReturnMode>("velodyne_return_mode", 10, true);
        velodyne_ethernet_msgs_ = nh.subscribe(ethernet_msgs_topic, 10 * 604, &Unpacker::processEthernetMsgs,
                                               this);
    }

    void Unpacker::processEthernetMsgs(const ethernet_msgs::PacketConstPtr &ethernet_msg)
    {
        if (output_.getNumSubscribers() == 0)      // no one listening?
            return;                                  // avoid much work

        // a packet contains data from 3 azimuth angles  use the biggest (last four blocks) in single ret in double all the azimuths are the same
        std::size_t azimuth_data_pos = velodyne_rawdata::BLOCK_SIZE * 8 + velodyne_rawdata::FLAG_SIZE;
        uint16_t azimuth = 0; // 0...36000
        std::memcpy(&azimuth, &ethernet_msg->payload[azimuth_data_pos], velodyne_rawdata::AZIMUTH_SIZE);


        // get if the packet is single or double return

        std::size_t ret_mode_data_pos =
                velodyne_rawdata::BLOCK_SIZE * velodyne_rawdata::BLOCKS_PER_PACKET + velodyne_rawdata::TIMESTAMP_SIZE;
        uint8_t return_mode = 0; // 55 .. 58
        std::memcpy(&return_mode, &ethernet_msg->payload[ret_mode_data_pos], velodyne_rawdata::RETURN_MODE_BYTE_SIZE);

        if (return_mode == velodyne_rawdata::VLS128_RETURN_MODE_STRONGEST ||
            return_mode == velodyne_rawdata::VLS128_RETURN_MODE_LAST)
        {
            points_per_packet = 12 * 32;
        }
        else
        {
            // Todo: This is only valid for the vls128
            points_per_packet = (12 - 4) * 32; // 8 of 12 blocks have point data in them
        }

        // create a packet message
        velodyne_msgs::VelodynePacket tmp_packet;
        std::memcpy(&tmp_packet.data, &ethernet_msg->payload[0], ethernet_msg->payload.size());
        tmp_packet.stamp = ethernet_msg->header.stamp;

        processPacket(tmp_packet, return_mode);

    }


    void
    Unpacker::processPacket(const velodyne_msgs::VelodynePacket &packetMsg, const uint8_t return_mode)
    {
        // contents of the setup function in base class ( no scan message to pas to this function)
        set_return_mode(return_mode);


        manage_tf_buffer();
        packets_in_scan = 1; // must be ste to at least one
        cloud.header.stamp = packetMsg.stamp;
        cloud.data.clear();
        cloud.data.resize(points_per_packet * cloud.point_step);
        cloud.width = 1;
        cloud.height = 0;// starts at 0 and increases with each succes full call to add point
        cloud.is_dense = false;

        iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
        iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
        iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
        iter_distance = sensor_msgs::PointCloud2Iterator<float>(cloud, "distance");
        iter_time = sensor_msgs::PointCloud2Iterator<float>(cloud, "time");
        iter_sub_segment = sensor_msgs::PointCloud2Iterator<uint32_t>(cloud, "sub_segment");
        iter_azimuth = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "azimuth");
        iter_rotation_segment = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "rotation_segment");
        iter_firing_bin = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "firing_bin");
        iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "ring");
        iter_intensity = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud, "intensity");
        iter_laser_id = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud, "laser_id");
        iter_first_ret = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud, "first_return_flag");


        if (!computeTransformToTarget(packetMsg.stamp))
        {
            // target frame not available
            return;
        }
        if (!computeTransformToFixed(packetMsg.stamp))
        {
            // fixed frame not available
            return;
        }
        ROS_DEBUG_STREAM("Unpacking" << cloud.height * cloud.width
                                      << " Velodyne points, time: " << cloud.header.stamp);

        raw_data_ptr_->unpack(packetMsg, *this,packetMsg.stamp,
                              0); // todo: this may have to be a running number
        ROS_DEBUG_STREAM("Publishing" << cloud.height * cloud.width
                                      << " Velodyne points, time: " << cloud.header.stamp);


        output_.publish(this->finishCloud(packetMsg.stamp));

        // publish return mode only if it has changed
        velodyne_msgs::VelodyneReturnMode retmode_msg;
        if (this->get_return_mode(retmode_msg))
            output_ret_mode_.publish(retmode_msg);


    }

    void
    Unpacker::addPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth, const float distance,
                       const float intensity, const float time)
    {
        uint64_t offset = cloud.height * config_.init_width;
        if (!pointInRange(distance))
        {
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
        }
        else
        {
            *(iter_x + offset) = std::numeric_limits<float>::quiet_NaN();
            *(iter_y + offset) = std::numeric_limits<float>::quiet_NaN();
            *(iter_z + offset) = std::numeric_limits<float>::quiet_NaN();
            *(iter_distance + offset) = std::numeric_limits<float>::quiet_NaN();
            *(iter_time + offset) = time;
            *(iter_sub_segment + offset) = 0;
            *(iter_azimuth + offset) = azimuth;
            *(iter_rotation_segment + offset) = 0;
            *(iter_firing_bin + offset) = 0;
            *(iter_intensity + offset) = static_cast<uint8_t>(intensity);
            *(iter_ring + offset) = ring;
            *(iter_laser_id + offset) = 0;
            *(iter_first_ret + offset) = 0;
        }
        ++cloud.height;
    }

    void
    Unpacker::addPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth, const float distance,
                       const float intensity, const float time, const uint32_t sub_segment,
                       const uint16_t rotation_segment, const uint16_t firing_bin, const uint8_t laser_id,
                       const uint8_t first_return_flag)
    {

        const uint64_t offset = cloud.height;

        if (pointInRange(distance))
        {
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
        }
        else
        {

            *(iter_x + offset) = std::numeric_limits<float>::quiet_NaN();
            *(iter_y + offset) = std::numeric_limits<float>::quiet_NaN();
            *(iter_z + offset) = std::numeric_limits<float>::quiet_NaN();
            *(iter_distance + offset) = std::numeric_limits<float>::quiet_NaN();
            *(iter_time + offset) = time;
            *(iter_sub_segment + offset) = sub_segment;
            *(iter_azimuth + offset) = azimuth;
            *(iter_rotation_segment + offset) = rotation_segment;
            *(iter_firing_bin + offset) = firing_bin;
            *(iter_intensity + offset) = static_cast<uint8_t>(intensity);
            *(iter_ring + offset) = ring;
            *(iter_laser_id + offset) = laser_id;
            *(iter_first_ret + offset) = first_return_flag;


        }

        ++cloud.height;

    }

    void Unpacker::addPoint_with_confidence(float x, float y, float z, const uint16_t ring, const uint16_t azimuth,
                                            const float distance, const float intensity, const float time,
                                            const uint32_t sub_segment, const uint16_t rotation_segment,
                                            const uint16_t firing_bin, const uint8_t laser_id,
                                            const uint8_t first_return_flag, const uint8_t drop,
                                            const uint8_t retro_shadow, const uint8_t range_limited,
                                            const uint8_t retro_ghost, const uint8_t interference,
                                            const uint8_t sun_lvl, const uint8_t confidence)
    {
        (void)drop;
        (void)retro_shadow;
        (void)range_limited;
        (void)retro_ghost;
        (void)interference;
        (void)sun_lvl;
        (void)confidence;

        ROS_WARN_STREAM_THROTTLE(60,
                                 "Recived packet with dual confidence return mode, but configured point cloud is EXTENDED,"
                                 "Use EXTENDEDCONF to get the confidence information in the cloud");
        addPoint(x, y, z, ring, azimuth, distance, intensity, time, sub_segment, rotation_segment, firing_bin, laser_id,
                 first_return_flag);
    }


    void Unpacker::newLine()
    {
        // this function is not necessary in this case

    }


}