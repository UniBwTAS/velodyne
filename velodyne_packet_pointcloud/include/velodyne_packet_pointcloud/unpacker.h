//
// Created by jugo on 05.08.24.
//

#ifndef VELODYNE_POINTCLOUD_UNPACKER_H
#define VELODYNE_POINTCLOUD_UNPACKER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ethernet_msgs/Packet.h>
#include <velodyne_msgs/VelodyneReturnMode.h>
#include <velodyne_pointcloud/calibration.h>
#include <velodyne_pointcloud/datacontainerbase.h>
#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_packet_pointcloud
{ ;

    class Unpacker : public velodyne_rawdata::DataContainerBase
    {
    public:
        static const std::string XYZIRT_TYPE;
        static const std::string EXTENDED_TYPE;
        static const std::string EXTENDEDCONF_TYPE;

        Unpacker(ros::NodeHandle &nh, ros::NodeHandle &private_nh, const std::string ethernet_msgs_topic, double max_range, const double min_range,
                 const std::string &target_frame, const std::string &fixed_frame, const std::string &sensor_frame,
                 unsigned int num_lasers, // determines the width of the cloud
                 unsigned int points_per_packet);

        Unpacker() = delete;

        void start_listening(ros::NodeHandle &nh, std::string ethernet_msgs_topic);


        ~Unpacker() = default;

        void addPoint(float x, float y, float z, const uint16_t ring,
                              const uint16_t azimuth, const float distance,
                              const float intensity, const float time) override;

        void addPoint(float x, float y, float z, const uint16_t ring,
                      const uint16_t azimuth, const float distance,
                      const float intensity, const float time,
                      const uint32_t sub_segment, const uint16_t  rotation_segment,
                      const uint16_t  firing_bin, const uint8_t laser_id,
                      const uint8_t first_return_flag, const uint8_t last_return_flag) override;

        void addPoint_with_confidence(float x, float y, float z, const uint16_t ring,
                                      const uint16_t azimuth, const float distance,
                                      const float intensity, const float time,
                                      const uint32_t sub_segment,
                                      const uint16_t rotation_segment,
                                      const uint16_t firing_bin, const uint8_t laser_id,
                                      const uint8_t first_return_flag,
                                      const uint8_t last_return_flag,
                                      const uint8_t drop,
                                      const uint8_t retro_shadow,
                                      const uint8_t range_limited,
                                      const uint8_t retro_ghost,
                                      const uint8_t interference,
                                      const uint8_t sun_lvl,
                                      const uint8_t confidence) override;

        void newLine() override;

    private:

        void processEthernetMsgs(const ethernet_msgs::PacketConstPtr &ethernet_msg);
        void processPacket(const velodyne_msgs::VelodynePacket &packetMsg, uint8_t mode);


        ros::Publisher output_;
        ros::Publisher output_ret_mode_;
        ros::Subscriber velodyne_ethernet_msgs_;


        unsigned int points_per_packet;
        std::string model;
        std::shared_ptr<velodyne_rawdata::RawData> raw_data_ptr_;

        sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z,  iter_distance, iter_time;
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_sub_segment;
        sensor_msgs::PointCloud2Iterator<uint16_t> iter_rotation_segment, iter_azimuth, iter_firing_bin, iter_ring;
        sensor_msgs::PointCloud2Iterator<uint8_t>   iter_intensity, iter_laser_id, iter_first_ret, iter_last_ret;



    };


}
#endif //VELODYNE_POINTCLOUD_UNPACKER_H
