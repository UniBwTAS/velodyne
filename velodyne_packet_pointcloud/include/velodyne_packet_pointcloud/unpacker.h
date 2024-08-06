//
// Created by jugo on 05.08.24.
//

#ifndef VELODYNE_POINTCLOUD_UNPACKER_H
#define VELODYNE_POINTCLOUD_UNPACKER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ethernet_msgs/Packet.h>
#include <velodyne_msgs/VelodyneReturnMode.h>
#include <velodyne_msgs/VelodynePacketPointCloud.h>
#include <velodyne_pointcloud/calibration.h>
#include <velodyne_pointcloud/datacontainerbase.h>
#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_packet_pointcloud
{

    static const std::string XYZIRT_TYPE = "XYZIRT";
    static const std::string ORGANIZED_TYPE = "ORGANIZED";
    static const std::string EXTENDED_TYPE = "EXTENDED";
    static const std::string EXTENDEDCONF_TYPE = "EXTENDEDCONF";

    class Unpacker
    {
    public:
        Unpacker(
                ros::NodeHandle node,
                ros::NodeHandle private_nh,
                std::string const &node_name = ros::this_node::getName());

        ~Unpacker()
        {
        }

    private:

        void processEthernetMsgs(const ethernet_msgs::PacketConstPtr &ethernet_msg);
        void processPacket(const velodyne_msgs::VelodynePacket &packetMsg,uint8_t mode);
        ros::Publisher output_;
        ros::Subscriber velodyne_ethernet_msgs_;
        std::string input_ethernet_msgs_topic; ///< topic name for ethernet_msgs
        double _rpm{600.0};

    };


}
#endif //VELODYNE_POINTCLOUD_UNPACKER_H
