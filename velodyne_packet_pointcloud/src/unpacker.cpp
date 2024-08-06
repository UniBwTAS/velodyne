//
// Created by jugo on 05.08.24.
//

#include "velodyne_packet_pointcloud/unpacker.h"

namespace velodyne_packet_pointcloud
{

    Unpacker::Unpacker(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const &node_name)
    {


        if (calibration)
        {
            ROS_DEBUG_STREAM("Calibration file loaded.");
            config_.num_lasers = static_cast<uint16_t>(calibration.get().num_lasers);
        }
        else
        {
            ROS_ERROR_STREAM("Could not load calibration file!");
        }

        if (!private_nh.getParam("input_ethernet_msgs_topic", input_ethernet_msgs_topic))
        {
            input_ethernet_msgs_topic = "";
        }
        ROS_WARN("Using %s as ethernet packages topic", input_ethernet_msgs_topic.c_str());
        if (! private_nh.getParam("rpm", _rpm))
        {
            _rpm = 600.0;
            ROS_ERROR("No Velodyne RPM specified using default %f!", _rpm);
        }

        output_ = node.advertise<velodyne_msgs::VelodynePacketPointCloud>("velodyne_packets_points", 10);
        velodyne_ethernet_msgs_ = node.subscribe(input_ethernet_msgs_topic, 10 * 604, &Unpacker::processEthernetMsgs,
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
        uint8_t mode = 0; // 55 .. 58
        std::memcpy(&mode, &ethernet_msg->payload[ret_mode_data_pos], velodyne_rawdata::RETURN_MODE_BYTE_SIZE);


        const double frequency = (_rpm / 60.0);
        int npackets = 0;
        if (mode == velodyne_rawdata::VLS128_RETURN_MODE_STRONGEST || mode == velodyne_rawdata::VLS128_RETURN_MODE_LAST)
            npackets = (int) ceil(velodyne_rawdata::PACKET_RATE_SINGLE_RET_MODE / frequency); // packets per rev
        else
            npackets = (int) ceil(velodyne_rawdata::PACKET_RATE_DUAL_RET_MODE / frequency); // packets per rev



        velodyne_msgs::VelodynePacket tmp_packet;
        std::memcpy(&tmp_packet.data, &ethernet_msg->payload[0], ethernet_msg->payload.size());
        tmp_packet.stamp = ethernet_msg->header.stamp;

            processPacket(tmp_packet, mode);




    }


    void
    Unpacker::processPacket(const velodyne_msgs::VelodynePacket &packetMsg, const uint8_t mode)
    {
        if (output_.getNumSubscribers() == 0)      // no one listening?
            return;                                     // avoid much work


        const int return_mode = mode;

        container_ptr->set_return_mode(return_mode);

        // allocate a point cloud with same time and frame ID as raw data
        container_ptr->setup(scanMsg);

        // sufficient to calculate single transform for whole scan
        if (!container_ptr->computeTransformToTarget(scanMsg->header.stamp))
        {
            // target frame not available
            return;
        }

        // process each packet provided by the driver
        for (size_t i = 0; i < scanMsg->packets.size(); ++i)
        {
            // calculate individual transform for each packet to account for ego
            // during one rotation of the velodyne sensor
            if (!container_ptr->computeTransformToFixed(scanMsg->packets[i].stamp))
            {
                // fixed frame not available
                return;
            }

            raw_data_ptr_->unpack(scanMsg->packets[i], *container_ptr,
                                  scanMsg->header.stamp, i);
        }
        // publish the accumulated cloud message
        output_.publish(container_ptr->finishCloud(scanMsg->header.stamp));

        // publish return mode only if it has changed
        velodyne_msgs::VelodyneReturnMode retmode_msg;
        if (container_ptr->get_return_mode(retmode_msg))
            output_ret_mode_.publish(retmode_msg);

        diag_topic_->tick(scanMsg->header.stamp);
        diagnostics_.update();
    }

}