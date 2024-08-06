//
// Created by jugo on 05.08.24.
//
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "velodyne_packet_pointcloud/unpacker.h"

namespace velodyne_packet_pointcloud
{
    class PacketpcNodelet: public nodelet::Nodelet
    {
    public:

        PacketpcNodelet() {}
        ~PacketpcNodelet() {}

    private:

        virtual void onInit();
        boost::shared_ptr<Unpacker> unpacker;
    };

    /** @brief Nodelet initialization. */
    void PacketpcNodelet::onInit()
    {
        unpacker.reset(new Unpacker(getNodeHandle(), getPrivateNodeHandle(), getName()));
    }

} // namespace velodyne_pointcloud


// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(velodyne_packet_pointcloud::PacketpcNodelet, nodelet::Nodelet)