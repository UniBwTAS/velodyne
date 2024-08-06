//
// Created by jugo on 05.08.24.
//
#include <ros/ros.h>
#include "velodyne_packet_pointcloud/unpacker.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "velodyne_packet_pointcloud");

    // create conversion class, which subscribes to raw data
    velodyne_packet_pointcloud::Unpacker unpacker(ros::NodeHandle(),
                                             ros::NodeHandle("~"));

    // handle callbacks until shut down
    ros::spin();

    return 0;

}