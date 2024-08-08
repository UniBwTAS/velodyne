//
// Created by jugo on 05.08.24.
//
#include <ros/ros.h>
#include "velodyne_packet_pointcloud/unpacker.h"
#include <ros/package.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "velodyne_packet_pointcloud");

    auto nh = ros::NodeHandle();
    auto private_nh = ros::NodeHandle("~");
    std::string sensor_frame;
    std::string target_frame;  ///< output frame of final point cloud
    std::string fixed_frame;   ///< world fixed frame for ego motion compenstation
    std::string cloud_type;    ///< selects the type of point cloud to use as output
    std::string input_ethernet_msgs_topic; ///< topic name for ethernet_msgs
    std::string model;
    double rpm;
    float max_range;
    float min_range;
    std::string calibrationFile;  ///< calibration file name

    if (!private_nh.getParam("model", model))
    {
        model =  std::string("64E");
        ROS_ERROR("No Velodyne Sensor Model specified using default %s!", model.c_str());

    }
    if (! private_nh.getParam("rpm", rpm))
    {
        rpm = 600.0;
        ROS_ERROR("No Velodyne RPM specified using default %f!", rpm);
    }


    // get path to angles.config file for this device
    if (!private_nh.getParam("calibration", calibrationFile))
    {
        ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

        // have to use something: grab unit test version as a default
        std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
        calibrationFile = pkgPath + "/params/64e_utexas.yaml";
    }


    double _rpm{600.0};

    if (!private_nh.getParam("input_ethernet_msgs_topic", input_ethernet_msgs_topic))
    {
        input_ethernet_msgs_topic = "";
    }
    ROS_WARN("Using %s as ethernet packages topic", input_ethernet_msgs_topic.c_str());
    if (!private_nh.getParam("rpm", _rpm))
    {
        _rpm = 600.0;
        ROS_ERROR("No Velodyne RPM specified using default %f!", _rpm);
    }

    if (!private_nh.getParam("sensor_frame", sensor_frame))
    {
        sensor_frame = "sensor/lidar/vls128_roof";
        ROS_ERROR("No sensor_frame specified using default %s!", sensor_frame.c_str());
    }
    if (!private_nh.getParam("target_frame", target_frame))
    {
        target_frame = "";
        ROS_ERROR("No target_frame specified using default %s!", target_frame.c_str());
    }
    if (!private_nh.getParam("fixed_frame", fixed_frame))
    {
        fixed_frame = "";
        ROS_ERROR("No fixed_frame specified using default %s!", fixed_frame.c_str());
    }
    if (!private_nh.getParam("max_range", max_range))
    {
        max_range = 300.0;
        ROS_ERROR("No max_range specified using default %f!", max_range);
    }
    if (!private_nh.getParam("min_range", min_range))
    {
        min_range = 2.0;
        ROS_ERROR("No min_range specified using default %f!", min_range);
    }

    if (!private_nh.getParam("cloud_type", cloud_type))
    {
        cloud_type = velodyne_packet_pointcloud::Unpacker::EXTENDED_TYPE;
        ROS_ERROR("No cloud type specified using default %s!", cloud_type.c_str());
    }

    // create conversion class, which subscribes to raw data
    std::unique_ptr<velodyne_rawdata::DataContainerBase> ptr_unpacker;

    if (cloud_type == velodyne_packet_pointcloud::Unpacker::XYZIRT_TYPE)
    {
        // Todo implement unpacker for this kind of cloud
        return 0;
    }
    else
    {
        if (cloud_type == velodyne_packet_pointcloud::Unpacker::EXTENDED_TYPE)
        {
            ptr_unpacker = std::make_unique<velodyne_packet_pointcloud::Unpacker>
                    (nh, private_nh, input_ethernet_msgs_topic,
                     max_range,
                     min_range,
                     target_frame,
                     fixed_frame,
                     sensor_frame,
                     1, // determines the width of the cloud
                     0); // points per packet (gets overwritten once there is data)
        }
        else
        {
            if (cloud_type == velodyne_packet_pointcloud::Unpacker::EXTENDEDCONF_TYPE)
            {
                // Todo implement unpacker for this kind of cloud
                return 0;
            }
            else
            {
                ptr_unpacker = std::make_unique<velodyne_packet_pointcloud::Unpacker>
                        (nh, private_nh, input_ethernet_msgs_topic,
                         max_range,
                         min_range,
                         target_frame,
                         fixed_frame,
                         sensor_frame,
                         1, // determines the width of the cloud
                         0); // points per packet (gets overwritten once there is data)

            }
        }
    }

    // handle callbacks until shut down
    ros::spin();

    return 0;

}