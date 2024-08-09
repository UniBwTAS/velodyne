//
// Created by jugo on 05.08.24.
//
#include <ros/ros.h>
#include <ros/package.h>
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
        std::unique_ptr<velodyne_rawdata::DataContainerBase> unpacker;
    };

    /** @brief Nodelet initialization. */
    void PacketpcNodelet::onInit()
    {

        auto nh = getNodeHandle();
        auto private_nh = getPrivateNodeHandle();
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
            ROS_ERROR("PacketpcNodelet No Velodyne Sensor Model specified using default %s!", model.c_str());

        }
        if (! private_nh.getParam("rpm", rpm))
        {
            rpm = 600.0;
            ROS_ERROR("PacketpcNodelet No Velodyne RPM specified using default %f!", rpm);
        }


        // get path to angles.config file for this device
        if (!private_nh.getParam("calibration", calibrationFile))
        {
            ROS_ERROR_STREAM("PacketpcNodelet No calibration angles specified! Using test values!");

            // have to use something: grab unit test version as a default
            std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
            calibrationFile = pkgPath + "/params/64e_utexas.yaml";
        }


        double _rpm{600.0};

        if (!private_nh.getParam("input_ethernet_msgs_topic", input_ethernet_msgs_topic))
        {
            ROS_ERROR("PacketpcNodelet No Velodyne input_ethernet_msgs_topic specified using default %f!", _rpm);
            input_ethernet_msgs_topic = "";
        }
        ROS_WARN("PacketpcNodelet Using %s as ethernet packages topic", input_ethernet_msgs_topic.c_str());
        if (!private_nh.getParam("rpm", _rpm))
        {
            _rpm = 600.0;
            ROS_ERROR("PacketpcNodelet No Velodyne RPM specified using default %f!", _rpm);
        }

        if (!private_nh.getParam("sensor_frame", sensor_frame))
        {
            sensor_frame = "sensor/lidar/vls128_roof";
            ROS_ERROR("PacketpcNodelet No sensor_frame specified using default %s!", sensor_frame.c_str());
        }
        if (!private_nh.getParam("target_frame", target_frame))
        {
            target_frame = "";
            ROS_ERROR("PacketpcNodelet No target_frame specified using default %s!", target_frame.c_str());
        }
        if (!private_nh.getParam("fixed_frame", fixed_frame))
        {
            fixed_frame = "";
            ROS_ERROR("PacketpcNodelet No fixed_frame specified using default %s!", fixed_frame.c_str());
        }
        if (!private_nh.getParam("max_range", max_range))
        {
            max_range = 300.0;
            ROS_ERROR("PacketpcNodelet No max_range specified using default %f!", max_range);
        }
        if (!private_nh.getParam("min_range", min_range))
        {
            min_range = 2.0;
            ROS_ERROR("PacketpcNodelet No min_range specified using default %f!", min_range);
        }

        if (!private_nh.getParam("cloud_type", cloud_type))
        {
            cloud_type = velodyne_packet_pointcloud::Unpacker::EXTENDED_TYPE;
            ROS_ERROR("PacketpcNodelet No cloud type specified using default %s!", cloud_type.c_str());
        }



        if (cloud_type == velodyne_packet_pointcloud::Unpacker::XYZIRT_TYPE)
        {
            // Todo implement unpacker for this kind of cloud
        }
        else
        {
            if (cloud_type == velodyne_packet_pointcloud::Unpacker::EXTENDED_TYPE)
            {

                unpacker = std::make_unique<velodyne_packet_pointcloud::Unpacker>
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

                }
                else
                {
                    unpacker = std::make_unique<velodyne_packet_pointcloud::Unpacker>
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

    }

} // namespace velodyne_pointcloud


// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(velodyne_packet_pointcloud::PacketpcNodelet, nodelet::Nodelet)