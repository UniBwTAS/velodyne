#include <ros/ros.h>

#include <velodyne_pointcloud/ego_motion_correction.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ego_motion_correction");

    velodyne_pointcloud::EgoMotionCorrection node(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();

    return 0;
}
