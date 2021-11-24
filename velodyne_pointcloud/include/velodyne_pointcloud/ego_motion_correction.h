#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>

namespace velodyne_pointcloud {

struct Point {
  float x;
  float y;
  float z;
  float range;
  float azimuth;
  int ring_idx;
  int azimuth_idx;
};

class EgoMotionCorrection {

public:
  EgoMotionCorrection(ros::NodeHandle nh, const ros::NodeHandle &nh_private);

private:
  void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);

private:
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<tf2::Transform> lut_tfs;

  std::string fixed_frame_;
  float time_range_full_rotation_{};
  float time_range_for_same_tf_{};
};
} // namespace velodyne_pointcloud
