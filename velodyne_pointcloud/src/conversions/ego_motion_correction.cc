#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <velodyne_pointcloud/ego_motion_correction.h>

namespace velodyne_pointcloud
{

EgoMotionCorrection::EgoMotionCorrection(ros::NodeHandle nh, const ros::NodeHandle& nh_private)
    : tf_listener_(tf_buffer_)
{
    nh_private.param<std::string>("fixed_frame", fixed_frame_, "odom");
    nh_private.param<float>("time_range_full_rotation", time_range_full_rotation_, 0.1f);
    nh_private.param<float>("time_range_for_same_tf", time_range_for_same_tf_, 0.1f / 36);

    pc_sub_ = nh.subscribe("velodyne_points", 5, &EgoMotionCorrection::callbackPointCloud, this);
    pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points_corrected", 5);
}

void EgoMotionCorrection::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (pc_pub_.getNumSubscribers() == 0)
        return;

    auto exec_start_total = std::chrono::steady_clock::now();

    // prepare look up table for transforms
    auto exec_start_look_up = std::chrono::steady_clock::now();
    lut_tfs.clear();
    int num_entries = static_cast<int>(std::ceil(time_range_full_rotation_ / time_range_for_same_tf_)) + 1;
    for (int i = 0; i < num_entries; ++i)
    {
        try
        {
            tf2::Transform t;
            ros::Time stamp = msg->header.stamp + ros::Duration(static_cast<float>(i) * time_range_for_same_tf_);
            auto raw = tf_buffer_.lookupTransform(fixed_frame_, msg->header.frame_id, stamp, ros::Duration(0.3));
            fromMsg(raw.transform, t);
            lut_tfs.push_back(t);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
    }
    std::chrono::duration<double> exec_dur_look_up = std::chrono::steady_clock::now() - exec_start_look_up;
    std::cout << "Time look_up: " << std::fixed << exec_dur_look_up.count() << std::setprecision(5) << std::endl;

    // copy untransformed message (allocate new shared pointers for zero-copy sharing with other nodelets)
    sensor_msgs::PointCloud2Ptr pc(new sensor_msgs::PointCloud2);
    pc->header.stamp = msg->header.stamp;
    pc->height = msg->height;
    pc->width = msg->width;
    pc->is_bigendian = msg->is_bigendian;
    pc->point_step = msg->point_step;
    pc->row_step = msg->row_step;
    pc->is_dense = msg->is_dense;

    // copy fields
    for (const sensor_msgs::PointField& pf : msg->fields)
        pc->fields.push_back(pf);

    // copy data
    pc->data.resize(msg->data.size());
    std::memcpy(&pc->data[0], &msg->data[0], msg->data.size());

    // set new frame id
    pc->header.frame_id = fixed_frame_;

    // get iterators to x, y, z
    sensor_msgs::PointCloud2Iterator<float> iter_x_out(*pc, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y_out(*pc, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z_out(*pc, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_time_out(*pc, "time");

    // transform point cloud
    for (; iter_x_out != iter_x_out.end(); ++iter_x_out, ++iter_y_out, ++iter_z_out, ++iter_time_out)
    {
        float x = *iter_x_out;
        float y = *iter_y_out;
        float z = *iter_z_out;
        float dt = *iter_time_out;

        int tf_idx = std::floor(dt / time_range_for_same_tf_);
        if (tf_idx < 0)
        {
            tf_idx = 0;
        }
        else if (tf_idx >= lut_tfs.size())
        {
            ROS_WARN("No transform precalculated for this stamp!");
            continue;
        }
        tf2::Transform t = lut_tfs[tf_idx];
        tf2::Vector3 v(x, y, z);

        v = t * v;

        *iter_x_out = static_cast<float>(v.x());
        *iter_y_out = static_cast<float>(v.y());
        *iter_z_out = static_cast<float>(v.z());
    }

    pc_pub_.publish(pc);

    std::chrono::duration<double> exec_dur_total = std::chrono::steady_clock::now() - exec_start_total;
    std::cout << "Time total: " << std::fixed << exec_dur_total.count() << std::setprecision(5) << std::endl;
}

} // namespace velodyne_pointcloud
