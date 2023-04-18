#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <velodyne_pointcloud/ego_motion_correction.h>

namespace velodyne_pointcloud
{

EgoMotionCorrection::EgoMotionCorrection(ros::NodeHandle nh, const ros::NodeHandle& nh_private)
    : tf_listener_(tf_buffer_, true, ros::TransportHints().tcp().tcpNoDelay())
{
    nh_private.param("num_threads", num_threads_, 4);
    thread_pool_.init(num_threads_);

    nh_private.param<std::string>("fixed_frame", fixed_frame_, "odom");
    nh_private.param<float>("time_range_full_rotation", time_range_full_rotation_, 0.1f);
    nh_private.param<float>("time_range_for_same_tf", time_range_for_same_tf_, 0.1f / 36);
    nh_private.param<std::string>("point_stamp_key", point_stamp_key_, "time");
    nh_private.param<bool>("point_stamp_in_ns", point_stamp_in_ns_, false);

    pc_sub_ = nh.subscribe("velodyne_points", 5, &EgoMotionCorrection::callbackPointCloud, this);
    pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points_corrected", 5);
}

EgoMotionCorrection::~EgoMotionCorrection()
{
    thread_pool_.shutdown();
}

void EgoMotionCorrection::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (pc_pub_.getNumSubscribers() == 0)
        return;

    auto exec_start_total = std::chrono::steady_clock::now();

    input_msg_ = msg;
    num_firings_ = static_cast<int>(msg->height);
    num_rings_ = static_cast<int>(msg->width);

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
    ROS_INFO_STREAM_NAMED(logger_name, "Time look_up: " << std::fixed << exec_dur_look_up.count() << std::setprecision(5) );

    // copy untransformed message (allocate new shared pointers for zero-copy sharing with other nodelets)
    output_msg_.reset(new sensor_msgs::PointCloud2);
    output_msg_->header.stamp = msg->header.stamp;
    output_msg_->height = msg->height;
    output_msg_->width = msg->width;
    output_msg_->is_bigendian = msg->is_bigendian;
    output_msg_->point_step = msg->point_step + (3 * sizeof(float));
    output_msg_->row_step = output_msg_->width * output_msg_->point_step;
    output_msg_->is_dense = msg->is_dense;

    // set new frame id
    output_msg_->header.frame_id = fixed_frame_;

    // copy fields
    output_msg_->fields = msg->fields;

    // append three more fields for original x, y, z values
    std::string names[] = {"x_sensor", "y_sensor", "z_sensor"};
    sensor_msgs::PointField pf;
    pf.count = 1;
    pf.datatype = sensor_msgs::PointField::FLOAT32;
    pf.offset = msg->point_step;
    for (auto& name : names)
    {
        pf.name = name;
        output_msg_->fields.push_back(pf);
        pf.offset += sizeof(float);
    }

    // allocate memory
    output_msg_->data.resize(num_firings_ * num_rings_ * output_msg_->point_step);

    // transform and fill message
    auto exec_start_transformAndFill = std::chrono::steady_clock::now();
    thread_pool_.shareWork(num_firings_, &EgoMotionCorrection::transformAndFill, this);
    std::chrono::duration<double> exec_dur_transformAndFill = std::chrono::steady_clock::now() - exec_start_transformAndFill;
    ROS_INFO_STREAM_NAMED(logger_name, "Time transformAndFill: " << std::fixed << exec_dur_transformAndFill.count() << std::setprecision(5));

    pc_pub_.publish(output_msg_);

    std::chrono::duration<double> exec_dur_total = std::chrono::steady_clock::now() - exec_start_total;
    ROS_INFO_STREAM_NAMED(logger_name, "Time total: " << std::fixed << exec_dur_total.count() << std::setprecision(5) );
}

void EgoMotionCorrection::transformAndFill(int start_firing_idx, int end_firing_idx)
{

    // get output iterators
    sensor_msgs::PointCloud2Iterator<float> iter_x_out(*output_msg_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y_out(*output_msg_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z_out(*output_msg_, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_time_out(*output_msg_, point_stamp_key_);
    sensor_msgs::PointCloud2Iterator<float> iter_x_sensor_out(*output_msg_, "x_sensor");
    sensor_msgs::PointCloud2Iterator<float> iter_y_sensor_out(*output_msg_, "y_sensor");
    sensor_msgs::PointCloud2Iterator<float> iter_z_sensor_out(*output_msg_, "z_sensor");

    // set iterators and indices according to start firing index
    iter_x_out += start_firing_idx * num_rings_;
    iter_y_out += start_firing_idx * num_rings_;
    iter_z_out += start_firing_idx * num_rings_;
    iter_time_out += start_firing_idx * num_rings_;
    iter_x_sensor_out += start_firing_idx * num_rings_;
    iter_y_sensor_out += start_firing_idx * num_rings_;
    iter_z_sensor_out += start_firing_idx * num_rings_;
    unsigned offset_src = start_firing_idx * input_msg_->row_step;
    unsigned offset_dst = start_firing_idx * output_msg_->row_step;

    // transform point cloud and copy remaining data
    for (int firing_idx = start_firing_idx; firing_idx < end_firing_idx; ++firing_idx)
    {
        for (int ring_idx = 0; ring_idx < num_rings_; ++ring_idx)
        {
            // copy existing data for this point
            std::memcpy(&output_msg_->data[offset_dst], &input_msg_->data[offset_src], input_msg_->point_step);

            // transform point and assign it
            const float x = *iter_x_out;
            const float y = *iter_y_out;
            const float z = *iter_z_out;
            float dt = *iter_time_out;
            if (point_stamp_in_ns_)
              dt = static_cast<float>(*reinterpret_cast<uint32_t *>(&dt) / 1e9);

            if (!std::isnan(x))
            {
                int tf_idx = std::floor(dt / time_range_for_same_tf_);
                if (tf_idx < 0)
                {
                    tf_idx = 0;
                }
                else if (tf_idx >= lut_tfs.size())
                {
                    ROS_WARN("No transform precalculated for this stamp!");
                    tf_idx = static_cast<int>(lut_tfs.size()) - 1;
                }
                const tf2::Transform t = lut_tfs[tf_idx];
                tf2::Vector3 v(x, y, z);

                v = t * v;

                *iter_x_out = static_cast<float>(v.x());
                *iter_y_out = static_cast<float>(v.y());
                *iter_z_out = static_cast<float>(v.z());
            }

            // store original values
            *iter_x_sensor_out = x;
            *iter_y_sensor_out = y;
            *iter_z_sensor_out = z;

            // increment iterators & indices
            ++iter_x_out;
            ++iter_y_out;
            ++iter_z_out;
            ++iter_time_out;
            ++iter_x_sensor_out;
            ++iter_y_sensor_out;
            ++iter_z_sensor_out;
            offset_src += input_msg_->point_step;
            offset_dst += output_msg_->point_step;
        }
    }
}

} // namespace velodyne_pointcloud