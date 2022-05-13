/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class transforms raw Velodyne 3D LIDAR packets to PointCloud2
    in the /map frame of reference.

    @author Jack O'Quin
    @author Jesse Vera
    @author Sebastian Pütz

*/

#include "velodyne_pointcloud/transform.h"

#include <velodyne_pointcloud/organized_cloudXYZIRT.h>
#include <velodyne_pointcloud/pointcloudXYZIRT.h>
#include <velodyne_pointcloud/pointcloud_extended.h>
#include <velodyne_pointcloud/pointcloud_extended_ng.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name):
    data_(new velodyne_rawdata::RawData),
    first_rcfg_call(true),
    diagnostics_(node, private_nh, node_name)
  {
    boost::optional<velodyne_pointcloud::Calibration> calibration = data_->setup(private_nh);
    if(calibration)
    {
      ROS_DEBUG_STREAM("Calibration file loaded.");
      config_.num_lasers = static_cast<uint16_t>(calibration.get().num_lasers);
    }
    else
    {
      ROS_ERROR_STREAM("Could not load calibration file!");
    }

    // advertise output point cloud (before subscribing to input data)
    output_ = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    srv_ = boost::make_shared<dynamic_reconfigure::Server<TransformNodeCfg>> (private_nh);
    dynamic_reconfigure::Server<TransformNodeCfg>::CallbackType f;
    f = boost::bind (&Transform::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);

    if(config_.input_ethernet_msgs_topic == "")
    {
        ROS_INFO_STREAM("subscribe to: velodyne_packets");
        velodyne_scan_ = node.subscribe("velodyne_packets", 10, &Transform::processScan, this);
    }
    else
    {
        ROS_INFO_STREAM("subscribe to: "<<config_.input_ethernet_msgs_topic);
        velodyne_ethernet_msgs_ = node.subscribe(config_.input_ethernet_msgs_topic, 10*604, &Transform::processEthernetMsgs, this,
                                                 ros::TransportHints().tcp().tcpNoDelay());
    }

    // Diagnostics
    diagnostics_.setHardwareID("Velodyne Transform");
    // Arbitrary frequencies since we don't know which RPM is used, and are only
    // concerned about monitoring the frequency.
    diag_min_freq_ = 2.0;
    diag_max_freq_ = 20.0;
    using namespace diagnostic_updater;
    diag_topic_.reset(new TopicDiagnostic("velodyne_points", diagnostics_,
                                          FrequencyStatusParam(&diag_min_freq_,
                                                               &diag_max_freq_,
                                                               0.1, 10),
                                          TimeStampStatusParam()));

  }

  void Transform::reconfigure_callback(
      velodyne_pointcloud::TransformNodeConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("Reconfigure request.");
    data_->setParameters(config.min_range, config.max_range,
                         config.view_direction, config.view_width);
    config_.sensor_frame = config.sensor_frame;
    config_.target_frame = config.target_frame;
    config_.fixed_frame = config.fixed_frame;
    ROS_INFO_STREAM("Sensor frame ID now: " << config_.sensor_frame);
    ROS_INFO_STREAM("Target frame ID now: " << config_.target_frame);
    ROS_INFO_STREAM("Fixed frame ID now: " << config_.fixed_frame);
    config_.min_range = config.min_range;
    config_.max_range = config.max_range;
    config_.input_ethernet_msgs_topic = config.input_ethernet_msgs_topic;

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);

    if(first_rcfg_call || config.cloud_type != config_.cloud_type){
      first_rcfg_call = false;
      config_.cloud_type = config.cloud_type;

      if(config_.cloud_type == ORGANIZED_TYPE)
      {
        ROS_INFO_STREAM("Using the organized cloud format...");
        container_ptr = boost::shared_ptr<OrganizedCloudXYZIRT>(
            new OrganizedCloudXYZIRT(config_.max_range, config_.min_range,
                                    config_.target_frame, config_.fixed_frame,
                                    config_.num_lasers, data_->scansPerPacket()));
      }
      else
      {
        if(config_.cloud_type == XYZIRT_TYPE) {
          ROS_INFO_STREAM("Using the XYZIRT cloud format...");
          container_ptr =
              boost::shared_ptr<PointcloudXYZIRT>(new PointcloudXYZIRT(
                  config_.max_range, config_.min_range, config_.target_frame,
                  config_.fixed_frame, data_->scansPerPacket()));
        }
        else
        {
          if(config_.cloud_type == EXTENDED_TYPE) {
            if(config_.num_lasers != 128)
            {
              ROS_ERROR_STREAM("Using the Extended cloud format with wrong model, only VLS128!");
            }
            ROS_INFO_STREAM("Using the Extended cloud format...");
            container_ptr =
                boost::shared_ptr<PointcloudExtended>(new PointcloudExtended(
                    config_.max_range, config_.min_range, config_.target_frame,
                    config_.fixed_frame, config_.num_lasers, data_->scansPerPacket()));
          }
          else
          {
            if(config_.cloud_type == EXTENDED_NG_TYPE) {
              ROS_INFO_STREAM("Using the Extended NG cloud format...");
              container_ptr =
                  boost::shared_ptr<PointcloudExtendedNg>(new PointcloudExtendedNg(
                      config_.max_range, config_.min_range, config_.target_frame,
                        config_.fixed_frame, config_.num_lasers, data_->scansPerPacket()));
            }
              else
              {
                ROS_ERROR("Wrong option in parameter cloud_type %s, using default type %s",
                          config_.cloud_type.c_str(), XYZIRT_TYPE.c_str());

                container_ptr =
                    boost::shared_ptr<PointcloudXYZIRT>(new PointcloudXYZIRT(
                        config_.max_range, config_.min_range, config_.target_frame,
                        config_.fixed_frame, data_->scansPerPacket()));
              }

          }

        }
      }

    }
    container_ptr->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);
  }

  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
    Transform::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)      // no one listening?
      return;                                     // avoid much work

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);

    // allocate a point cloud with same time and frame ID as raw data
    container_ptr->setup(scanMsg);

    // sufficient to calculate single transform for whole scan
    if(!container_ptr->computeTransformToTarget(scanMsg->header.stamp))
    {
      // target frame not available
      return;
    }

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      // calculate individual transform for each packet to account for ego
      // during one rotation of the velodyne sensor
      if(!container_ptr->computeTransformToFixed(scanMsg->packets[i].stamp))
      {
        // fixed frame not available
        return;
      }

      data_->unpack(scanMsg->packets[i], *container_ptr,
                    scanMsg->header.stamp, i);
    }
    // publish the accumulated cloud message
    output_.publish(container_ptr->finishCloud(scanMsg->header.stamp));

    diag_topic_->tick(scanMsg->header.stamp);
    diagnostics_.update();
  }


  void Transform::processEthernetMsgs(const ethernet_msgs::PacketConstPtr& msg)
  {
      if (output_.getNumSubscribers() == 0) // no one listening?
          return;                           // avoid much work

      uint16_t current_angle = 0; // 0...36000
      std::memcpy(&current_angle, &msg->payload[2], 2);

      if (prev_rotation_angle_ < 0 || prev_rotation_angle_ < 18000 && current_angle >= 18000) // new rotation
      {
          if (prev_rotation_angle_ >= 0) // don't publish on very first packet, just init new scan
          {
              output_.publish(container_ptr->finishCloud(scan_->header.stamp));
              std::cout << "LATENCY1: " << (ros::Time::now() - scan_->header.stamp).toSec() << std::endl;
          }

          scan_.reset(new velodyne_msgs::VelodyneScan);
          scan_->header.stamp = msg->header.stamp;
          scan_->header.frame_id = config_.sensor_frame;
          container_ptr->setup(scan_);
          packet_pos_in_scan = 0;

          // container_ptr->computeTransformToTarget(msg->header.stamp)
      }
      prev_rotation_angle_ = current_angle;

      if (std::abs((msg->header.stamp - last_transform_lookup).toSec()) > 0.01)
      {
          container_ptr->computeTransformToFixed(msg->header.stamp);
          last_transform_lookup = msg->header.stamp;
      }

      velodyne_msgs::VelodynePacket tmp_packet;
      std::memcpy(&tmp_packet.data, &msg->payload[0], msg->payload.size());
      tmp_packet.stamp = msg->header.stamp;
      data_->unpack(tmp_packet, *container_ptr, scan_->header.stamp, packet_pos_in_scan);

      ++packet_pos_in_scan;
  }

} // namespace velodyne_pointcloud
