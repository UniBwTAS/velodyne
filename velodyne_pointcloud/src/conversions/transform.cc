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
#include <velodyne_pointcloud/pointcloud_extended_confidence.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name):
          raw_data_ptr_(new velodyne_rawdata::RawData),
    first_rcfg_call(true),
    diagnostics_(node, private_nh, node_name)
  {
    boost::optional<velodyne_pointcloud::Calibration> calibration = raw_data_ptr_->setup(private_nh);
    config_.model = raw_data_ptr_->get_sensor_model();
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
    output_ret_mode_ = node.advertise<velodyne_msgs::VelodyneReturnMode>("velodyne_return_mode", 10,true);

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
        velodyne_ethernet_msgs_ = node.subscribe(config_.input_ethernet_msgs_topic, 10*604, &Transform::processEthernetMsgs, this);
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



      double cut_angle;
      private_nh.param("cut_angle", cut_angle, M_PI);
      if (cut_angle < 0.0) {
          ROS_INFO_STREAM("Cut at specific angle feature deactivated.");
      } else if (cut_angle < (2 * M_PI)) {
          ROS_INFO_STREAM("Cut at specific angle feature activated. "
                          "Cutting velodyne points always at " << cut_angle << " rad.");
      } else {
          ROS_ERROR_STREAM("cut_angle parameter is out of range. Allowed range is "
                            << "between 0.0 and 2*PI or negative values to deactivate this feature.");
          cut_angle = -0.01;
      }

      // Convert cut_angle from radian to one-hundredth degree,
      // which is used in velodyne packets
      _cut_angle = int((cut_angle * 360 / (2 * M_PI)) * 100);


      if (! private_nh.getParam("rpm", _rpm))
      {
          _rpm = 600.0;
          ROS_ERROR("No Velodyne RPM specified using default %f!", _rpm);
      }

  }

    void Transform::reconfigure_callback(
            velodyne_pointcloud::TransformNodeConfig &config, uint32_t level) {
        ROS_INFO_STREAM("Reconfigure request.");
        raw_data_ptr_->setParameters(config.min_range, config.max_range,
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

        if (first_rcfg_call || config.cloud_type != config_.cloud_type) {
            first_rcfg_call = false;
            config_.cloud_type = config.cloud_type;

            if (config_.cloud_type == ORGANIZED_TYPE) {
                ROS_INFO_STREAM("Using the organized cloud format...");
                container_ptr = boost::shared_ptr<OrganizedCloudXYZIRT>(
                        new OrganizedCloudXYZIRT(config_.max_range, config_.min_range,
                                                 config_.target_frame, config_.fixed_frame,
                                                 config_.num_lasers, raw_data_ptr_->pointsPerPacket()));
            } else {
                if (config_.cloud_type == XYZIRT_TYPE) {
                    ROS_INFO_STREAM("Using the XYZIRT cloud format...");
                    container_ptr =
                            boost::shared_ptr<PointcloudXYZIRT>(new PointcloudXYZIRT(
                                    config_.max_range, config_.min_range, config_.target_frame,
                                    config_.fixed_frame, raw_data_ptr_->pointsPerPacket()));
                } else {
                    if (config_.cloud_type == EXTENDED_TYPE) {
                        if (config_.model != velodyne_rawdata::VLS128) {
                            ROS_ERROR_STREAM("Using the Extended cloud format with wrong model, only VLS128!");
                        }
                        ROS_INFO_STREAM("Using the Extended cloud format...");
                        container_ptr =
                                boost::shared_ptr<PointcloudExtended>(new PointcloudExtended(
                                        config_.max_range, config_.min_range, config_.target_frame,
                                        config_.fixed_frame, config_.num_lasers, raw_data_ptr_->pointsPerPacket()));
                    } else {

                        if (config_.cloud_type == EXTENDEDCONF_TYPE) {

                            if (config_.model != velodyne_rawdata::VLS128) {
                                ROS_ERROR_STREAM("Using the Extended cloud format with wrong model, only VLS128!");
                            }
                            ROS_INFO_STREAM("Using the Extended cloud format with confidence information...");
                            container_ptr =
                                    boost::shared_ptr<PointcloudExtendedConfidence>(new PointcloudExtendedConfidence(
                                            config_.max_range, config_.min_range, config_.target_frame,
                                            config_.fixed_frame, config_.num_lasers, raw_data_ptr_->pointsPerPacket()));
                        } else {
                            ROS_ERROR("Wrong option in parameter cloud_type %s, using default type %s",
                                      config_.cloud_type.c_str(), XYZIRT_TYPE.c_str());

                            container_ptr =
                                    boost::shared_ptr<PointcloudXYZIRT>(new PointcloudXYZIRT(
                                            config_.max_range, config_.min_range, config_.target_frame,
                                            config_.fixed_frame, raw_data_ptr_->pointsPerPacket()));

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

    if (scanMsg->packets.empty())
        return;

    const int return_mode = raw_data_ptr_->read_return_mode(scanMsg->packets[0]);

    container_ptr->set_return_mode(return_mode);
    if (config_.model != velodyne_rawdata::VLS128 && (return_mode != velodyne_rawdata::VLS128_RETURN_MODE_LAST &&
            return_mode != velodyne_rawdata::VLS128_RETURN_MODE_STRONGEST) ) {
        ROS_ERROR_STREAM("Using double return mode with a sensor model that is not implemented, only use double return mode with the VLS128!");
    }
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

      raw_data_ptr_->unpack(scanMsg->packets[i], *container_ptr,
                    scanMsg->header.stamp, i);
    }
    // publish the accumulated cloud message
    output_.publish(container_ptr->finishCloud(scanMsg->header.stamp));

    // publish return mode only if it has changed
    velodyne_msgs::VelodyneReturnMode retmode_msg;
    if(container_ptr->get_return_mode(retmode_msg))
        output_ret_mode_.publish(retmode_msg);

    diag_topic_->tick(scanMsg->header.stamp);
    diagnostics_.update();
  }


    void
    Transform::processEthernetMsgs(const ethernet_msgs::PacketConstPtr &ethernet_msg) {
        if (output_.getNumSubscribers() == 0)      // no one listening?
            return;                                  // avoid much work

        // global vector to buffer several ethernet_msg:
        static std::shared_ptr<std::vector<ethernet_msgs::PacketConstPtr>>
                vec_ethernet_msgs = std::make_shared<std::vector<ethernet_msgs::PacketConstPtr>>();

        // a packet contains data from 3 azimuth angles  use the biggest (last four blocks) in single ret in double all the azimuths are the same
        std::size_t azimuth_data_pos = velodyne_rawdata::BLOCK_SIZE * 8 + velodyne_rawdata::FLAG_SIZE;
        uint16_t azimuth = 0; // 0...36000
        std::memcpy(&azimuth, &ethernet_msg->payload[azimuth_data_pos], velodyne_rawdata::AZIMUTH_SIZE);

        if (_last_azimuth == -1) {
            _last_azimuth = azimuth;
            return;
        }

        // get if the packet is single or double return

        std::size_t ret_mode_data_pos = velodyne_rawdata::BLOCK_SIZE * velodyne_rawdata::BLOCKS_PER_PACKET + velodyne_rawdata::TIMESTAMP_SIZE;
        uint8_t mode = 0; // 55 .. 58
        std::memcpy(&mode, &ethernet_msg->payload[ret_mode_data_pos], velodyne_rawdata::RETURN_MODE_BYTE_SIZE);

        // Compare with the return mode of the previous package to detect a change
        if (_last_return_mode != mode)
        {
            // Change in the return mode from previous package
            // set first rotation flag to tru to discard the current polled packages, as they are
            // a combination of packages taken with different return modes
            _first_rotation = true;
            _last_return_mode = mode;
        }

        if ((_last_azimuth < _cut_angle && _cut_angle <= azimuth) || //  e.g. la 50,  ca 60,  a 70
            (azimuth < _last_azimuth && _cut_angle <= azimuth) ||    //  e.g. la 350, ca 5,   a 10
            (azimuth < _last_azimuth && _last_azimuth < _cut_angle)) {// e.g. la 340, ca 359, a 15

            if (_first_rotation) { // discard the first accumulation of packets as it
                // does not build a complete rotation
                _first_rotation = false;
                ROS_INFO("Discard first incomplete scan expected packets %i, polled "
                         "packets %li",
                         _npackets, vec_ethernet_msgs->size());
                vec_ethernet_msgs->clear();
                _last_azimuth = azimuth;

                return;
            }

            
            const double frequency = (_rpm / 60.0);
            if (mode == velodyne_rawdata::VLS128_RETURN_MODE_STRONGEST || mode == velodyne_rawdata::VLS128_RETURN_MODE_LAST)
                _npackets = (int) ceil(velodyne_rawdata::PACKET_RATE_SINGLE_RET_MODE / frequency); // packets per rev
            else
                _npackets = (int) ceil(velodyne_rawdata::PACKET_RATE_DUAL_RET_MODE / frequency); // packets per rev


            // Cut angle passed, one full revolution collected
            velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
            scan->packets.reserve(_npackets);

            // if the starting angle for the current polled packages is still bigger than the current azimuth include this package in the scan
            bool omit_package = false;
            if (scan_first_azimuth > azimuth) {
                vec_ethernet_msgs->push_back(ethernet_msg);
                omit_package = true;
            }

            for (ethernet_msgs::PacketConstPtr &msg: *vec_ethernet_msgs) {
                velodyne_msgs::VelodynePacket tmp_packet;
                std::memcpy(&tmp_packet.data, &msg->payload[0], msg->payload.size());
                tmp_packet.stamp = msg->header.stamp;
                scan->packets.push_back(tmp_packet);
            }
            if (!vec_ethernet_msgs->empty()) {
                scan->header.stamp = vec_ethernet_msgs->front()->header.stamp;
                scan->header.frame_id = config_.sensor_frame;
                // publish the pointcloud2
                processScan(scan);
            }

            // delete the buffer
            vec_ethernet_msgs->clear();

            if (omit_package) {
                _last_azimuth = azimuth;
                return;
            }
        }
        _last_azimuth = azimuth;

        //store the first azimuth for the scan
        if (vec_ethernet_msgs->empty()) {
            scan_first_azimuth = azimuth;
        }

        // adding to global accumulator
        vec_ethernet_msgs->push_back(ethernet_msg);
    }

} // namespace velodyne_pointcloud
