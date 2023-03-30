/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2019, Kaarta Inc, Shawn Hanna
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Shawn Hanna
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_rawdata
{
inline float SQR(float val) { return val*val; }

  ////////////////////////////////////////////////////////////////////////
  //
  // RawData base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  RawData::RawData() {}
  
  /** Update parameters: conversions and update */
  void RawData::setParameters(double min_range,
                              double max_range,
                              double view_direction,
                              double view_width)
  {
    config_.min_range = min_range;
    config_.max_range = max_range;

    //converting angle parameters into the velodyne reference (rad)
    config_.tmp_min_angle = view_direction + view_width/2;
    config_.tmp_max_angle = view_direction - view_width/2;
    
    //computing positive modulo to keep theses angles into [0;2*M_PI]
    config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle,2*M_PI) + 2*M_PI,2*M_PI);
    config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle,2*M_PI) + 2*M_PI,2*M_PI);

    //converting into the hardware velodyne ref (negative yaml and degrees)
    //adding 0.5 perfomrs a centered double to int conversion 
    config_.min_angle = 100 * (2*M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (2*M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;

    if (config_.min_angle == config_.max_angle)
    {
      //avoid returning empty cloud if min_angle = max_angle
      config_.min_angle = 0;
      config_.max_angle = 36000;
    }
  }

  int RawData::pointsPerPacket() const
  {
    if( calibration_.num_lasers == 16)
    {
      return BLOCKS_PER_PACKET * VLP16_FIRINGS_PER_BLOCK *
          VLP16_SCANS_PER_FIRING;
    }
    else{
        if (current_return_mode != VLS128_RETURN_MODE_DUAL && current_return_mode != VLS128_RETURN_MODE_DUAL_CONF)
            return BLOCKS_PER_PACKET * POINTS_PER_BLOCK;
        else
            return (BLOCKS_PER_PACKET-VLS128_BLOCKS_PER_FIRING_SEQ) * POINTS_PER_BLOCK;
    }
  }

  /**
   * Build a timing table for each block/firing. Stores in timing_offsets vector
   */
  bool RawData::buildTimings(){
    // vlp16    
    if (config_.model == "VLP16"){
      // timing table calculation, from velodyne user manual
      timing_offsets.resize(12);
      for (size_t i=0; i < timing_offsets.size(); ++i){
        timing_offsets[i].resize(32);
      }
      // constants
      double full_firing_cycle = 55.296 * 1e-6; // seconds
      double single_firing = 2.304 * 1e-6; // seconds
      double dataBlockIndex, dataPointIndex;
      bool dual_mode = false;
      // compute timing offsets
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          if (dual_mode){
            dataBlockIndex = (x - (x % 2)) + (y / 16);
          }
          else{
            dataBlockIndex = (x * 2) + (y / 16);
          }
          dataPointIndex = y % 16;
          //timing_offsets[block][firing]
          timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
        }
      }
    }
    // vlp32
    else if (config_.model == "32C"){
      // timing table calculation, from velodyne user manual
      timing_offsets.resize(12);
      for (size_t i=0; i < timing_offsets.size(); ++i){
        timing_offsets[i].resize(32);
      }
      // constants
      double full_firing_cycle = 55.296 * 1e-6; // seconds
      double single_firing = 2.304 * 1e-6; // seconds
      double dataBlockIndex, dataPointIndex;
      bool dual_mode = false;
      // compute timing offsets
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          if (dual_mode){
            dataBlockIndex = x / 2;
          }
          else{
            dataBlockIndex = x;
          }
          dataPointIndex = y / 2;
          timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
        }
      }
    }
    // hdl32
    else if (config_.model == "32E"){
      // timing table calculation, from velodyne user manual
      timing_offsets.resize(12);
      for (size_t i=0; i < timing_offsets.size(); ++i){
        timing_offsets[i].resize(32);
      }
      // constants
      double full_firing_cycle = 46.080 * 1e-6; // seconds
      double single_firing = 1.152 * 1e-6; // seconds
      double dataBlockIndex, dataPointIndex;
      bool dual_mode = false;
      // compute timing offsets
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          if (dual_mode){
            dataBlockIndex = x / 2;
          }
          else{
            dataBlockIndex = x;
          }
          dataPointIndex = y / 2;
          timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
        }
      }
    }
    else if (config_.model == "VLS128"){
        // Only the timing offsets of the first firing sequence can be calculated beforehand, as
        // after firmware update there is a variable rest period after each firing sequence
        // to avoid interference from other sensors

      timing_offsets.resize(1);
      for(size_t i=0; i < timing_offsets.size(); ++i)
      {
        timing_offsets[i].resize(16);
      }

      const double full_firing_cycle = VLS128_SEQ_TDURATION * 1e-6; //seconds
      const double single_firing = VLS128_CHANNEL_TDURATION * 1e-6; // seconds
      const double offset_paket_time = VLS128_TOH_ADJUSTMENT * 1e-6; //seconds
      const double duration_rest_period = VLS128_REST_PERIOD_TDURATION * 1e-6; //seconds
      double sequenceIndex, firingGroupIndex;
      // Compute timing offsets
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){

          sequenceIndex = x;
          firingGroupIndex = y;
          timing_offsets[x][y] = (full_firing_cycle * sequenceIndex) + (single_firing * firingGroupIndex)
                  - offset_paket_time + (duration_rest_period*(firingGroupIndex>7?1:0));
          ROS_DEBUG(" firing_seque %lu firing_group %lu offset %f",x,y,timing_offsets[x][y]);
        }
      }
    }
    else{
      timing_offsets.clear();
      ROS_WARN("Timings not supported for model %s", config_.model.c_str());
    }

    if (timing_offsets.size()){
      ROS_INFO("VELODYNE TIMING TABLE [usec]:");
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          printf("%04.3f ", timing_offsets[x][y] * 1e6);
        }
        printf("\n");
      }
      return true;
    }
    else{
      ROS_WARN("NO TIMING OFFSETS CALCULATED. ARE YOU USING A SUPPORTED VELODYNE SENSOR?");
    }
    return false;
  }

  /** Set up for on-line operation. */
  boost::optional<velodyne_pointcloud::Calibration> RawData::setup(ros::NodeHandle private_nh)
  {

    if (!private_nh.getParam("model", config_.model))
    {
      config_.model =  std::string("64E");
      ROS_ERROR("No Velodyne Sensor Model specified using default %s!", config_.model.c_str());

    }
      if (! private_nh.param("rpm", config_.rpm))
      {
          config_.rpm = 600.0;
          ROS_ERROR("No Velodyne RPM specified using default %f!", config_.rpm);
      }
    buildTimings();

    // get path to angles.config file for this device
    if (!private_nh.getParam("calibration", config_.calibrationFile))
    {
      ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

      // have to use something: grab unit test version as a default
      std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
      config_.calibrationFile = pkgPath + "/params/64e_utexas.yaml";
    }

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    if (!loadCalibration()) {
      return boost::none;
    }
    ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");

    setupSinCosCache();
    setupAzimuthCache();

    return calibration_;
  }

  /** Set up for offline operation */
  int RawData::setupOffline(std::string calibration_file, double max_range_, double min_range_)
  {

    config_.max_range = max_range_;
    config_.min_range = min_range_;
    ROS_INFO_STREAM("data ranges to publish: ["
      << config_.min_range << ", "
      << config_.max_range << "]");

    config_.calibrationFile = calibration_file;

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    if (!loadCalibration()) {
      return -1;
    }
    ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");

    setupSinCosCache();
    setupAzimuthCache();

    return 0;
  }

  bool RawData::loadCalibration() {

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
      ROS_ERROR_STREAM("Unable to open calibration file: " << config_.calibrationFile);
      return false;
    }
    return true;

  }
  void RawData::setupSinCosCache()
  {
    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }
  }

void RawData::setupAzimuthCache()
{ // this is a simple rule of three: if in a full cycle (seq) the sensor rotates azimuth1-azimuth0 then how much does it
      // rotate in 2.666 times the firing group index. The + i/8 is there to take into account the rest period in the old firmware
      // this also does not take into account that the azimuth in the message is measured after the 4 first firing groups have fired.
      // The sqe duration is also no longer the same for each seq due to the random waiting time after firing the 128 lasers.

  if (config_.model == "VLS128") {
    for (uint8_t i = 0; i < 16; i++) {
      vls_128_laser_azimuth_cache[i] =
          (VLS128_CHANNEL_TDURATION / VLS128_SEQ_TDURATION) * (i + i / 8);
    }
  }
  else{
    ROS_WARN("No Azimuth Cache configured for model %s", config_.model.c_str());
  }
}


  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase& data,
                     const ros::Time& scan_start_time, const size_t packet_pos_in_scan)
  {
    using velodyne_pointcloud::LaserCorrection;
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);

    /** special parsing for the VLS128 **/
    if (pkt.data[VLS128_MODEL_ID_POSITION] == VLS128_MODEL_ID) { // VLS 128
      unpack_vls128(pkt, data, scan_start_time, packet_pos_in_scan);
      return;
    }

    /** special parsing for the VLP16 **/
    if (calibration_.num_lasers == 16)
    {
      unpack_vlp16(pkt, data, scan_start_time);
      return;
    }

    float time_diff_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();
    
    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

      // upper bank lasers are numbered [0..31]
      // NOTE: this is a change from the old velodyne_common implementation

      int bank_origin = 0;
      if (raw->blocks[i].header == LOWER_BANK) {
        // lower bank lasers are [32..63]
        bank_origin = 32;
      }

      for (int j = 0, k = 0; j < POINTS_PER_BLOCK; j++, k += RAW_POINT_SIZE) {
        
        float x, y, z;
        float intensity;
        const uint8_t laser_number  = j + bank_origin;
        float time = 0;

        const LaserCorrection &corrections = calibration_.laser_corrections[laser_number];

        /** Position Calculation */
        const raw_block_t &block = raw->blocks[i];
        union two_bytes tmp;
        tmp.bytes[0] = block.data[k];
        tmp.bytes[1] = block.data[k+1];

        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((block.rotation >= config_.min_angle
             && block.rotation <= config_.max_angle
             && config_.min_angle < config_.max_angle)
             ||(config_.min_angle > config_.max_angle 
             && (block.rotation <= config_.max_angle
             || block.rotation >= config_.min_angle))){

          if (timing_offsets.size())
          {
            time = timing_offsets[i][j] + time_diff_start_to_this_packet;
          }

          if (tmp.uint == 0) // no valid laser beam return
          {
            // call to addPoint is still required since output could be organized
            data.addPoint(nanf(""), nanf(""), nanf(""), corrections.laser_ring, raw->blocks[i].rotation, nanf(""), nanf(""), time);
            continue;
          }

          float distance = tmp.uint * calibration_.distance_resolution_m;
          distance += corrections.dist_correction;
  
          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;
  
          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle = 
            cos_rot_table_[block.rotation] * cos_rot_correction +
            sin_rot_table_[block.rotation] * sin_rot_correction;
          float sin_rot_angle = 
            sin_rot_table_[block.rotation] * cos_rot_correction -
            cos_rot_table_[block.rotation] * sin_rot_correction;
  
          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;
  
          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
  
          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) xx=-xx;
          if (yy < 0) yy=-yy;
    
          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = 
              (corrections.dist_correction - corrections.dist_correction_x)
                * (xx - 2.4) / (25.04 - 2.4) 
              + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = 
              (corrections.dist_correction - corrections.dist_correction_y)
                * (yy - 1.93) / (25.04 - 1.93)
              + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }
  
          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
          ///the expression wiht '-' is proved to be better than the one with '+'
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
  
          float distance_y = distance + distance_corr_y;
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
  
          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
  
          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;
  
          /** Intensity Calculation */
  
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;
  
          intensity = raw->blocks[i].data[k+2];
  
          float focal_offset = 256 
                             * (1 - corrections.focal_distance / 13100) 
                             * (1 - corrections.focal_distance / 13100);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope * (std::abs(focal_offset - 256 * 
            SQR(1 - static_cast<float>(tmp.uint)/65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, raw->blocks[i].rotation, distance, intensity, time);
        }
      }
      data.newLine();
    }
  }

/** @brief gets the return mode from the raw VLS128 packet
*
*  @param pkt raw packet
*/
unsigned int RawData::read_return_mode(const velodyne_msgs::VelodynePacket& pkt)
{
    const auto *raw = (const raw_packet_vls128_t *) &pkt.data[0];

    return raw->return_mode;

}


/** @brief convert raw VLS128 packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack_vls128(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase& data,
                            const ros::Time& scan_start_time, const size_t packet_pos_in_scan) {
  float azimuth_diff, azimuth_corrected_f, azimuth_rot_corrected_f;
  float last_azimuth_diff = 0;
  uint16_t azimuth, azimuth_next, azimuth_corrected, azimuth_rot_corrected;
  float x_coord, y_coord, z_coord;
  float distance;
  const auto *raw = (const raw_packet_vls128_t *) &pkt.data[0];
  union two_bytes tmp;

  float cos_vert_angle, sin_vert_angle, cos_rot_correction, sin_rot_correction;
  float cos_rot_angle, sin_rot_angle;
  float xy_distance;

  const float vel = config_.rpm * 36000.0f / 60.0f;

  // time from the arrival of the first packet in the scan (this only works if time stamp first packes ids configured)
  // to the arrival of the current packet
  float time_diff_scan_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();

  uint8_t laser_number, firing_order;

  current_return_mode = raw->return_mode;
  bool dual_return = (current_return_mode == VLS128_RETURN_MODE_DUAL);

  // common functions between return modes used while parsing the packets

        auto get_bank_origin = [](const uint16_t &block_header) {
            switch (block_header) {
                case VLS128_BANK_1:
                    return 0;
                case VLS128_BANK_2:
                    return 32;
                case VLS128_BANK_3:
                    return 64;
                case VLS128_BANK_4:
                    return 96;
                default:
                    return -1;
            }
        };

//        auto calc_point_and_add_to_data = [this](
//                const raw_block_t &block,
//                const int position,
//                const velodyne_pointcloud::LaserCorrection &corrections,
//                const uint16_t& azimuth_corrected,
//                const uint16_t& azimuth_rot_corrected,
//                const std::uint16_t& rotation_segment,
//                const uint16_t& firing_seq_in_scan,
//                const uint8_t& laser_number,
//                const uint8_t& first_return_flag,
//                const float time,
//                DataContainerBase& data) {
//
//            if ((config_.min_angle < config_.max_angle && azimuth_rot_corrected >= config_.min_angle &&
//                 azimuth_rot_corrected <= config_.max_angle) ||
//                (config_.min_angle > config_.max_angle &&
//                 (azimuth_rot_corrected >= config_.min_angle || azimuth_rot_corrected <= config_.max_angle))) {
//
//                union two_bytes tmp;
//
//                // distance extraction
//                tmp.bytes[0] = block.data[position];
//                tmp.bytes[1] = block.data[position + 1];
//                const float distance = tmp.uint * VLS128_DISTANCE_RESOLUTION;
//
//                // convert polar coordinates to Euclidean XYZ
//                const float cos_vert_angle = corrections.cos_vert_correction;
//                const float sin_vert_angle = corrections.sin_vert_correction;
//                const float cos_rot_correction = corrections.cos_rot_correction;
//                const float sin_rot_correction = corrections.sin_rot_correction;
//
//                // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
//                // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
//                const float cos_rot_angle =
//                        cos_rot_table_[azimuth_corrected] * cos_rot_correction +
//                        sin_rot_table_[azimuth_corrected] * sin_rot_correction;
//                const float sin_rot_angle =
//                        sin_rot_table_[azimuth_corrected] * cos_rot_correction -
//                        cos_rot_table_[azimuth_corrected] * sin_rot_correction;
//
//                // Compute the distance in the xy plane (w/o accounting for
//                // rotation)
//                const float xy_distance = distance * cos_vert_angle;
//
//                data.addPoint(xy_distance * cos_rot_angle,
//                              -(xy_distance * sin_rot_angle),
//                              distance * sin_vert_angle,
//                              corrections.laser_ring,
//                              azimuth_rot_corrected,
//                              distance,
//                              static_cast<float>(block.data[position + 2]),
//                              time,
//                              corrections.laser_ring + calibration_.num_lasers * rotation_segment,
//                              rotation_segment,
//                              firing_seq_in_scan,
//                              laser_number,
//                              first_return_flag);
//            }
//            else {
//                // point is outside the valid angle range
//
//                data.addPoint(nanf(""),
//                              nanf(""),
//                              nanf(""),
//                              corrections.laser_ring,
//                              azimuth_corrected,
//                              nanf(""),
//                              0.0,
//                              time,
//                              corrections.laser_ring +
//                              calibration_.num_lasers * rotation_segment,
//                              rotation_segment,
//                              firing_seq_in_scan,
//                              laser_number,
//                              first_return_flag);
//            }
//        };

  // parse the packages according to the return mode
  if(current_return_mode == VLS128_RETURN_MODE_STRONGEST || current_return_mode == VLS128_RETURN_MODE_LAST)
  {
      // variables used to estimate the time offset of each point
      const float time_to_first_random_rest = time_diff_scan_start_to_this_packet
                                              - (VLS128_TOH_ADJUSTMENT * 1e-6) + (VLS128_SEQ_TDURATION* 1e-6);
      float first_random_rest_estimate = 0.0f;
      float second_random_rest_estimate = 0.0f;
      float third_random_rest_estimate = 0.0f;

      // number of firing sequences in this scan
      int num_firing_sequences_in_one_scan = std::floor(BLOCKS_PER_PACKET/VLS128_BLOCKS_PER_FIRING_SEQ) * data.packetsInScan();

      // Parse the blocks
      for (int block = 0;
           block < BLOCKS_PER_PACKET ; block++) {
          // cache block for use
          const raw_block_t &current_block = raw->blocks[block];
          float point_time = 0;

          //To which firing seq does this block belong in the package
          const uint8_t  firing_seq_in_package = block/VLS128_BLOCKS_PER_FIRING_SEQ;
          // To which firing sequence does this block belongs in the scan
          const uint16_t firing_seq_in_scan = packet_pos_in_scan * (BLOCKS_PER_PACKET / VLS128_BLOCKS_PER_FIRING_SEQ) +
                  firing_seq_in_package;

          // Used to detect which bank of 32 lasers is in this block
          int bank_origin = get_bank_origin(current_block.header);
          if (bank_origin < 0)
          {
              // ignore packets with mangled or otherwise different contents
              // Do not flood the log with messages, only issue at most one
              // of these warnings per minute.
              ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLS-128 packet: block "
                      << block << " header value is "
                      << raw->blocks[block].header);
              return; // bad packet: skip the rest
          }

          // Calculate difference between current and next azimuth angle.

          azimuth = current_block.rotation;

          if ((block % VLS128_BLOCKS_PER_FIRING_SEQ == 0) && (firing_seq_in_package < 2)) {
              // Get the rotation of the next firing sequence to calculate how far we rotate between sequences
              azimuth_next = raw->blocks[block + VLS128_BLOCKS_PER_FIRING_SEQ].rotation;

              // Finds the difference between two successive blocks
              azimuth_diff = (float) ((36000 + azimuth_next - azimuth) % 36000);

              // This is used when the last block is next to predict rotation amount
              last_azimuth_diff = azimuth_diff;
          } else {
              // This makes the assumption the difference between the last block and the next packet is the
              // same as the last to the second to last.
              // Assumes RPM doesn't change much between blocks
              azimuth_diff = last_azimuth_diff;
          }

          // calculate the timing offsets depending on the block position in the package

          float time_from_scan_start_to_firing_seq_azimuth_time = 0;

          // time to use in the correction of the single lasers due to the rotation according to the seq number
          float time_to_cover_azimuth_diff = 0;

          if(firing_seq_in_package == 0) {

              time_from_scan_start_to_firing_seq_azimuth_time =
                      time_diff_scan_start_to_this_packet;

              // estimate the first random rest time using the difference between the azimuths of the first and
              // second firing seq
              first_random_rest_estimate = (azimuth_diff / vel) - (VLS128_SEQ_TDURATION * 1e-6);

              time_to_cover_azimuth_diff = first_random_rest_estimate +
                                           (VLS128_TOH_ADJUSTMENT * 1e-6);
          }
          else
          {
              // To calculate the time offset of the measurements beyond the first
              // firing sequence, the random time between firing sequences must be
              // taken into account (see pg 65 of manual Rev 3)

              if (firing_seq_in_package == 1) {

                  time_from_scan_start_to_firing_seq_azimuth_time = time_to_first_random_rest
                                                                    + first_random_rest_estimate +
                                                                    (VLS128_TOH_ADJUSTMENT * 1e-6);

                  second_random_rest_estimate = (azimuth_diff / vel) - (VLS128_SEQ_TDURATION * 1e-6);

                  time_to_cover_azimuth_diff = second_random_rest_estimate +
                                               (VLS128_TOH_ADJUSTMENT * 1e-6);

              } else {

                  time_from_scan_start_to_firing_seq_azimuth_time = time_to_first_random_rest +
                                                                    first_random_rest_estimate +
                                                                    (VLS128_SEQ_TDURATION * 1e-6) +
                                                                    second_random_rest_estimate +
                                                                    (VLS128_TOH_ADJUSTMENT * 1e-6);

                  // The third random rest period can not be estimated without the next package,
                  // here the average values according to the manual is used

                  third_random_rest_estimate = 1.941 * 1e-6;

                  time_to_cover_azimuth_diff = third_random_rest_estimate +
                                               (VLS128_TOH_ADJUSTMENT * 1e-6);

              }

          }

          // Parse the data points
          for (int j = 0, k = 0; j < POINTS_PER_BLOCK; j++, k += RAW_POINT_SIZE) {

              laser_number = j + bank_origin;   // Offset the laser in this block by which block it's in
              firing_order = laser_number / 8;  // VLS-128 fires 8 lasers at a time

              if (!timing_offsets.empty()) {

                  point_time = timing_offsets[0][firing_order] +
                          time_from_scan_start_to_firing_seq_azimuth_time;
              }
              else
              {
                  point_time = 0;
              }

              velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[laser_number];

              // correct for the laser rotation as a function of timing during the firings
              azimuth_corrected_f = azimuth + (azimuth_diff * timing_offsets[0][firing_order]/time_to_cover_azimuth_diff);
              azimuth_corrected = ((uint16_t) round(azimuth_corrected_f)) % 36000;

              // correct azimuth for laser rot offset
              // Keep it in range [0,36000]
              azimuth_rot_corrected_f = fmod(
                      fmod(azimuth_corrected_f - corrections.rot_correction_deg * 100, 36000) + 36000, 36000);
              azimuth_rot_corrected = ((uint16_t) round(azimuth_rot_corrected_f));

              std::uint16_t rotation_segment = ((7 - (laser_number - (8 * std::floor(laser_number / 8)))) * 9) +
                      firing_seq_in_scan;

              while (rotation_segment >= num_firing_sequences_in_one_scan) {
                  rotation_segment = rotation_segment - num_firing_sequences_in_one_scan;
              }

              inl_calc_point_and_add_to_data(
                      current_block,
                      k,
                      corrections,
                      azimuth_corrected,
                      azimuth_rot_corrected,
                      rotation_segment,
                      firing_seq_in_scan,
                      laser_number,
                      1,
                      point_time,
                      data);
          }

          if ((block + 1) % VLS128_BLOCKS_PER_FIRING_SEQ ==
              0) //add a new line every 4 blocks (one firing for each of the 128 lasers)
              data.newLine();
      }
  }
  else
  {
      if(current_return_mode == VLS128_RETURN_MODE_DUAL)
      {
          // number of firing sequences in this scan
          int num_firing_sequences_in_one_scan = data.packetsInScan();

          // Parse the blocks
          for(int block = 0; block < BLOCKS_PER_PACKET - VLS128_BLOCKS_PER_FIRING_SEQ; block = block + 2 )
          {
              const raw_block_t &first_ret_block = raw->blocks[block];
              const raw_block_t &second_ret_block = raw->blocks[block+1];
              float point_time = 0;

              //To which firing seq does this block belong in the package
              const uint8_t  firing_seq_in_package = 0;

              // To which firing sequence does this block belongs in the scan
              const uint16_t firing_seq_in_scan = packet_pos_in_scan  +
                                                  firing_seq_in_package;

              // Used to detect which bank of 32 lasers is in this block
              int bank_origin = get_bank_origin(first_ret_block.header);
              if (bank_origin < 0)
              {
                  // ignore packets with mangled or otherwise different contents
                  // Do not flood the log with messages, only issue at most one
                  // of these warnings per minute.
                  ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLS-128 packet: block "
                          << block << " header value is "
                          << raw->blocks[block].header);
                  return; // bad packet: skip the rest
              }

              // Is not necessary to Calculate difference between current and next block's azimuth angle.
              // as all blocks in dual return have the same azimuth.
              // The difference with the previous packet azimuth is used as a estimate
              // of the difference of the azimuth of this packet and the next one (constant rpm)
              // first time this node is used the difference is simply 0.

              azimuth = first_ret_block.rotation;
              if (azimuth_previous_packet != 0)
                azimuth_diff = (float) ((36000 + azimuth - azimuth_previous_packet) % 36000);
              else
                  azimuth_diff = 0;
              azimuth_previous_packet = azimuth;

              // Parse the data points

              for (int j = 0, k = 0; j < POINTS_PER_BLOCK; j++, k += RAW_POINT_SIZE)
              {

                  laser_number = j + bank_origin;   // Offset the laser in this block by which block it's in
                  firing_order = laser_number / 8;  // VLS-128 fires 8 lasers at a time

                  if (!timing_offsets.empty()) {
                      point_time = timing_offsets[0][firing_order] +
                              time_diff_scan_start_to_this_packet;
                  }
                  else
                  {
                      point_time = 0;
                  }

                  velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[laser_number];

                  // correct for the laser rotation as a function of timing during the firings
                  azimuth_corrected_f = azimuth + (azimuth_diff * timing_offsets[0][firing_order]/(VLS128_SEQ_TDURATION * 1e-6));
                  azimuth_corrected = ((uint16_t) round(azimuth_corrected_f)) % 36000;

                  // correct azimuth for laser rot offset
                  // Keep it in range [0,36000]
                  azimuth_rot_corrected_f = fmod(
                          fmod(azimuth_corrected_f - corrections.rot_correction_deg * 100, 36000) + 36000, 36000);
                  azimuth_rot_corrected = ((uint16_t) round(azimuth_rot_corrected_f));

                  std::uint16_t rotation_segment = ((7 - (laser_number - (8 * std::floor(laser_number / 8)))) * 9) +
                                                   firing_seq_in_scan;

                  while (rotation_segment >= num_firing_sequences_in_one_scan) {
                      rotation_segment = rotation_segment - num_firing_sequences_in_one_scan;
                  }

                  inl_calc_point_and_add_to_data(
                          first_ret_block,
                          k,
                          corrections,
                          azimuth_corrected,
                          azimuth_rot_corrected,
                          rotation_segment,
                          firing_seq_in_scan,
                          laser_number,
                          1,
                          point_time,
                          data);

                  inl_calc_point_and_add_to_data(
                          second_ret_block,
                          k,
                          corrections,
                          azimuth_corrected,
                          azimuth_rot_corrected,
                          rotation_segment,
                          firing_seq_in_scan,
                          laser_number,
                          0,
                          point_time,
                          data);
              }
          }
          //add 2 new line after parsing this package (two firings for each of the 128 lasers)
          data.newLine();
          data.newLine();
      }
      else
      {
          if(current_return_mode == VLS128_RETURN_MODE_DUAL_CONF)
          {

          }
          else
          {
              ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-128 packet: Return mode is not recognised "
                      << current_return_mode);
          }
      }
  }
}
  
  /** @brief convert raw VLP16 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase& data, const ros::Time& scan_start_time)
  {
    float azimuth;
    float azimuth_diff;
    int raw_azimuth_diff;
    float last_azimuth_diff=0;
    float azimuth_corrected_f;
    int azimuth_corrected;
    float x, y, z;
    float intensity;

    float time_diff_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int block = 0; block < BLOCKS_PER_PACKET; block++) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                 << block << " header value is "
                                 << raw->blocks[block].header);
        return;                         // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      azimuth = (float)(raw->blocks[block].rotation);
      if (block < (BLOCKS_PER_PACKET-1)){
	raw_azimuth_diff = raw->blocks[block+1].rotation - raw->blocks[block].rotation;
        azimuth_diff = (float)((36000 + raw_azimuth_diff)%36000);
	// some packets contain an angle overflow where azimuth_diff < 0 
	if(raw_azimuth_diff < 0)//raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
	  {
	    ROS_WARN_STREAM_THROTTLE(60, "Packet containing angle overflow, first angle: " << raw->blocks[block].rotation << " second angle: " << raw->blocks[block+1].rotation);
	    // if last_azimuth_diff was not zero, we can assume that the velodyne's speed did not change very much and use the same difference
	    if(last_azimuth_diff > 0){
	      azimuth_diff = last_azimuth_diff;
	    }
	    // otherwise we are not able to use this data
	    // TODO: we might just not use the second 16 firings
	    else{
	      continue;
	    }
	  }
        last_azimuth_diff = azimuth_diff;
      }else{
        azimuth_diff = last_azimuth_diff;
      }

      for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++){
        for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_POINT_SIZE){
          velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[dsr];

          /** Position Calculation */
          union two_bytes tmp;
          tmp.bytes[0] = raw->blocks[block].data[k];
          tmp.bytes[1] = raw->blocks[block].data[k+1];
          
          /** correct for the laser rotation as a function of timing during the firings **/
          azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
          azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;
                 
          /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
          if ((azimuth_corrected >= config_.min_angle 
               && azimuth_corrected <= config_.max_angle 
               && config_.min_angle < config_.max_angle)
               ||(config_.min_angle > config_.max_angle 
               && (azimuth_corrected <= config_.max_angle 
               || azimuth_corrected >= config_.min_angle))){

            // convert polar coordinates to Euclidean XYZ
            float distance = tmp.uint * calibration_.distance_resolution_m;
            distance += corrections.dist_correction;
            
            float cos_vert_angle = corrections.cos_vert_correction;
            float sin_vert_angle = corrections.sin_vert_correction;
            float cos_rot_correction = corrections.cos_rot_correction;
            float sin_rot_correction = corrections.sin_rot_correction;
    
            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
            float cos_rot_angle = 
              cos_rot_table_[azimuth_corrected] * cos_rot_correction + 
              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            float sin_rot_angle = 
              sin_rot_table_[azimuth_corrected] * cos_rot_correction - 
              cos_rot_table_[azimuth_corrected] * sin_rot_correction;
    
            float horiz_offset = corrections.horiz_offset_correction;
            float vert_offset = corrections.vert_offset_correction;
    
            // Compute the distance in the xy plane (w/o accounting for rotation)
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
    
            // Calculate temporal X, use absolute value.
            float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
            // Calculate temporal Y, use absolute value
            float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
            if (xx < 0) xx=-xx;
            if (yy < 0) yy=-yy;
      
            // Get 2points calibration values,Linear interpolation to get distance
            // correction for X and Y, that means distance correction use
            // different value at different distance
            float distance_corr_x = 0;
            float distance_corr_y = 0;
            if (corrections.two_pt_correction_available) {
              distance_corr_x = 
                (corrections.dist_correction - corrections.dist_correction_x)
                  * (xx - 2.4) / (25.04 - 2.4) 
                + corrections.dist_correction_x;
              distance_corr_x -= corrections.dist_correction;
              distance_corr_y = 
                (corrections.dist_correction - corrections.dist_correction_y)
                  * (yy - 1.93) / (25.04 - 1.93)
                + corrections.dist_correction_y;
              distance_corr_y -= corrections.dist_correction;
            }
    
            float distance_x = distance + distance_corr_x;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
            x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    
            float distance_y = distance + distance_corr_y;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
            y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    
            // Using distance_y is not symmetric, but the velodyne manual
            // does this.
            /**the new term of 'vert_offset * cos_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
  
    
            /** Use standard ROS coordinate system (right-hand rule) */
            float x_coord = y;
            float y_coord = -x;
            float z_coord = z;
    
            /** Intensity Calculation */
            float min_intensity = corrections.min_intensity;
            float max_intensity = corrections.max_intensity;
    
            intensity = raw->blocks[block].data[k+2];
    
            float focal_offset = 256 * SQR(1 - corrections.focal_distance / 13100);
            float focal_slope = corrections.focal_slope;
            intensity += focal_slope * (std::abs(focal_offset - 256 * 
              SQR(1 - tmp.uint/65535)));
            intensity = (intensity < min_intensity) ? min_intensity : intensity;
            intensity = (intensity > max_intensity) ? max_intensity : intensity;
  
            float time = 0;
            if (timing_offsets.size())
              time = timing_offsets[block][firing * 16 + dsr] + time_diff_start_to_this_packet;

            data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, distance, intensity, time);
          }
        }
        data.newLine();
      }
    }
  }
} // namespace velodyne_rawdata
