// Copyright (C) 2007, 2009, 2010, 2012, 2019 Yaxin Liu, Patrick Beeson, Jack O'Quin, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 */

#ifndef VELODYNE_POINTCLOUD_RAWDATA_H
#define VELODYNE_POINTCLOUD_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>
#include <vector>
#include <bitset>

#include <ros/ros.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/calibration.h>
#include <velodyne_pointcloud/datacontainerbase.h>
#include <velodyne_pointcloud/return_types_flags.h>

namespace velodyne_rawdata
{
/**
 * Raw Velodyne packet constants and structures.
 */
static const int SIZE_BLOCK = 100;

static const float ROTATION_RESOLUTION = 0.01f;     // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;  // [deg/100]

static const int  PACKET_SIZE = 1206;
static const int  BLOCKS_PER_PACKET = 12;
static const int  PACKET_STATUS_SIZE = 4;
static const int  RAW_POINT_SIZE=  3; // A data point has a size of 3 bytes
static const int  AZIMUTH_SIZE=  2; // The azimuth per data block has size 2
static const int  FLAG_SIZE=  2; // The flag per data block has size 2
static const int  TIMESTAMP_SIZE=  4; // The time stamp data block has size 4
static const int  RETURN_MODE_BYTE_SIZE=  1; // The return mode is 1 byte
static const int  MODEL_BYTE_SIZE=  1; // The model id is 1 byte
static const int  POINTS_PER_BLOCK=  32; // A block has 32 data points
static const int  BLOCK_SIZE=  FLAG_SIZE+AZIMUTH_SIZE+(POINTS_PER_BLOCK*RAW_POINT_SIZE); // A block of data consist on a flag, an azimuth and 32 data points
static const int  BLOCK_DATA_SIZE = (POINTS_PER_BLOCK * RAW_POINT_SIZE);
static const int  POINTS_PER_PACKET = (POINTS_PER_BLOCK * BLOCKS_PER_PACKET);

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f;  // [µs]
static const float VLP16_DSR_TOFFSET = 2.304f;        // [µs]
static const float VLP16_FIRING_TOFFSET = 55.296f;    // [µs]
    static const int  VLP16_MODEL_ID=  34;

/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block
{
  uint16_t header;    ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
}
raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes
{
  uint16_t uint;
  uint8_t bytes[2];
};

/*Model names strings*/
static const std::string  VLP16    = "VLP16";
static const std::string  V32C = "32C";
static const std::string  V32E  = "32E";
static const std::string  VLS128  = "VLS128";


/** Special Definitions for VLS128 support **/
// These are used to detect which bank of 32 lasers is in this block
static const uint16_t VLS128_BANK_1 = 0xeeff;
static const uint16_t VLS128_BANK_2 = 0xddff;
static const uint16_t VLS128_BANK_3 = 0xccff;
static const uint16_t VLS128_BANK_4 = 0xbbff;

static const float  VLS128_CHANNEL_TDURATION  =  2.665f;  // [µs] Channels corresponds to one laser firing
static const float  VLS128_SEQ_TDURATION      =  53.3f;   // [µs] Sequence is a set of laser firings including recharging
static const float  VLS128_TOH_ADJUSTMENT    =  7.0f;   // [µs] μs. Top Of the Hour is aligned with the fourth firing group in a firing sequence.
static const float  VLS128_REST_PERIOD_TDURATION=  5.333f;  // [µs] Duration of rest period RP0 and RP1
static const float  VLS128_DISTANCE_RESOLUTION=  0.004f;  // [m]
static const int  VLS128_MODEL_ID=  161;
static const int  VLS128_BLOCKS_PER_FIRING_SEQ=  4; // A packet has 3 firing sequences and each one has 4 blocks of 32 individual laser firings
static const int VLS128_RETURN_MODE_STRONGEST= 55; // Codes for the possible return modes
static const int VLS128_RETURN_MODE_LAST= 56;
static const int VLS128_RETURN_MODE_DUAL= 57;
static const int VLS128_RETURN_MODE_DUAL_CONF= 59;
static const int VLS128_RETURN_MODE_POSITION= 1204;
static const int VLS128_MODEL_ID_POSITION= 1205;


static const float  PACKET_RATE_SINGLE_RET_MODE  = 6030.5;
static const float  PACKET_RATE_DUAL_RET_MODE  = 18091.36;


/** \brief Raw Velodyne packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
}
raw_packet_t;

typedef struct raw_packet_vls128
{
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint32_t time_stamp;
    uint8_t return_mode;
    uint8_t model_id;
}
raw_packet_vls128_t;

/** \brief Velodyne data conversion class */
class RawData
{
public:
  RawData();
  ~RawData()
  {
  }

  /** \brief Set up for data processing.
   *
   *  Perform initializations needed before data processing can
   *  begin:
   *
   *    - read device-specific angles calibration
   *
   *  @param private_nh private node handle for ROS parameters
   *  @returns an optional calibration
   */
  boost::optional<velodyne_pointcloud::Calibration> setup(ros::NodeHandle private_nh);


  void setupSinCosCache();
  void setupAzimuthCache();
  bool loadCalibration();

  /** \brief Set up for data processing offline.
   * Performs the same initialization as in setup, in the abscence of a ros::NodeHandle.
   * this method is useful if unpacking data directly from bag files, without passing
   * through a communication overhead.
   *
   * @param calibration_file path to the calibration file
   * @param max_range_ cutoff for maximum range
   * @param min_range_ cutoff for minimum range
   * @returns 0 if successful;
   *           errno value for failure
   */
  int setupOffline(std::string calibration_file, double max_range_, double min_range_);

  void unpack(const velodyne_msgs::VelodynePacket& pkt, DataContainerBase& data_container,
              const ros::Time& scan_start_time, const size_t packet_pos_in_scan=0);

  void setParameters(double min_range, double max_range, double view_direction, double view_width);

  int pointsPerPacket() const;

  unsigned int read_return_mode(const velodyne_msgs::VelodynePacket& pkt);

  std::string get_sensor_model(){return config_.model;}



private:
  /** configuration parameters */
  typedef struct
  {
    std::string model;
    std::string calibrationFile;  ///< calibration file name
    double max_range;             ///< maximum range to publish
    double min_range;             ///< minimum range to publish
    int min_angle;                ///< minimum angle to publish
    int max_angle;                ///< maximum angle to publish
    double rpm;
    double tmp_min_angle;
    double tmp_max_angle;
  }
  Config;
  Config config_;

  unsigned int current_return_mode{VLS128_RETURN_MODE_STRONGEST};

  /**
   * Calibration file
   */
  velodyne_pointcloud::Calibration calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];

  // Caches the azimuth percent offset for the VLS-128 laser firings
  float vls_128_laser_azimuth_cache[16];

  // timing offset lookup table
  std::vector< std::vector<float> > timing_offsets;

  // azimuth of the last package, used to estimate the laser correction due to rotation of the sensor in dual return mode
  uint16_t azimuth_previous_packet{0};

  /** \brief setup per-point timing offsets
   * 
   *  Runs during initialization and determines the firing time for each point in the scan
   * 
   *  NOTE: Does not support all sensors yet (vlp16, vlp32, and hdl32 are currently supported)
   */
  bool buildTimings();

  /** add private function to handle the VLP16 **/
  void unpack_vlp16(const velodyne_msgs::VelodynePacket& pkt, DataContainerBase& data_container,
                    const ros::Time& scan_start_time);

    void unpack_vls128(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase& data_container,
                     const ros::Time& scan_start_time, const size_t packet_pos_in_scan);

  /** in-line test whether a point is in range */
  inline bool pointInRange(float range)
  {
    return (range >= config_.min_range
            && range <= config_.max_range);
  }

    inline void calculate_and_add_point_to_container(const raw_block_t &block,
                                                     const int position,
                                                     const velodyne_pointcloud::LaserCorrection &corrections,
                                                     const uint16_t &azimuth_corrected,
                                                     const uint16_t &azimuth_rot_corrected,
                                                     const std::uint16_t &rotation_segment,
                                                     const uint16_t &firing_seq_in_scan,
                                                     const uint8_t &laser_number,
                                                     const uint8_t &return_type,
                                                     const float time,
                                                     DataContainerBase &data_container,
                                                     const bool add_invalid = false) {
        if (!add_invalid &&
            ((config_.min_angle < config_.max_angle && azimuth_rot_corrected >= config_.min_angle &&
              azimuth_rot_corrected <= config_.max_angle) ||
             (config_.min_angle > config_.max_angle &&
              (azimuth_rot_corrected >= config_.min_angle || azimuth_rot_corrected <= config_.max_angle)))) {

            union two_bytes tmp;

            // distance extraction
            tmp.bytes[0] = block.data[position];
            tmp.bytes[1] = block.data[position + 1];
            const float distance = tmp.uint * VLS128_DISTANCE_RESOLUTION;

            // convert polar coordinates to Euclidean XYZ
            const float cos_vert_angle = corrections.cos_vert_correction;
            const float sin_vert_angle = corrections.sin_vert_correction;
            const float cos_rot_correction = corrections.cos_rot_correction;
            const float sin_rot_correction = corrections.sin_rot_correction;

            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
            const float cos_rot_angle =
                    cos_rot_table_[azimuth_corrected] * cos_rot_correction +
                    sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            const float sin_rot_angle =
                    sin_rot_table_[azimuth_corrected] * cos_rot_correction -
                    cos_rot_table_[azimuth_corrected] * sin_rot_correction;

            // Compute the distance in the xy plane (w/o accounting for
            // rotation)
            const float xy_distance = distance * cos_vert_angle;

            data_container.addPoint(xy_distance * cos_rot_angle,
                                    -(xy_distance * sin_rot_angle),
                          distance * sin_vert_angle,
                                    corrections.laser_ring,
                                    azimuth_rot_corrected,
                                    distance,
                                    static_cast<float>(block.data[position + 2]),
                                    time,
                          corrections.laser_ring + calibration_.num_lasers * rotation_segment,
                                    rotation_segment,
                                    firing_seq_in_scan,
                                    laser_number,
                                    return_type
                                    );
        } else {
            // point is outside the valid angle range

            data_container.addPoint(nanf(""),
                                    nanf(""),
                                    nanf(""),
                                    corrections.laser_ring,
                                    azimuth_corrected,
                                    nanf(""),
                                    0.0,
                                    time,
                          corrections.laser_ring +
                          calibration_.num_lasers * rotation_segment,
                                    rotation_segment,
                                    firing_seq_in_scan,
                                    laser_number,
                                    return_type
                                    );
        }
    }

    inline void calculate_and_add_point_and_confidence_to_container(const raw_block_t &block,
                                                                    const uint8_t (&confidence_info)[3],
                                                     const int position,
                                                     const velodyne_pointcloud::LaserCorrection &corrections,
                                                     const uint16_t &azimuth_corrected,
                                                     const uint16_t &azimuth_rot_corrected,
                                                     const std::uint16_t &rotation_segment,
                                                     const uint16_t &firing_seq_in_scan,
                                                     const uint8_t &laser_number,
                                                     const uint8_t &return_type,
                                                     const float time,
                                                     DataContainerBase &data_container,
                                                     const bool add_invalid = false) {
        if (!add_invalid &&
            ((config_.min_angle < config_.max_angle && azimuth_rot_corrected >= config_.min_angle &&
              azimuth_rot_corrected <= config_.max_angle) ||
             (config_.min_angle > config_.max_angle &&
              (azimuth_rot_corrected >= config_.min_angle || azimuth_rot_corrected <= config_.max_angle)))) {

            union two_bytes tmp;

            // distance extraction
            tmp.bytes[0] = block.data[position];
            tmp.bytes[1] = block.data[position + 1];
            const float distance = tmp.uint * VLS128_DISTANCE_RESOLUTION;

            // convert polar coordinates to Euclidean XYZ
            const float cos_vert_angle = corrections.cos_vert_correction;
            const float sin_vert_angle = corrections.sin_vert_correction;
            const float cos_rot_correction = corrections.cos_rot_correction;
            const float sin_rot_correction = corrections.sin_rot_correction;

            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
            const float cos_rot_angle =
                    cos_rot_table_[azimuth_corrected] * cos_rot_correction +
                    sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            const float sin_rot_angle =
                    sin_rot_table_[azimuth_corrected] * cos_rot_correction -
                    cos_rot_table_[azimuth_corrected] * sin_rot_correction;

            // Compute the distance in the xy plane (w/o accounting for
            // rotation)
            const float xy_distance = distance * cos_vert_angle;

            // now get the confidence information from the confidence block according to the return flag

            uint8_t read_mask = 0;
            uint8_t shift = 0;
            // ((1 << fieldLength) - 1) << (fieldIndex - 1)


            uint8_t  drop = 0;
            uint8_t  retro_shadow = 0;
            uint8_t  range_limited = 0;
            uint8_t  retro_ghost = 0;
            uint8_t  interference = 0;
            uint8_t  sun = 0;
            uint8_t  confidence = 0;

            if(is_flag_set(return_type,FIRST_RETURN_FLAG))
            {
                // get info for first return
                //drop
                shift = (4 - 1);
                read_mask = ((1 << 1) - 1) << shift;
                drop = (confidence_info[1] & read_mask) >> shift;
                //retro_shadow
                shift = (2 - 1);
                read_mask = ((1 << 1) - 1) << shift;
                retro_shadow = (confidence_info[1] & read_mask) >> shift;
                //range_limited
                shift = (1 - 1);
                read_mask = ((1 << 1) - 1) << shift;
                range_limited = (confidence_info[1] & read_mask) >> shift;
                //retro_ghost
                shift = (8 - 1);
                read_mask = ((1 << 1) - 1) << shift;
                retro_ghost = (confidence_info[2] & read_mask) >> shift;
                //interference
                shift = (6 - 1);
                read_mask = ((1 << 2) - 1) << shift;
                interference = (confidence_info[2] & read_mask) >> shift;
                //sun level
                shift = (4 - 1);
                read_mask = ((1 << 2) - 1) << shift;
                sun = (confidence_info[2] & read_mask) >> shift;
                //confidence
                shift = (1 - 1);
                read_mask = ((1 << 3) - 1) << shift;
                confidence = (confidence_info[2] & read_mask) >> shift;

            }
            else
            {
                // get info for second return

                //drop
                shift = (8 - 1);
                read_mask = ((1 << 1) - 1) << shift;
                drop = (confidence_info[0] & read_mask) >> shift;
                //retro_shadow
                shift = (6 - 1);
                read_mask = ((1 << 1) - 1) << shift;
                retro_shadow = (confidence_info[0] & read_mask) >> shift;
                //range_limited
                shift = (5 - 1);
                read_mask = ((1 << 1) - 1) << shift;
                range_limited = (confidence_info[0] & read_mask) >> shift;
                //retro_ghost
                shift = (4 - 1);
                read_mask = ((1 << 1) - 1) << shift;
                retro_ghost = (confidence_info[0] & read_mask) >> shift;
                //interference
                shift = (2 - 1);
                read_mask = ((1 << 2) - 1) << shift;
                interference = (confidence_info[0] & read_mask) >> shift;
                //sun level msb
                shift = (1 - 1);
                read_mask = ((1 << 1) - 1) << shift;
                sun = (confidence_info[0] & read_mask) >> (shift-1);
                //sun level lsb
                shift = (8 - 1);
                read_mask = ((1 << 1) - 1) << shift;
                sun = sun & ((confidence_info[1] & read_mask) >> shift);
                //confidence
                shift = (5 - 1);
                read_mask = ((1 << 3) - 1) << shift;
                confidence = (confidence_info[1] & read_mask) >> shift;


            }

            data_container.addPoint_with_confidence(xy_distance * cos_rot_angle,
                                                    -(xy_distance * sin_rot_angle),
                          distance * sin_vert_angle,
                                                    corrections.laser_ring,
                                                    azimuth_rot_corrected,
                                                    distance,
                                                    static_cast<float>(block.data[position + 2]),
                                                    time,
                          corrections.laser_ring + calibration_.num_lasers * rotation_segment,
                                                    rotation_segment,
                                                    firing_seq_in_scan,
                                                    laser_number,
                                                    return_type,
                                                    drop,
                                                    retro_shadow,
                                                    range_limited,
                                                    retro_ghost,
                                                    interference,
                                                    sun,
                                                    confidence);
        } else {
            // point is outside the valid angle range

            data_container.addPoint_with_confidence(nanf(""),
                                                    nanf(""),
                                                    nanf(""),
                                                    corrections.laser_ring,
                                                    azimuth_corrected,
                                                    nanf(""),
                                                    0.0,
                                                    time,
                          corrections.laser_ring +
                          calibration_.num_lasers * rotation_segment,
                                                    rotation_segment,
                                                    firing_seq_in_scan,
                                                    laser_number,
                                                    return_type,
                                                    255,
                                                    255,
                                                    255,
                                                    255,
                                                    255,
                                                    255,
                                                    255);
        }

  }

};

}  // namespace velodyne_rawdata

#endif  // VELODYNE_POINTCLOUD_RAWDATA_H
