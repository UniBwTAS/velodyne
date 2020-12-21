// Copyright (C) 2007, 2009, 2010, 2015 Austin Robot Technology, Patrick Beeson, Jack O'Quin
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

/** \file
 *
 *  Input classes for the Velodyne HDL-64E 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <velodyne_driver/input.h>
#include <velodyne_driver/time_conversion.hpp>

namespace velodyne_driver
{
  static const size_t packet_size =
    sizeof(velodyne_msgs::VelodynePacket().data);

  ////////////////////////////////////////////////////////////////////////
  // Input base class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number.
   */
  Input::Input(ros::NodeHandle private_nh, uint16_t port):
    private_nh_(private_nh),
    port_(port)
  {
    private_nh.param("device_ip", devip_str_, std::string(""));
    private_nh.param("gps_time", gps_time_, false);
    if (!devip_str_.empty())
      ROS_INFO_STREAM("Only accepting packets from IP address: "
                      << devip_str_);
  }

  ////////////////////////////////////////////////////////////////////////
  // InputSocket class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   */
  InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port):
    Input(private_nh, port)
  {
    sockfd_ = -1;

    if (!devip_str_.empty()) {
      inet_aton(devip_str_.c_str(),&devip_);
    }

    // connect to Velodyne UDP port
    ROS_INFO_STREAM("Opening UDP socket: port " << port);
    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd_ == -1)
      {
        perror("socket");               // TODO: ROS_ERROR errno
        return;
      }

    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(port);          // port in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

    if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
      {
        perror("bind");                 // TODO: ROS_ERROR errno
        return;
      }

    if (fcntl(sockfd_,F_SETFL, O_NONBLOCK|FASYNC) < 0)
      {
        perror("non-block");
        return;
      }

    ROS_DEBUG("Velodyne socket fd is %d\n", sockfd_);
  }

  /** @brief destructor */
  InputSocket::~InputSocket(void)
  {
    (void) close(sockfd_);
  }

  /** @brief Get one velodyne packet. */
  int InputSocket::getPacket(velodyne_msgs::VelodynePacket *pkt, const double time_offset)
  {
    double time1 = ros::Time::now().toSec();

    struct pollfd fds[1];
    fds[0].fd = sockfd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
      {
        // Unfortunately, the Linux kernel recvfrom() implementation
        // uses a non-interruptible sleep() when waiting for data,
        // which would cause this method to hang if the device is not
        // providing data.  We poll() the device first to make sure
        // the recvfrom() will not block.
        //
        // Note, however, that there is a known Linux kernel bug:
        //
        //   Under Linux, select() may report a socket file descriptor
        //   as "ready for reading", while nevertheless a subsequent
        //   read blocks.  This could for example happen when data has
        //   arrived but upon examination has wrong checksum and is
        //   discarded.  There may be other circumstances in which a
        //   file descriptor is spuriously reported as ready.  Thus it
        //   may be safer to use O_NONBLOCK on sockets that should not
        //   block.

        // poll() until input available
        do
          {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0)             // poll() error?
              {
                if (errno != EINTR)
                  ROS_ERROR("poll() error: %s", strerror(errno));
                return -1;
              }
            if (retval == 0)            // poll() timeout?
              {
                ROS_WARN("Velodyne poll() timeout");
                return -1;
              }
            if ((fds[0].revents & POLLERR)
                || (fds[0].revents & POLLHUP)
                || (fds[0].revents & POLLNVAL)) // device error?
              {
                ROS_ERROR("poll() reports Velodyne error");
                return -1;
              }
          } while ((fds[0].revents & POLLIN) == 0);

        // Receive packets that should now be available from the
        // socket using a blocking read.
        ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0],
                                  packet_size,  0,
                                  (sockaddr*) &sender_address,
                                  &sender_address_len);

        if (nbytes < 0)
          {
            if (errno != EWOULDBLOCK)
              {
                perror("recvfail");
                ROS_INFO("recvfail");
                return -1;
              }
          }
        else if ((size_t) nbytes == packet_size)
          {
            // read successful,
            // if packet is not from the lidar scanner we selected by IP,
            // continue otherwise we are done
            if(devip_str_ != ""
               && sender_address.sin_addr.s_addr != devip_.s_addr)
              continue;
            else
              break; //done
          }

        ROS_DEBUG_STREAM("incomplete Velodyne packet read: "
                         << nbytes << " bytes");
      }

    if (!gps_time_) {
      // Average the times at which we begin and end reading.  Use that to
      // estimate when the scan occurred. Add the time offset.
      double time2 = ros::Time::now().toSec();
      pkt->stamp = ros::Time((time2 + time1) / 2.0 + time_offset);
    } else {
      // time for each packet is a 4 byte uint located starting at offset 1200 in
      // the data packet
      pkt->stamp = rosTimeFromGpsTimestamp(&(pkt->data[1200]));
    }

    return 0;
  }

  ////////////////////////////////////////////////////////////////////////
  // InputPCAP class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param filename PCAP dump file name
   */
  InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port, std::string filename, bool read_once, bool read_fast,
                       double repeat_delay)
    : Input(private_nh, port),
    packet_rate_(1),
    filename_(filename)
  {
    pcap_ = nullptr;
    empty_ = true;

    // get parameters using private node handle
    private_nh.param("pcap_velodyne_model", velodyne_model_, std::string("64E"));
    private_nh.param("pcap_read_once", read_once_, false);
    private_nh.param("pcap_read_fast", read_fast_, false);
    private_nh.param("pcap_repeat_delay", repeat_delay_, 0.0);
    // parse old parameters for compatibility (TODO: remove this line in a future release)
    parse_deprecated_parameters(private_nh);

    // get packet rate in order to play pcap file in real speed (if read_fast_ is false)
    packet_rate_ = obtainPacketRate();

    if (read_once_)
      ROS_INFO("Read input file only once.");
    if (read_fast_)
      ROS_INFO("Read input file as quickly as possible.");
    if (repeat_delay_ > 0.0)
      ROS_INFO("Delay %.3f seconds before repeating input file.", repeat_delay_);

    // Open the PCAP dump file
    ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
    if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL)
      {
        ROS_FATAL("Error opening Velodyne socket dump file.");
        return;
      }

    std::stringstream filter;
    if( devip_str_ != "" )              // using specific IP?
      {
        filter << "src host " << devip_str_ << " && ";
      }
    filter << "udp dst port " << port;
    pcap_compile(pcap_, &pcap_packet_filter_,
                 filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
  }

  /** destructor */
  InputPCAP::~InputPCAP(void)
  {
    pcap_close(pcap_);
  }

  /** @brief Get one velodyne packet. */
  int InputPCAP::getPacket(velodyne_msgs::VelodynePacket *pkt, const double time_offset)
  {
    struct pcap_pkthdr *header;
    const u_char *pkt_data;

    while (true)
      {
        int res;
        if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
          {
            // Skip packets not for the correct port and from the
            // selected IP address.
            if (0 == pcap_offline_filter(&pcap_packet_filter_,
                                          header, pkt_data))
              continue;

            // Keep the reader from blowing through the file.
            if (!read_fast_)
              packet_rate_.sleep();

            memcpy(&pkt->data[0], pkt_data+42, packet_size);
            pkt->stamp = ros::Time::now(); // time_offset not considered here, as no synchronization required
            empty_ = false;
            return 0;                   // success
          }

        if (empty_)                 // no data in file?
          {
            ROS_WARN("Error %d reading Velodyne packet: %s",
                     res, pcap_geterr(pcap_));
            return -1;
          }

        if (read_once_)
          {
            ROS_INFO("end of file reached -- done reading.");
            return -1;
          }

        if (repeat_delay_ > 0.0)
          {
            ROS_INFO("end of file reached -- delaying %.3f seconds.",
                     repeat_delay_);
            usleep(rint(repeat_delay_ * 1000000.0));
          }

        ROS_DEBUG("replaying Velodyne dump file");

        // I can't figure out how to rewind the file, because it
        // starts with some kind of header.  So, close the file
        // and reopen it with pcap.
        pcap_close(pcap_);
        pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
        empty_ = true;              // maybe the file disappeared?
      } // loop back and try again
  }

  double InputPCAP::obtainPacketRate()
  {
    auto model = velodyne_model_;

    if ((model == "VLS128"))  //  3 firing cycles in a data packet. 3 x 53.3 μs = 0.1599 ms is the accumulation delay
                              //  per packet. 1 packet/0.1599 ms = 6253.9 packets/second
    {
      return 6253.9;
    }
    else if ((model == "64E_S2") || (model == "64E_S2.1"))  // generates 1333312 points per second
    {                                                       // 1 packet holds 384 points
      return 3472.17;                                       // 1333312 / 384
    }
    else if (model == "64E")
    {
      return 2600.0;
    }
    else if (model == "64E_S3")  // generates 2222220 points per second (half for strongest and half for lastest)
    {                            // 1 packet holds 384 points
      return 5787.03;            // 2222220 / 384
    }
    else if (model == "32E")
    {
      return 1808.0;
    }
    else if (model == "32C")
    {
      return 1507.0;
    }
    else if (model == "VLP16")
    {
      return 754;  // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
    }
    else
    {
      ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << model);
      return 2600.0;
    }
  }

  void InputPCAP::parse_deprecated_parameters(const ros::NodeHandle& private_nh)
  {
    // This method is designed in a way that it can be removed without breaking the new code (using the new parameters).

    // deprecation warnings were already printed in driver.cc so here we silently use the deprecated parameters.
    std::string model;
    if(private_nh.getParam("model", model)) {
      velodyne_model_ = model;
    }
    bool read_once;
    if(private_nh.getParam("read_once", read_once)) {
      read_once_ = read_once;
    }
    bool read_fast;
    if(private_nh.getParam("read_fast", read_fast)) {
      read_fast_ = read_fast;
    }
    double repeat_delay;
    if(private_nh.getParam("repeat_delay", repeat_delay)) {
      repeat_delay_ = repeat_delay;
    }
  }

  }  // namespace velodyne_driver
