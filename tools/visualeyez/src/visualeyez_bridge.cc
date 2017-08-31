/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef VISUALEYEZ_VISUALEYEZ_BRIDGE_H_
#define VISUALEYEZ_VISUALEYEZ_BRIDGE_H_

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Boost includes
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

// FSW includes
#include <config_reader/config_reader.h>

// FSW messages
#include <ff_msgs/VisualeyezData.h>
#include <ff_msgs/VisualeyezDataArray.h>

// Visualeyez includes
#include <visualeyez/visualeyez.h>

// STL includes
#include <string>

using udp = boost::asio::ip::udp;

/**
 * \ingroup tools
 */
namespace visualeyez {

//!  A class for receiving UDP visualeyez measurements
/*!
  To be written
*/
class VisualeyezBridge : public nodelet::Nodelet {
 public:
  // Constructor
  VisualeyezBridge() : socket_(io_service_) {
    // Extract the port from the config file
    std::string addr; unsigned int port;
    config_reader::ConfigReader config_params;
    config_params.AddFile("tools/visualeyez.config");
    if (!config_params.ReadFiles())
      ROS_FATAL("Couldn't read config file");
    if (!config_params.GetUInt("server_port", &port))
      ROS_FATAL("Server port not specified in config file.");
    if (!config_params.GetStr("server_addr", &addr))
      ROS_FATAL("Server address not specified in config file.");
    // Open the socket
    socket_ = udp::socket(io_service_, udp::endpoint(udp::v4(), port));
    // Start listening
    Listen();
  }

  // Destructor
  virtual ~VisualeyezBridge() {}

  // Start listening for incoming packets
  void Listen() {
    socket_.async_receive_from(
      boost::asio::buffer(rx_buffer_, MAX_BUFFER_SIZE), rx_endpoint_,
      boost::bind(&VisualeyezBridge::Receive, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  }

 protected:
  // Called when ROS is loaded and ready
  void onInit() {
    // Setup the publisher before be start the ASIO service.
    pub_ = getNodeHandle().advertise<ff_msgs::VisualeyezDataArray>(
      TOPIC_VISUALEYEZ_DATA, 1);
    // Now that the publisher is ready we can start the ASIO run service.
    thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
  }

  //  Presumably the messages arrive in checksummed packets, and these packets get reassembled in order. Since
  //  we know each LED measurement has a specific size, we can determine the size of the measurement matrix by
  //  just looking at the size of the packet
  void Receive(const boost::system::error_code& error, std::size_t size) {
    if (!error && size % (PACKET_LEN * sizeof(double)) == (HEADER_LEN * sizeof(double))) {
      size_t n = size / (PACKET_LEN * sizeof(double));
      msg_.header.stamp = ros::Time(rx_buffer_[HEADER_DATE]);
      msg_.measurements.resize(n);
      for (size_t i = 0; i < n; i++) {
        msg_.measurements[i].tcmid = static_cast<uint8_t>(rx_buffer_[HEADER_LEN+i*PACKET_LEN+PACKET_TCMID]);
        msg_.measurements[i].ledid = static_cast<uint8_t>(rx_buffer_[HEADER_LEN+i*PACKET_LEN+PACKET_LEDID]);
        msg_.measurements[i].position.x = rx_buffer_[HEADER_LEN+i*PACKET_LEN+PACKET_POS_X];
        msg_.measurements[i].position.y = rx_buffer_[HEADER_LEN+i*PACKET_LEN+PACKET_POS_Y];
        msg_.measurements[i].position.z = rx_buffer_[HEADER_LEN+i*PACKET_LEN+PACKET_POS_Z];
      }
      pub_.publish(msg_);
    } else {
      ROS_WARN_STREAM("Malformed packet of " << size << " bytes");
    }
    // Restart listening
    Listen();
  }

 private:
  boost::thread thread_;                              /*!< Service thread.    */
  boost::asio::io_service io_service_;                /*!< ASIO service.      */
  udp::socket socket_;                                /*!< Network socket.    */
  udp::endpoint rx_endpoint_;                         /*!< Remote endpoint.   */
  boost::array<double, MAX_BUFFER_SIZE> rx_buffer_;    /*!< Receive buffer     */
  ros::Publisher pub_;                                /*!< ROS publisher      */
  ff_msgs::VisualeyezDataArray msg_;                  /*!< ROS message        */
};

// Register the nodelet with the system
PLUGINLIB_DECLARE_CLASS(visualeyez, VisualeyezBridge,
                        visualeyez::VisualeyezBridge, nodelet::Nodelet);

}  // namespace visualeyez

#endif  // VISUALEYEZ_VISUALEYEZ_BRIDGE_H_
