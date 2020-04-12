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

#ifndef VISUALEYEZ_VISUALEYEZ_H_
#define VISUALEYEZ_VISUALEYEZ_H_

// ROS includes
#include <ros/ros.h>

// ROS messages
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

// FSW tools
#include <config_reader/config_reader.h>

// FSW mesages
#include <ff_msgs/VisualeyezData.h>
#include <ff_msgs/VisualeyezDataArray.h>
#include <ff_msgs/VisualeyezCalibration.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Boost includes
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>

// STL C++ includes
#include <string>
#include <utility>
#include <vector>
#include <map>
#include <fstream>

// Topic names
#define TOPIC_VISUALEYEZ_DATA                 "/visualeyez/data"
#define TOPIC_VISUALEYEZ_GROUNDING            "/visualeyez/grounding"
#define SERVICE_VISUALEYEZ_CALIBRATE_CONFIG   "/visualeyez/calibrate/config"
#define TOPIC_VISUALEYEZ_CALIBRATE_FEEDBACK   "/visualeyez/calibrate/feedback"
#define TOPIC_VISUALEYEZ_RVIZ_GROUNDING       "/visualeyez/rviz/grounding"
#define TOPIC_VISUALEYEZ_RVIZ_MARKERS         "/visualeyez/rviz/markers"

// Test calibration file
#define TEST_CALIBRATION_FILE                 "test.bin"
#define TEST_CALIBRATION_DURATION_SECS        60.0
#define TEST_CALIBRATION_RATE_HZ              10.0
#define CALIBRATION_XRADS_PER_SEC             0.01
#define CALIBRATION_YRADS_PER_SEC             0.02
#define CALIBRATION_ZRADS_PER_SEC             0.03
#define EPSILON                               1e-6

// Synthetic room dimensions
#define AREA_WIDTH                            4.0
#define AREA_LENGTH                           8.0
#define AREA_HEIGHT                           2.0
#define ROBOT_WIDTH                           0.3
#define ROBOT_LENGTH                          0.3
#define ROBOT_HEIGHT                          0.3

// Header of doubles
#define HEADER_DATE       0
#define HEADER_LEN        1

// Packet of doubles
#define PACKET_TCMID      0
#define PACKET_LEDID      1
#define PACKET_POS_X      2
#define PACKET_POS_Y      3
#define PACKET_POS_Z      4
#define PACKET_LEN        5

// Max number of packets / measurement
#define MAX_NUM_PACKETS   512

// Maximum buffer size
#define MAX_BUFFER_SIZE (HEADER_LEN + MAX_NUM_PACKETS * PACKET_LEN)

using udp = boost::asio::ip::udp;
namespace fs  = boost::filesystem;

/**
 * \ingroup tools
 */
namespace visualeyez {

// Data structure for storing matches
typedef std::pair < uint8_t, uint8_t > VZIndex;
typedef std::vector < VZIndex > VZMatches;
typedef std::map < std::string, VZMatches > VZTargetMatches;

// Data structure for storing markers
typedef std::map < VZIndex, Eigen::Vector3d > VZMarkers;
typedef std::map < std::string, VZMarkers > VZTargetMarkers;

// Utility functions for the Visualeyez tracking system
class VisualeyezUtils {
 public:
  // Helper function to see if an LED value is valid.
  static bool IsValidMarkerValue(geometry_msgs::Vector3 const& pt);

  // Helper function for grabbing marker information from a LUA config file
  static VZMarkers GetMarkers(config_reader::ConfigReader::Table *table);

  // Write calibration data to a binary file for a given set of targets
  static bool WriteConfig(std::string file_name, VZTargetMarkers & targets);

  // Read calibration data from a binary file for a given set of targets
  static bool ReadConfig(std::string file_name, VZTargetMarkers & targets);

  // This algorithm solves the Procrustes problem in that it finds an affine transform
  // (rotation, translation, scale) that maps the "in" matrix to the "out" matrix
  // Code from: https://github.com/oleg-alexandrov/projects/blob/master/eigen/Kabsch.cpp
  // License is that this code is release in the public domain... Thanks, Oleg :)
  template <typename T>
  static bool Kabsch(Eigen::Matrix<T, 3, Eigen::Dynamic> in, Eigen::Matrix<T, 3, Eigen::Dynamic> out,
    Eigen::Transform<T, 3, Eigen::Affine> &A, bool allowScale) {
    // Default output
    A.linear() = Eigen::Matrix<T, 3, 3>::Identity(3, 3);
    A.translation() = Eigen::Matrix<T, 3, 1>::Zero();
    // A simple check to see that we have a sufficient number of correspondences
    if (in.cols() < 4) {
      // ROS_WARN("Visualeyez needs to see at least four LEDs to track");
      return false;
    }
    // A simple check to see that we have a sufficient number of correspondences
    if (in.cols() != out.cols()) {
      // ROS_ERROR("Same number of points required in input matrices");
      return false;
    }
    // First find the scale, by finding the ratio of sums of some distances,
    // then bring the datasets to the same scale.
    T dist_in = T(0.0), dist_out = T(0.0);
    for (int col = 0; col < in.cols()-1; col++) {
      dist_in  += (in.col(col+1) - in.col(col)).norm();
      dist_out += (out.col(col+1) - out.col(col)).norm();
    }
    if (dist_in <= T(0.0) || dist_out <= T(0.0))
      return true;
    T scale = T(1.0);
    if (allowScale) {
      scale = dist_out/dist_in;
      out /= scale;
    }
    // Find the centroids then shift to the origin
    Eigen::Matrix<T, 3, 1> in_ctr = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> out_ctr = Eigen::Matrix<T, 3, 1>::Zero();
    for (int col = 0; col < in.cols(); col++) {
      in_ctr  += in.col(col);
      out_ctr += out.col(col);
    }
    in_ctr /= T(in.cols());
    out_ctr /= T(out.cols());
    for (int col = 0; col < in.cols(); col++) {
      in.col(col)  -= in_ctr;
      out.col(col) -= out_ctr;
    }
    // SVD
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Cov = in * out.transpose();
    Eigen::JacobiSVD < Eigen::Matrix < T, Eigen::Dynamic, Eigen::Dynamic > > svd(Cov,
      Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Find the rotation
    T d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
    if (d > T(0.0))
      d = T(1.0);
    else
      d = T(-1.0);
    Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity(3, 3);
    I(2, 2) = d;
    Eigen::Matrix<T, 3, 3> R = svd.matrixV() * I * svd.matrixU().transpose();
    // The final transform
    A.linear() = scale * R;
    A.translation() = scale*(out_ctr - R*in_ctr);
    // Success
    return true;
  }
};

class VisualeyezClient {
 public:
  // Constructor
  VisualeyezClient();

  // Destructor
  ~VisualeyezClient();

  // Sends a single message
  void Send(ff_msgs::VisualeyezDataArray const& msg);

 protected:
  // Called when message is sent
  void SendCallback(const boost::system::error_code& error, std::size_t bytes_transferred);

 private:
  boost::thread thread_;                              /*!< Service thread.    */
  boost::asio::io_service io_service_;                /*!< ASIO service.      */
  udp::socket socket_;                                /*!< Network socket.    */
  udp::endpoint remote_endpoint_;                     /*!< Remote endpoint.   */
  boost::array<double, MAX_BUFFER_SIZE> tx_buffer_;    /*!< Receive buffer     */
};

}  // namespace visualeyez

#endif  // VISUALEYEZ_VISUALEYEZ_H_
