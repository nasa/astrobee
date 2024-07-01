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
#ifndef ROS_GRAPH_VIO_ROS_GRAPH_VIO_WRAPPER_H_
#define ROS_GRAPH_VIO_ROS_GRAPH_VIO_WRAPPER_H_

#include <ff_msgs/DepthOdometry.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/GraphVIOState.h>
// #include <ff_msgs/SerializedGraph.h>
#include <graph_vio/graph_vio.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/imu_measurement.h>
#include <ros_graph_vio/imu_bias_initializer.h>
#include <ros_graph_vio/ros_graph_vio_wrapper_params.h>

#include <sensor_msgs/Imu.h>

#include <string>
#include <utility>

namespace ros_graph_vio {
// Converts ROS messages and passes these to the GraphVIO graph.
// Initializes the biases fro the graph using IMU messages.
// Creates msgs from latest combined nav states in the graph.
class RosGraphVIOWrapper {
 public:
  explicit RosGraphVIOWrapper(const std::string& graph_config_path_prefix = "");

  // Load configs for graph_vio and IMU bias initializer.
  void LoadConfigs(const std::string& graph_config_path_prefix);

  // Add feature points msg to graph_vio.
  void FeaturePointsCallback(const ff_msgs::Feature2dArray& feature_array_msg);

  // Add imu msg to graph_vio and IMU bias initializer if necessary.
  // If the graph hasn't been initialized and an imu bias is
  // available in the initializer, initializes the graph.
  void ImuCallback(const sensor_msgs::Imu& imu_msg);

  // Add depth odometry msg to graph_vio.
  void DepthOdometryCallback(const ff_msgs::DepthOdometry& depth_odometry_msg);

  // Add flight mode msg to graph_vio and IMU bias initializer.
  void FlightModeCallback(const ff_msgs::FlightMode& flight_mode);

  // Updates the graph_vio if it has been initialized.
  void Update();

  // Returns whether the graph_vio has been initialized.
  bool Initialized() const;

  // Resets the graph with the latest biases.
  void ResetVIO();

  // Resets the graph and biases. Biases need to be estimated again by the bias initializer.
  void ResetBiasesAndVIO();

  // Resets the graph and and loads biases from a saved file.
  void ResetBiasesFromFileAndResetVIO();

  // Creates a GraphVIOState msg using the history
  // of nav states and covariances in graph_vio.
  // Returns boost::none if no states available or no changes
  // have occured since last msg.
  boost::optional<ff_msgs::GraphVIOState> GraphVIOStateMsg();

  // Const accessor to graph_vio object
  const std::unique_ptr<graph_vio::GraphVIO>& graph_vio() const;

  // Accessor to graph_vio object
  std::unique_ptr<graph_vio::GraphVIO>& graph_vio();

 private:
  // Initialize the graph if the IMU bias is initialized.
  void Initialize();

  std::unique_ptr<graph_vio::GraphVIO> graph_vio_;
  std::unique_ptr<ImuBiasInitializer> imu_bias_initializer_;
  graph_vio::GraphVIOParams params_;
  boost::optional<localization_common::Time> latest_msg_time_;
  RosGraphVIOWrapperParams wrapper_params_;
  int feature_point_count_;
};
}  // namespace ros_graph_vio

#endif  // ROS_GRAPH_VIO_ROS_GRAPH_VIO_WRAPPER_H_
