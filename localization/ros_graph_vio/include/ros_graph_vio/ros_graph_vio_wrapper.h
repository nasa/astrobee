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

#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/GraphVIOState.h>
#include <ff_msgs/SerializedGraph.h>
#include <graph_vio/graph_vio.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/imu_measurement.h>
#include <ros_graph_vio/imu_bias_initializer.h>

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

  // Gets the latest combined nav state from the graph.
  // boost::optional<localization_common::CombinedNavState> LatestCombinedNavState() const;

  // Creates a vio state msg using the latest combined nav state from the graph.
  // boost::optional<ff_msgs::GraphVIOState> LatestVIOStateMsg();

  // Creates a graph msg using the latest combined nav state from the graph.
  // boost::optional<ff_msgs::SerializedGraph> LatestGraphMsg() const;

  // boost::optional<const vision_common::FeatureTrackIdMap&> feature_tracks() const;

  // boost::optional<const GraphVIO&> graph_vio() const;

  // void SaveGraphDotFile() const;

  // bool publish_graph() const;

  // bool save_graph_dot_file() const;

 private:
  // Initialize the graph if the IMU bias is initialized.
  void Initialize();

  std::unique_ptr<graph_vio::GraphVIO> graph_vio_;
  graph_vio::GraphVIOParams params_;
  ImuBiasInitializer imu_bias_initializer_;
  // TODO(rsoussan): Make graph vio wrapper params
  // bool publish_graph_;
  // bool save_graph_dot_file_;
  // boost::optional<gtsam::imuBias::ConstantBias> latest_biases_;
};
}  // namespace ros_graph_vio

#endif  // ROS_GRAPH_VIO_ROS_GRAPH_VIO_WRAPPER_H_
