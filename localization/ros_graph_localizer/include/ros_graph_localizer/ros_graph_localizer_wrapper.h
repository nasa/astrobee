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
#ifndef ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_WRAPPER_H_
#define ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_WRAPPER_H_

#include <ff_msgs/EkfState.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/GraphVIOState.h>
#include <ff_msgs/GraphLocState.h>
#include <ff_msgs/VisualLandmarks.h>
// #include <ff_msgs/SerializedGraph.h>
#include <graph_localizer/graph_localizer.h>
#include <imu_integration/imu_integrator.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/imu_measurement.h>
#include <localization_common/pose_interpolater.h>
#include <localization_common/timestamped_set.h>
#include <ros_graph_localizer/ros_graph_localizer_wrapper_params.h>

#include <sensor_msgs/Imu.h>

#include <memory>
#include <string>

namespace ros_graph_localizer {
// Converts ROS messages and passes these to the GraphLocalizer graph.
// Creates msgs from pose nodes in the graph.
// Waits until a valid map-match is received to initialize the localizer.
class RosGraphLocalizerWrapper {
 public:
  explicit RosGraphLocalizerWrapper(const std::string& graph_config_path_prefix = "");

  // Load configs for graph_localizer
  void LoadConfigs(const std::string& graph_config_path_prefix);

  // Store IMU msgs for world_T_dock estimation
  void ImuCallback(const sensor_msgs::Imu& imu_msg);

  // Add flight mode msg to IMU filter.
  void FlightModeCallback(const ff_msgs::FlightMode& flight_mode);

  // Add sparse map visual landmarks msg to graph_localizer.
  void SparseMapVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg);

  // Add AR tag visual landmarks msg to graph_localizer.
  void ARVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg);

  // Add graph vio state to graph localizer.
  bool GraphVIOStateCallback(const ff_msgs::GraphVIOState& graph_vio_state_msg);

  // Updates the graph_localizer if it has been initialized.
  void Update();

  // Returns whether the graph_localizer has been initialized.
  bool Initialized() const;

  // Resets the graph.
  void ResetLocalizer();

  // Resets the world_T_dock frame relative transform.
  void ResetWorldTDock();

  // Returns the latest timestamp in the graph if it exists.
  boost::optional<localization_common::Time> LatestTimestamp() const;

  // Returns the latest pose in the graph if it exists.
  boost::optional<gtsam::Pose3> LatestPose() const;

  // Returns latest world_T_dock if it exists.
  boost::optional<gtsam::Pose3> WorldTDock() const;

  // Creates graph loc state msg using latest pose and graph information
  // in graph localizer.
  // Returns boost::none if no state is available or no changes have occured since last msg.
  boost::optional<ff_msgs::GraphLocState> GraphLocStateMsg();

  // Accessor to graph localizer
  std::unique_ptr<graph_localizer::GraphLocalizer>& graph_localizer();

  // Const accessor to graph localizer
  const std::unique_ptr<graph_localizer::GraphLocalizer>& graph_localizer() const;

 private:
  // Initialize the graph
  void Initialize();

  std::unique_ptr<graph_localizer::GraphLocalizer> graph_localizer_;
  std::unique_ptr<imu_integration::ImuIntegrator> imu_integrator_;
  localization_common::PoseInterpolater odom_interpolator_;
  graph_localizer::GraphLocalizerParams params_;
  RosGraphLocalizerWrapperParams wrapper_params_;
  localization_common::TimestampedSet<ff_msgs::GraphVIOState> vio_measurement_buffer_;
  boost::optional<localization_common::CombinedNavState> latest_vio_state_;
  boost::optional<localization_common::Time> latest_msg_time_;
  boost::optional<localization_common::Time> last_vio_msg_time_;
  boost::optional<gtsam::Pose3> world_T_dock_;
  int latest_num_detected_ml_features_;
  int latest_num_detected_ar_features_;
};
}  // namespace ros_graph_localizer

#endif  // ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_WRAPPER_H_
