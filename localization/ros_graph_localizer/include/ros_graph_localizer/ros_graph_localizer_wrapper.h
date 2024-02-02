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

#include <ff_msgs/GraphVIOState.h>
#include <ff_msgs/GraphLocState.h>
#include <ff_msgs/VisualLandmarks.h>
// #include <ff_msgs/SerializedGraph.h>
#include <graph_localizer/graph_localizer.h>
#include <localization_common/timestamped_set.h>

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

  // Add sparse map visual landmarks msg to graph_localizer.
  void SparseMapVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg);

  // Add graph vio state to graph localizer.
  bool GraphVIOStateCallback(const ff_msgs::GraphVIOState& graph_vio_state_msg);

  // Updates the graph_localizer if it has been initialized.
  void Update();

  // Returns whether the graph_localizer has been initialized.
  bool Initialized() const;

  // Resets the graph.
  void ResetLocalizer();

  // Creates graph loc state msg using latest pose and graph information
  // in graph localizer.
  // Returns boost::none if no state is available or no changes have occured since last msg.
  boost::optional<ff_msgs::GraphLocState> GraphLocStateMsg();

 private:
  // Initialize the graph
  void Initialize();

  std::unique_ptr<graph_localizer::GraphLocalizer> graph_localizer_;
  graph_localizer::GraphLocalizerParams params_;
  localization_common::TimestampedSet<ff_msgs::GraphVIOState> vio_measurement_buffer_;
  boost::optional<localization_common::Time> last_vio_msg_time_;
  boost::optional<localization_common::Time> latest_msg_time_;
};
}  // namespace ros_graph_localizer

#endif  // ROS_GRAPH_LOCALIZER_ROS_GRAPH_LOCALIZER_WRAPPER_H_