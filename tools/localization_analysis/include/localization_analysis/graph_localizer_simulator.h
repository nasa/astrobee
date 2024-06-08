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

#ifndef LOCALIZATION_ANALYSIS_GRAPH_LOCALIZER_SIMULATOR_H_
#define LOCALIZATION_ANALYSIS_GRAPH_LOCALIZER_SIMULATOR_H_

#include <ff_msgs/GraphVIOState.h>
#include <ff_msgs/VisualLandmarks.h>
#include <localization_analysis/graph_localizer_simulator_params.h>
#include <localization_common/time.h>
#include <ros_graph_localizer/ros_graph_localizer_wrapper.h>

#include <string>
#include <vector>

namespace localization_analysis {
// Buffers msgs and passes these to the graph localizer after a simluated
// optimization time occurs (as set in the params). Enables a set optimization time
// to be simulated regardless of the hardware used for offline replay.
class GraphLocalizerSimulator : public ros_graph_localizer::RosGraphLocalizerWrapper {
 public:
  GraphLocalizerSimulator(const GraphLocalizerSimulatorParams& params, const std::string& graph_config_path_prefix);

  void BufferImuMsg(const sensor_msgs::Imu& imu_msg);

  void BufferFlightModeMsg(const ff_msgs::FlightMode& flight_mode_msg);

  void BufferGraphVIOStateMsg(const ff_msgs::GraphVIOState& graph_vio_state_msg);

  void BufferVLVisualLandmarksMsg(const ff_msgs::VisualLandmarks& visual_landmarks_msg);

  void BufferARVisualLandmarksMsg(const ff_msgs::VisualLandmarks& visual_landmarks_msg);

  bool AddMeasurementsAndUpdateIfReady(const localization_common::Time& current_time);

 private:
  std::vector<sensor_msgs::Imu> imu_msg_buffer_;
  std::vector<ff_msgs::FlightMode> flight_mode_msg_buffer_;
  std::vector<ff_msgs::GraphVIOState> vio_msg_buffer_;
  std::vector<ff_msgs::VisualLandmarks> vl_msg_buffer_;
  std::vector<ff_msgs::VisualLandmarks> ar_msg_buffer_;
  boost::optional<localization_common::Time> last_update_time_;
  GraphLocalizerSimulatorParams params_;
};
}  // namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_GRAPH_LOCALIZER_SIMULATOR_H_
