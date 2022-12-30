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
#ifndef GRAPH_VIO_GRAPH_VIO_WRAPPER_H_
#define GRAPH_VIO_GRAPH_VIO_WRAPPER_H_

#include <ff_msgs/GraphState.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/VIOGraph.h>
#include <graph_vio/feature_counts.h>
#include <graph_vio/graph_vio.h>
#include <graph_vio/graph_vio_initializer.h>
#include <graph_vio/graph_vio_stats.h>
#include <graph_vio/sanity_checker.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/imu_measurement.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

#include <string>
#include <utility>
#include <vector>

namespace graph_vio {
// Handles initialization of parameters, biases, and initial pose for graph
// VIO.  Provides callbacks that can be used by a ROS or non-ROS system
// (i.e. graph_bag, which does not use a ROS core, vs. graph_vio_nodelet,
// which is used when running live).
class GraphVIOWrapper {
 public:
  explicit GraphVIOWrapper(const std::string& graph_config_path_prefix = "");

  // Assumes previous bias estimates are available and uses these.
  void ResetVIO();

  void ResetBiasesAndVIO();

  void ResetBiasesFromFileAndResetVIO();

  boost::optional<localization_common::CombinedNavState> LatestCombinedNavState() const;

  boost::optional<ff_msgs::GraphVIOState> LatestVIOStateMsg();

  boost::optional<ff_msgs::Graph> LatestGraphMsg() const;

  bool Initialized() const;

  void Update();

  void OpticalFlowCallback(const ff_msgs::Feature2dArray& feature_array_msg);

  void ImuCallback(const sensor_msgs::Imu& imu_msg);

  void FlightModeCallback(const ff_msgs::FlightMode& flight_mode);

  boost::optional<const FeatureTrackIdMap&> feature_tracks() const;

  boost::optional<const GraphVIO&> graph_vio() const;

  void SaveGraphDotFile() const;

  boost::optional<const GraphVIOStats&> graph_stats() const;

  bool publish_graph() const;

  bool save_graph_dot_file() const;

 private:
  void InitializeGraph();

  bool CheckPoseSanity(const gtsam::Pose3& pose, const localization_common::Time timestamp) const;

  bool CheckCovarianceSanity() const;

  std::unique_ptr<GraphVIO> graph_vio_;
  // TODO(rsoussan): Make graph vio wrapper params
  bool publish_graph_;
  bool save_graph_dot_file_;
  boost::optional<gtsam::imuBias::ConstantBias> latest_biases_;
  GraphVIOInitializer graph_vio_initializer_;
  FeatureCounts feature_counts_;
  std::unique_ptr<SanityChecker> sanity_checker_;
  double position_cov_log_det_lost_threshold_;
  double orientation_cov_log_det_lost_threshold_;
  localization_measurements::FanSpeedMode fan_speed_mode_;
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_GRAPH_VIO_WRAPPER_H_
