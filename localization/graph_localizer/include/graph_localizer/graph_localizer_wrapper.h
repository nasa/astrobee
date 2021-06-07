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
#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_WRAPPER_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_WRAPPER_H_

#include <ff_msgs/DepthLandmarks.h>
#include <ff_msgs/GraphState.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/LocalizationGraph.h>
#include <ff_msgs/VisualLandmarks.h>
#include <graph_localizer/feature_counts.h>
#include <graph_localizer/graph_localizer.h>
#include <graph_localizer/graph_localizer_initializer.h>
#include <graph_localizer/graph_localizer_stats.h>
#include <graph_localizer/sanity_checker.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/imu_measurement.h>
#include <localization_measurements/matched_projections_measurement.h>
#include <localization_measurements/timestamped_handrail_pose.h>
#include <localization_measurements/timestamped_pose.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

#include <string>
#include <utility>
#include <vector>

namespace graph_localizer {
// Handles initialization of parameters, biases, and initial pose for graph
// localizer.  Provides callbacks that can be used by a ROS or non-ROS system
// (i.e. graph_bag, which does not use a ROS core, vs. graph_localizer_nodelet,
// which is used when running live).
class GraphLocalizerWrapper {
 public:
  explicit GraphLocalizerWrapper(const std::string& graph_config_path_prefix = "");

  // Assumes previous bias estimates are available and uses these.
  void ResetLocalizer();

  void ResetBiasesAndLocalizer();

  void ResetBiasesFromFileAndResetLocalizer();

  boost::optional<geometry_msgs::PoseStamped> LatestSparseMappingPoseMsg() const;

  boost::optional<geometry_msgs::PoseStamped> LatestARTagPoseMsg() const;

  boost::optional<geometry_msgs::PoseStamped> LatestHandrailPoseMsg() const;

  boost::optional<localization_common::CombinedNavState> LatestCombinedNavState() const;

  boost::optional<ff_msgs::GraphState> LatestLocalizationStateMsg();

  boost::optional<ff_msgs::LocalizationGraph> LatestLocalizationGraphMsg() const;

  bool Initialized() const;

  void Update();

  void OpticalFlowCallback(const ff_msgs::Feature2dArray& feature_array_msg);

  void VLVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg);

  void ARVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg);

  void DepthLandmarksCallback(const ff_msgs::DepthLandmarks& depth_landmarks_msg);

  void ImuCallback(const sensor_msgs::Imu& imu_msg);

  void FlightModeCallback(const ff_msgs::FlightMode& flight_mode);

  boost::optional<const FeatureTrackIdMap&> feature_tracks() const;

  boost::optional<const GraphLocalizer&> graph_localizer() const;

  void MarkWorldTDockForResettingIfNecessary();

  void MarkWorldTHandrailForResetting();

  void ResetWorldTDockUsingLoc(const ff_msgs::VisualLandmarks& visual_landmarks_msg);

  void ResetWorldTHandrailIfNecessary(const ff_msgs::DepthLandmarks& depth_landmarks_msg);

  gtsam::Pose3 estimated_world_T_dock() const;

  boost::optional<localization_measurements::TimestampedHandrailPose> estimated_world_T_handrail() const;

  void SaveLocalizationGraphDotFile() const;

  boost::optional<const GraphLocalizerStats&> graph_localizer_stats() const;

  bool publish_localization_graph() const;

  bool save_localization_graph_dot_file() const;

 private:
  void InitializeGraph();

  bool CheckPoseSanity(const gtsam::Pose3& sparse_mapping_pose, const localization_common::Time timestamp) const;

  bool CheckCovarianceSanity() const;

  std::unique_ptr<GraphLocalizer> graph_localizer_;
  // TODO(rsoussan): Make graph localizer wrapper params
  bool publish_localization_graph_;
  bool save_localization_graph_dot_file_;
  boost::optional<gtsam::imuBias::ConstantBias> latest_biases_;
  GraphLocalizerInitializer graph_localizer_initializer_;
  FeatureCounts feature_counts_;
  boost::optional<localization_measurements::TimestampedPose> sparse_mapping_pose_;
  boost::optional<localization_measurements::TimestampedPose> ar_tag_pose_;
  boost::optional<localization_measurements::TimestampedHandrailPose> sensor_T_handrail_;
  std::unique_ptr<SanityChecker> sanity_checker_;
  double position_cov_log_det_lost_threshold_;
  double orientation_cov_log_det_lost_threshold_;
  gtsam::Pose3 estimated_world_T_dock_;
  boost::optional<localization_measurements::TimestampedHandrailPose> estimated_world_T_handrail_;
  bool reset_world_T_dock_;
  bool reset_world_T_handrail_;
  bool estimate_world_T_dock_using_loc_;
  int ar_min_num_landmarks_;
  int sparse_mapping_min_num_landmarks_;
  localization_measurements::FanSpeedMode fan_speed_mode_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_WRAPPER_H_
