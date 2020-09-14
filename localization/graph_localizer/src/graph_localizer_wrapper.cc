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

#include <config_reader/config_reader.h>
#include <graph_localizer/graph_localizer_wrapper.h>
#include <graph_localizer/parameter_reader.h>
#include <graph_localizer/utilities.h>
#include <imu_integration/utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>

#include <Eigen/Core>

#include <glog/logging.h>

namespace graph_localizer {
namespace lc = localization_common;
namespace lm = localization_measurements;
GraphLocalizerWrapper::GraphLocalizerWrapper() {
  config_reader::ConfigReader config;
  config.AddFile("graph_localizer.config");
  config.AddFile("transforms.config");
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");

  if (!config.ReadFiles()) {
    LOG(FATAL) << "Failed to read config files.";
  }

  if (!config.GetInt("num_bias_estimation_measurements", &num_bias_estimation_measurements_)) {
    LOG(FATAL) << "Failed to load num_bias_estimation_measurements.";
  }

  if (!config.GetBool("publish_localization_graph", &publish_localization_graph_)) {
    LOG(FATAL) << "Failed to load publish_localization_graph.";
  }

  if (!config.GetBool("save_localization_graph_dot_file", &save_localization_graph_dot_file_)) {
    LOG(FATAL) << "Failed to load save_localization_graph_dot_file.";
  }

  graph_loc_initialization_.LoadGraphLocalizerParams(config);
  SanityCheckerParams sanity_checker_params;
  LoadSanityCheckerParams(config, sanity_checker_params);
  sanity_checker_.reset(new SanityChecker(sanity_checker_params));
  sanity_checker_enabled_ = sanity_checker_params.enabled;
}

void GraphLocalizerWrapper::OpticalFlowCallback(const ff_msgs::Feature2dArray& feature_array_msg) {
  if (graph_localizer_) {
    if (graph_localizer_->AddOpticalFlowMeasurement(lm::MakeFeaturePointsMeasurement(feature_array_msg))) {
      feature_counts_.of = graph_localizer_->NumOFFactors();
      // Optimize graph on receival of camera images
      graph_localizer_->Update();
    }
  }
}

void GraphLocalizerWrapper::ResetLocalizer() {
  LOG(INFO) << "ResetLocalizer: Resetting localizer.";
  graph_loc_initialization_.ResetStartPose();
  if (!have_latest_imu_biases_) {
    LOG(DFATAL) << "ResetLocalizer: Trying to reset localizer when no biases "
                   "are available.";
    return;
  }
  // TODO(rsoussan): compare current time with latest bias timestamp and print
  // warning if it is too old
  graph_loc_initialization_.SetBiases(latest_accelerometer_bias_, latest_gyro_bias_);
  graph_localizer_.reset();
}

void GraphLocalizerWrapper::ResetBiasesAndLocalizer() {
  LOG(INFO) << "ResetBiasAndLocalizer: Resetting biases and localizer.";
  graph_loc_initialization_.ResetBiasesAndStartPose();
  graph_localizer_.reset();
}

void GraphLocalizerWrapper::VLVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg) {
  if (!ValidVLMsg(visual_landmarks_msg)) return;
  if (graph_localizer_) {
    graph_localizer_->AddSparseMappingMeasurement(lm::MakeMatchedProjectionsMeasurement(visual_landmarks_msg));
    feature_counts_.vl = visual_landmarks_msg.landmarks.size();
  }

  // TODO(rsoussan): Clean this up
  const Eigen::Isometry3d sparse_mapping_global_T_body = lc::EigenPose(
      visual_landmarks_msg, lc::EigenPose(graph_loc_initialization_.params().calibration.body_T_nav_cam.inverse()));
  const lc::Time timestamp = lc::TimeFromHeader(visual_landmarks_msg.header);
  sparse_mapping_pose_ = std::make_pair(sparse_mapping_global_T_body, timestamp);

  // Sanity Check
  if (graph_localizer_ && sanity_checker_enabled_ && !CheckSanity(sparse_mapping_global_T_body, timestamp)) {
    LOG(INFO) << "VLVisualLandmarksCallback: Sanity check failed, resetting localizer.";
    ResetLocalizer();
    return;
  }

  if (!graph_localizer_) {
    // Set or update initial pose if a new one is available before the localizer
    // has started running.
    graph_loc_initialization_.SetStartPose(sparse_mapping_pose_->first, sparse_mapping_pose_->second);
  }
}

bool GraphLocalizerWrapper::CheckSanity(const Eigen::Isometry3d& sparse_mapping_pose, const lc::Time timestamp) {
  if (!graph_localizer_) return true;
  const auto combined_nav_state = graph_localizer_->GetCombinedNavState(timestamp);
  if (!combined_nav_state) {
    LOG(INFO) << "VLVisualLandmarksCallback: Failed to get combined nav state for sanity check.";
    return true;
  }
  return sanity_checker_->CheckSanity(lc::GtPose(sparse_mapping_pose), combined_nav_state->pose());
}

void GraphLocalizerWrapper::ARVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg) {
  if (!ValidVLMsg(visual_landmarks_msg)) return;
  if (graph_localizer_) {
    const Eigen::Isometry3d dock_T_dock_cam = lc::EigenPose(visual_landmarks_msg);
    graph_localizer_->AddARTagMeasurement(lm::MakeMatchedProjectionsMeasurement(visual_landmarks_msg),
                                          lc::GtPose(dock_T_dock_cam.inverse()));
    // TODO(rsoussan): Make seperate ar count, update EkfState
    feature_counts_.vl = visual_landmarks_msg.landmarks.size();
  }
}

void GraphLocalizerWrapper::ImuCallback(const sensor_msgs::Imu& imu_msg) {
  if (graph_localizer_) {
    graph_localizer_->AddImuMeasurement(lm::ImuMeasurement(imu_msg));
    const auto latest_biases = graph_localizer_->LatestBiases();
    if (!latest_biases) {
      LOG(WARNING) << "ImuCallback: Failed to get latest biases.";
    } else {
      latest_accelerometer_bias_ = latest_biases->first.accelerometer();
      latest_gyro_bias_ = latest_biases->first.gyroscope();
      latest_bias_timestamp_ = latest_biases->second;
      have_latest_imu_biases_ = true;
    }
  } else if (graph_loc_initialization_.EstimateBiases()) {
    EstimateAndSetImuBiases(lm::ImuMeasurement(imu_msg), num_bias_estimation_measurements_, imu_bias_measurements_,
                            graph_loc_initialization_);
  }

  // TODO(rsoussan): put this somewhere else?
  if (!graph_localizer_ && graph_loc_initialization_.ReadyToInitialize()) {
    InitializeGraph();
    LOG(INFO) << "ImuCallback: Initialized Graph.";
  }
}

void GraphLocalizerWrapper::InitializeGraph() {
  if (!graph_loc_initialization_.ReadyToInitialize()) {
    LOG(ERROR) << "InitializeGraph: Trying to initialize graph when not ready.";
    return;
  }

  graph_localizer_.reset(new graph_localizer::GraphLocalizer(graph_loc_initialization_.params()));
}

const FeatureTrackMap* const GraphLocalizerWrapper::feature_tracks() const {
  if (!graph_localizer_) return nullptr;
  return &(graph_localizer_->feature_tracks());
}

boost::optional<std::pair<Eigen::Isometry3d, lc::Time>> GraphLocalizerWrapper::estimated_world_T_dock() const {
  if (!graph_localizer_ || !graph_localizer_->estimated_world_T_dock()) {
    LOG(ERROR) << "estimated_world_T_dock: Failed to get world_T_dock";
    return boost::none;
  }
  return graph_localizer_->estimated_world_T_dock();
}

boost::optional<geometry_msgs::PoseStamped> GraphLocalizerWrapper::LatestSparseMappingPoseMsg() const {
  if (!sparse_mapping_pose_) {
    LOG_EVERY_N(WARNING, 50) << "LatestSparseMappingPoseMsg: Failed to get latest sparse mapping pose msg.";
    return boost::none;
  }

  return PoseMsg(sparse_mapping_pose_->first, sparse_mapping_pose_->second);
}

boost::optional<ff_msgs::EkfState> GraphLocalizerWrapper::LatestLocalizationStateMsg() {
  if (!graph_localizer_) {
    LOG_EVERY_N(WARNING, 50) << "LatestLocalizationMsg: Graph localizater not initialized yet.";
    return boost::none;
  }
  const auto combined_nav_state_and_covariances = graph_localizer_->LatestCombinedNavStateAndCovariances();
  if (!combined_nav_state_and_covariances) {
    LOG(ERROR) << "LatestLocalizationMsg: No combined nav state and covariances available.";
    return boost::none;
  }
  // Angular velocity and acceleration are added by imu integrator
  const auto ekf_state_msg =
      EkfStateMsg(combined_nav_state_and_covariances->first, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                  combined_nav_state_and_covariances->second, feature_counts_.of, feature_counts_.vl,
                  graph_loc_initialization_.EstimateBiases());
  feature_counts_.Reset();
  return ekf_state_msg;
}

boost::optional<ff_msgs::LocalizationGraph> GraphLocalizerWrapper::LatestLocalizationGraphMsg() const {
  if (!graph_localizer_) {
    LOG_EVERY_N(WARNING, 50) << "LatestGraphMsg: Graph localizater not initialized yet.";
    return boost::none;
  }
  return GraphMsg(*graph_localizer_);
}

void GraphLocalizerWrapper::SaveLocalizationGraphDotFile() const {
  if (graph_localizer_) graph_localizer_->SaveGraphDotFile();
}

bool GraphLocalizerWrapper::publish_localization_graph() const { return publish_localization_graph_; }

bool GraphLocalizerWrapper::save_localization_graph_dot_file() const { return save_localization_graph_dot_file_; }

}  // namespace graph_localizer
