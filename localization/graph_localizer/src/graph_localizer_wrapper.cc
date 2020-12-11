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
  lc::LoadGraphLocalizerConfig(config);
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

  position_cov_log_det_lost_threshold_ = lc::LoadDouble(config, "position_cov_log_det_lost_threshold");
  orientation_cov_log_det_lost_threshold_ = lc::LoadDouble(config, "orientation_cov_log_det_lost_threshold");

  graph_localizer_initialization_.LoadGraphLocalizerParams(config);
  SanityCheckerParams sanity_checker_params;
  LoadSanityCheckerParams(config, sanity_checker_params);
  sanity_checker_.reset(new SanityChecker(sanity_checker_params));
}

void GraphLocalizerWrapper::Update() {
  if (graph_localizer_) {
    graph_localizer_->Update();
    // Sanity check covariances after updates
    if (!CheckCovarianceSanity()) {
      LOG(INFO) << "OpticalFlowCallback: Covariance sanity check failed, resetting localizer.";
      ResetLocalizer();
      return;
    }
  }
}

void GraphLocalizerWrapper::OpticalFlowCallback(const ff_msgs::Feature2dArray& feature_array_msg) {
  if (graph_localizer_) {
    if (graph_localizer_->AddOpticalFlowMeasurement(lm::MakeFeaturePointsMeasurement(feature_array_msg))) {
      feature_counts_.of = graph_localizer_->NumOFFactors();
      // Optimize graph on receival of camera images
      graph_localizer_->Update();
      // Sanity check covariances after updates
      if (!CheckCovarianceSanity()) {
        LOG(INFO) << "OpticalFlowCallback: Covariance sanity check failed, resetting localizer.";
        ResetLocalizer();
        return;
      }
    }
  }
}

void GraphLocalizerWrapper::ResetLocalizer() {
  LOG(INFO) << "ResetLocalizer: Resetting localizer.";
  graph_localizer_initialization_.ResetStartPose();
  if (!latest_biases_) {
    LOG(DFATAL) << "ResetLocalizer: Trying to reset localizer when no biases "
                   "are available.";
    return;
  }
  // TODO(rsoussan): compare current time with latest bias timestamp and print
  // warning if it is too old
  graph_localizer_initialization_.SetBiases(latest_biases_->first);
  graph_localizer_.reset();
  sanity_checker_->Reset();
}

void GraphLocalizerWrapper::ResetBiasesAndLocalizer() {
  LOG(INFO) << "ResetBiasAndLocalizer: Resetting biases and localizer.";
  graph_localizer_initialization_.ResetBiasesAndStartPose();
  graph_localizer_.reset();
  sanity_checker_->Reset();
}

void GraphLocalizerWrapper::VLVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg) {
  if (!ValidVLMsg(visual_landmarks_msg)) return;
  if (graph_localizer_) {
    graph_localizer_->AddSparseMappingMeasurement(lm::MakeMatchedProjectionsMeasurement(visual_landmarks_msg));
    feature_counts_.vl = visual_landmarks_msg.landmarks.size();
  }

  const gtsam::Pose3 sparse_mapping_global_T_body =
    lc::GtPose(visual_landmarks_msg, graph_localizer_initialization_.params().calibration.body_T_nav_cam.inverse());
  const lc::Time timestamp = lc::TimeFromHeader(visual_landmarks_msg.header);
  sparse_mapping_pose_ = std::make_pair(sparse_mapping_global_T_body, timestamp);

  // Sanity Check
  if (graph_localizer_ && !CheckPoseSanity(sparse_mapping_global_T_body, timestamp)) {
    LOG(INFO) << "VLVisualLandmarksCallback: Sanity check failed, resetting localizer.";
    ResetLocalizer();
    return;
  }

  if (!graph_localizer_) {
    // Set or update initial pose if a new one is available before the localizer
    // has started running.
    graph_localizer_initialization_.SetStartPose(sparse_mapping_pose_->first, sparse_mapping_pose_->second);
  }
}

bool GraphLocalizerWrapper::CheckPoseSanity(const gtsam::Pose3& sparse_mapping_pose, const lc::Time timestamp) const {
  if (!graph_localizer_) return true;
  const auto combined_nav_state = graph_localizer_->GetCombinedNavState(timestamp);
  if (!combined_nav_state) {
    LOG(INFO) << "CheckPoseSanity: Failed to get combined nav state.";
    return true;
  }
  return sanity_checker_->CheckPoseSanity(sparse_mapping_pose, combined_nav_state->pose());
}

bool GraphLocalizerWrapper::CheckCovarianceSanity() const {
  if (!graph_localizer_) return true;
  const auto combined_nav_state_and_covariances = graph_localizer_->LatestCombinedNavStateAndCovariances();
  if (!combined_nav_state_and_covariances) {
    LOG(INFO) << "CheckCovarianceSanity: No combined nav state and covariances available.";
    return true;
  }

  return sanity_checker_->CheckCovarianceSanity(combined_nav_state_and_covariances->second);
}

void GraphLocalizerWrapper::ARVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg) {
  if (!ValidVLMsg(visual_landmarks_msg)) return;
  if (graph_localizer_) {
    graph_localizer_->AddARTagMeasurement(lm::MakeMatchedProjectionsMeasurement(visual_landmarks_msg));
    // TODO(rsoussan): Make seperate ar count, update EkfState
    feature_counts_.vl = visual_landmarks_msg.landmarks.size();
  }
}

void GraphLocalizerWrapper::ImuCallback(const sensor_msgs::Imu& imu_msg) {
  if (graph_localizer_) {
    graph_localizer_->AddImuMeasurement(lm::ImuMeasurement(imu_msg));
    latest_biases_ = graph_localizer_->LatestBiases();
    if (!latest_biases_) {
      LOG(WARNING) << "ImuCallback: Failed to get latest biases.";
    }
  } else if (graph_localizer_initialization_.EstimateBiases()) {
    EstimateAndSetImuBiases(lm::ImuMeasurement(imu_msg), num_bias_estimation_measurements_, imu_bias_measurements_,
                            graph_localizer_initialization_);
  }

  if (!graph_localizer_ && graph_localizer_initialization_.ReadyToInitialize()) {
    InitializeGraph();
    LOG(INFO) << "ImuCallback: Initialized Graph.";
  }
}

void GraphLocalizerWrapper::InitializeGraph() {
  if (!graph_localizer_initialization_.ReadyToInitialize()) {
    LOG(ERROR) << "InitializeGraph: Trying to initialize graph when not ready.";
    return;
  }

  graph_localizer_.reset(new graph_localizer::GraphLocalizer(graph_localizer_initialization_.params()));
}

boost::optional<const FeatureTrackMap&> GraphLocalizerWrapper::feature_tracks() const {
  if (!graph_localizer_) return boost::none;
  return graph_localizer_->feature_tracks();
}

boost::optional<std::pair<gtsam::Pose3, lc::Time>> GraphLocalizerWrapper::estimated_world_T_dock() const {
  if (!graph_localizer_ || !graph_localizer_->estimated_world_T_dock()) {
    LOG_EVERY_N(WARNING, 50) << "estimated_world_T_dock: Failed to get world_T_dock";
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

boost::optional<lc::CombinedNavState> GraphLocalizerWrapper::LatestCombinedNavState() const {
  if (!graph_localizer_) {
    LOG_EVERY_N(WARNING, 50) << "LatestCombinedNavState: Graph localizater not initialized yet.";
    return boost::none;
  }
  const auto latest_combined_nav_state = graph_localizer_->LatestCombinedNavState();
  if (!latest_combined_nav_state) {
    LOG_EVERY_N(WARNING, 50) << "LatestCombinedNavState: No combined nav state available.";
    return boost::none;
  }
  return latest_combined_nav_state;
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
                graph_localizer_initialization_.EstimateBiases(), position_cov_log_det_lost_threshold_,
                orientation_cov_log_det_lost_threshold_, graph_localizer_->standstill());
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
