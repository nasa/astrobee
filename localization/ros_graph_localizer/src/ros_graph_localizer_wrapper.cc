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
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>
#include <msg_conversions/msg_conversions.h>
#include <ros_graph_localizer/ros_graph_localizer_wrapper.h>

#include <Eigen/Core>

namespace ros_graph_localizer {
namespace gl = graph_localizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

RosGraphLocalizerWrapper::RosGraphLocalizerWrapper(const std::string& graph_config_path_prefix) {
  LoadConfigs(graph_config_path_prefix);
}

void RosGraphLocalizerWrapper::LoadConfigs(const std::string& graph_config_path_prefix) {
  config_reader::ConfigReader config;
  lc::LoadGraphLocalizerConfig(config, graph_config_path_prefix);
  config.AddFile("transforms.config");
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");

  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  // TODO(rsoussan): put this back!!
  // LoadGraphLocalizerParams(config, params_);
}

void RosGraphLocalizerWrapper::SparseMapVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg) {
  if (Initialized())
    graph_localizer_->AddSparseMapMatchedProjectionsMeasurement(
      lm::MakeMatchedProjectionsMeasurement(visual_landmarks_msg));
}

void RosGraphLocalizerWrapper::CombinedNavStateArrayCallback(
  const ff_msgs::CombinedNavStateArray& combined_nav_state_array_msg) {
  // TODO(rsoussan): Send full pose/cov history instead of just latest pose/cov
  const auto& latest_combined_nav_state_msg = combined_nav_state_array_msg.combined_nav_states.back();
  const auto latest_combined_nav_state = lc::CombinedNavStateFromMsg(latest_combined_nav_state_msg);
  const auto latest_covariances = lc::CombinedNavStateCovariancesFromMsg(latest_combined_nav_state_msg);
  const lm::TimestampedPoseWithCovariance pose_measurement(
    lc::PoseWithCovariance(lc::EigenPose(latest_combined_nav_state.pose()), latest_covariances.pose_covariance()),
    latest_combined_nav_state.timestamp());
  if (Initialized()) graph_localizer_->AddPoseMeasurement(pose_measurement);
}

void RosGraphLocalizerWrapper::Update() {
  if (Initialized()) graph_localizer_->Update();
}

bool RosGraphLocalizerWrapper::Initialized() const { return graph_localizer_ != nullptr; }

void RosGraphLocalizerWrapper::ResetLocalizer() {
  LogInfo("ResetLocalizer: Resetting vio.");
  if (!Initialized()) {
    LogError("ResetLocalizer: Localizer not initialized, nothing to do.");
    return;
  }
  // TODO(rsoussan): Don't initialize until new sparse map pose received!!
  graph_localizer_.reset(new gl::GraphLocalizer(params_));
}
}  // namespace ros_graph_localizer
