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
#include <ros_graph_vio/ros_graph_vio_wrapper.h>

#include <Eigen/Core>

namespace ros_graph_vio {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

RosGraphVIOWrapper::RosGraphVIOWrapper(const std::string& graph_config_path_prefix) {
  LoadConfigs(graph_config_path_prefix);
}

void RosGraphVIOWrapper::LoadConfigs(const std::string& graph_config_path_prefix) {
  config_reader::ConfigReader config;
  lc::LoadGraphVIOConfig(config, graph_config_path_prefix);
  config.AddFile("transforms.config");
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");

  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  // TODO(rsoussan): put this back!!
  // LoadGraphVIOParams(config, params_);
}

void RosGraphVIOWrapper::FeaturePointsCallback(const ff_msgs::Feature2dArray& feature_array_msg) {
  if (Initialized()) graph_vio_->AddFeaturePointsMeasurement(lm::MakeFeaturePointsMeasurement(feature_array_msg));
}

void RosGraphVIOWrapper::ImuCallback(const sensor_msgs::Imu& imu_msg) {
  const auto imu_measurement = lm::ImuMeasurement(imu_msg);
  imu_bias_initializer_.AddImuMeasurement(imu_measurement);
  if (!Initialized() && imu_bias_initializer_.Bias()) {
    // Set initial nav state. Use bias from initializer and
    // assume zero initial velocity. Set pose initial to identity.
    const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                             *(imu_bias_initializer_.Bias()), imu_measurement.timestamp);
    params_.combined_nav_state_node_adder.start_node = initial_state;
    params_.combined_nav_state_node_adder.starting_time = initial_state.timestamp();
    graph_vio_.reset(new graph_vio::GraphVIO(params_));
    LogDebug("ImuCallback: Initialized Graph.");
  }
  if (Initialized()) graph_vio_->AddImuMeasurement(imu_measurement);
}

void RosGraphVIOWrapper::FlightModeCallback(const ff_msgs::FlightMode& flight_mode) {
  const auto fan_speed_mode = lm::ConvertFanSpeedMode(flight_mode.speed);
  // TODO(rsoussan): Add support for fan speed mode in graph vio
  // if (Initialized()) graph_vio_->SetFanSpeedMode(fan_speed_mode_);
  imu_bias_initializer_.AddFanSpeedModeMeasurement(fan_speed_mode);
}

void RosGraphVIOWrapper::Update() {
  if (Initialized()) graph_vio_->Update();
}

bool RosGraphVIOWrapper::Initialized() const { return graph_vio_ != nullptr; }

void RosGraphVIOWrapper::ResetVIO() {
  LogInfo("ResetVIO: Resetting vio.");
  if (!Initialized()) {
    LogError("ResetVIO: VIO not initialized, nothing to do.");
    return;
  }
  const auto latest_combined_nav_state = graph_vio_->combined_nav_state_nodes().LatestNode();
  imu_bias_initializer_.UpdateBias(latest_combined_nav_state->bias());
  graph_vio_.reset();
}

void RosGraphVIOWrapper::ResetBiasesAndVIO() {
  LogInfo("ResetBiasAndVIO: Resetting biases and vio.");
  imu_bias_initializer_.Reset();
  graph_vio_.reset();
}

void RosGraphVIOWrapper::ResetBiasesFromFileAndResetVIO() {
  LogInfo("ResetBiasAndVIO: Resetting biases from file and resetting vio.");
  imu_bias_initializer_.LoadFromFile();
  graph_vio_.reset();
}

ff_msgs::GraphVIOState RosGraphVIOWrapper::GraphVIOStateMsg() const {
  ff_msgs::GraphVIOState msg;
  if (!Initialized()) return msg;
  const auto& nodes = graph_vio_->combined_nav_state_nodes();
  for (const auto& time : nodes.Timestamps()) {
    const auto combined_nav_state = nodes.Node(time);
    const auto keys = nodes.Keys(time);
    if (!combined_nav_state || keys.empty() || keys.size() != 3) {
      LogError("CombinedNavStateArrayMsg: Failed to get combined nav state and keys.");
      return msg;
    }
    const auto pose_covariance = graph_vio_->Covariance(keys[0]);
    const auto velocity_covariance = graph_vio_->Covariance(keys[1]);
    const auto imu_bias_covariance = graph_vio_->Covariance(keys[2]);
    if (!pose_covariance || !velocity_covariance || !imu_bias_covariance) {
      LogError("CombinedNavStateArrayMsg: Failed to get combined nav state covariances.");
      return msg;
    }
    msg.combined_nav_states.combined_nav_states.push_back(
      lc::CombinedNavStateToMsg(*combined_nav_state, *pose_covariance, *velocity_covariance, *imu_bias_covariance));
  }
  lc::TimeToHeader(*(nodes.LatestTimestamp()), msg.header);
  msg.child_frame_id = "odom";
  // TODO(rsoussan): Add more stats here!
  return msg;
}
}  // namespace ros_graph_vio
