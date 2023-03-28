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
#include <graph_vio/graph_vio_wrapper.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>
#include <msg_conversions/msg_conversions.h>

#include <Eigen/Core>

namespace graph_vio {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

GraphVIOWrapper::GraphVIOWrapper(const std::string& graph_config_path_prefix) { LoadConfigs(graph_config_path_prefix); }

void GraphVIOWrapper::LoadConfigs(const std::string& graph_config_path_prefix) {
  config_reader::ConfigReader config;
  lc::LoadGraphVIOConfig(config, graph_config_path_prefix);
  config.AddFile("transforms.config");
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");

  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  if (!config.GetBool("publish_graph", &publish_graph_)) {
    LogFatal("Failed to load publish_graph.");
  }

  if (!config.GetBool("save_graph_dot_file", &save_graph_dot_file_)) {
    LogFatal("Failed to load save_graph_dot_file.");
  }

  LoadGraphVIOParams(config, params_);
}

void GraphVIOWrapper::FeaturePointsCallback(const ff_msgs::Feature2dArray& feature_array_msg) {
  if (Initialized()) graph_vio_->AddFeaturePointsMeasurement(lm::MakeFeaturePointsMeasurement(feature_array_msg));
}

void GraphVIOWrapper::ImuCallback(const sensor_msgs::Imu& imu_msg) {
  const auto imu_measurement = lm::ImuMeasurement(imu_msg);
  imu_bias_initializer_.AddImuMeasurement(imu_measurement);
  if (!Initialized() && imu_bias_initializer_.Bias()) {
    // Set initial nav state. Use bias from initializer and
    // assume zero initial velocity. Set pose initial to identity.
    const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                             *(imu_bias_initializer_.Bias()), imu_measurement.timestamp);
    params_.combined_nav_state_node_adder.start_node = initial_state;
    params_.combined_nav_state_node_adder.starting_time = initial_state.timestamp;
    graph_vio_.reset(new graph_vio::GraphVIO(params_);
    LogDebug("ImuCallback: Initialized Graph.");
  }
  if (Initialized()) graph_vio_->AddImuMeasurement(imu_measurement);
}

void GraphVIOWrapper::FlightModeCallback(const ff_msgs::FlightMode& flight_mode) {
  fan_speed_mode_ = lm::ConvertFanSpeedMode(flight_mode.speed);
  if (Initialized()) graph_vio_->SetFanSpeedMode(fan_speed_mode_);
  imu_bias_initializer_.AddFanSpeedModeMeasurement(fan_speed_mode_);
}

void GraphVIOWrapper::Update() {
  if (Initialized()) graph_vio_->Update();
}

bool GraphVIOWrapper::Initialized() const { return graph_vio_; }

void GraphVIOWrapper::ResetVIO() {
  LogInfo("ResetVIO: Resetting vio.");
  if (!Initialized()) {
    LogError("ResetVIO: VIO not initialized, nothing to do.");
    return;
  }
  // TODO(rsoussan): Get latest bias from graph vio!
  imu_bias_initializer_.UpdateBias(latest_bias);
  graph_vio_.reset();
}

void GraphVIOWrapper::ResetBiasesAndVIO() {
  LogInfo("ResetBiasAndVIO: Resetting biases and vio.");
  imu_bias_initializer_.Reset();
  graph_vio_.reset();
}

void GraphVIOWrapper::ResetBiasesFromFileAndResetVIO() {
  LogInfo("ResetBiasAndVIO: Resetting biases from file and resetting vio.");
  imu_bias_initializer_.LoadFromFile();
  graph_vio_.reset();
}
}  // namespace graph_vio
