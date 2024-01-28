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
#include <parameter_reader/graph_vio.h>
#include <ros_graph_vio/parameter_reader.h>
#include <ros_graph_vio/ros_graph_vio_wrapper.h>

#include <Eigen/Core>

namespace ros_graph_vio {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;
namespace pr = parameter_reader;

RosGraphVIOWrapper::RosGraphVIOWrapper(const std::string& graph_config_path_prefix) {
  LoadConfigs(graph_config_path_prefix);
}

void RosGraphVIOWrapper::LoadConfigs(const std::string& graph_config_path_prefix) {
  config_reader::ConfigReader config;
  config.AddFile("localization/imu_bias_initializer.config");
  config.AddFile("localization/imu_filter.config");
  lc::LoadGraphVIOConfig(config, graph_config_path_prefix);
  pr::LoadGraphVIOParams(config, params_);
  ImuBiasInitializerParams imu_bias_initializer_params;
  LoadImuBiasInitializerParams(config, imu_bias_initializer_params);
  imu_bias_initializer_.reset(new ImuBiasInitializer(imu_bias_initializer_params));
}

void RosGraphVIOWrapper::FeaturePointsCallback(const ff_msgs::Feature2dArray& feature_array_msg) {
  if (Initialized()) graph_vio_->AddFeaturePointsMeasurement(lm::MakeFeaturePointsMeasurement(feature_array_msg));
}

void RosGraphVIOWrapper::ImuCallback(const sensor_msgs::Imu& imu_msg) {
  const auto imu_measurement = lm::ImuMeasurement(imu_msg);
  imu_bias_initializer_->AddImuMeasurement(imu_measurement);
  if (!Initialized() && imu_bias_initializer_->Bias()) {
    // Set initial nav state. Use bias from initializer and
    // assume zero initial velocity. Set pose initial to identity.
    const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                             *(imu_bias_initializer_->Bias()), imu_measurement.timestamp);
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
  imu_bias_initializer_->AddFanSpeedModeMeasurement(fan_speed_mode);
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
  imu_bias_initializer_->UpdateBias(latest_combined_nav_state->bias());
  graph_vio_.reset();
}

void RosGraphVIOWrapper::ResetBiasesAndVIO() {
  LogInfo("ResetBiasAndVIO: Resetting biases and vio.");
  imu_bias_initializer_->Reset();
  graph_vio_.reset();
}

void RosGraphVIOWrapper::ResetBiasesFromFileAndResetVIO() {
  LogInfo("ResetBiasAndVIO: Resetting biases from file and resetting vio.");
  imu_bias_initializer_->LoadFromFile();
  graph_vio_.reset();
}

boost::optional<ff_msgs::GraphVIOState> RosGraphVIOWrapper::GraphVIOStateMsg() {
  if (!Initialized()) return boost::none;
  const auto& nodes = graph_vio_->combined_nav_state_nodes();
  const auto times = nodes.Timestamps();
  // Avoid sending repeat msgs
  if (times.empty() || (latest_msg_time_ && times.back() == *latest_msg_time_)) return boost::none;
  const lc::Time latest_time = times.back();
  latest_msg_time_ = latest_time;
  ff_msgs::GraphVIOState msg;
  for (const auto& time : times) {
    const auto combined_nav_state = nodes.Node(time);
    const auto keys = nodes.Keys(time);
    if (!combined_nav_state || keys.empty() || keys.size() != 3) {
      LogError("CombinedNavStateArrayMsg: Failed to get combined nav state and keys.");
      return boost::none;
    }
    const auto pose_covariance = graph_vio_->Covariance(keys[0]);
    const auto velocity_covariance = graph_vio_->Covariance(keys[1]);
    const auto imu_bias_covariance = graph_vio_->Covariance(keys[2]);
    if (!pose_covariance || !velocity_covariance || !imu_bias_covariance) {
      LogError("CombinedNavStateArrayMsg: Failed to get combined nav state covariances.");
      return boost::none;
    }
    msg.combined_nav_states.combined_nav_states.push_back(
      lc::CombinedNavStateToMsg(*combined_nav_state, *pose_covariance, *velocity_covariance, *imu_bias_covariance));
  }
  lc::TimeToHeader(*(nodes.LatestTimestamp()), msg.header);
  msg.child_frame_id = "odom";
  msg.standstill = graph_vio_->standstill();
  msg.num_of_factors = graph_vio_->NumFactors<factor_adders::RobustSmartFactor>();
  // TODO(rsoussan): Add more stats here!
  return msg;
}
}  // namespace ros_graph_vio
