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

#include <ff_common/ff_names.h>
#include <localization_analysis/imu_bias_extrapolator.h>
#include <localization_analysis/utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/imu_measurement.h>
#include <msg_conversions/msg_conversions.h>
#include <parameter_reader/imu_integration.h>

namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;
namespace pr = parameter_reader;

namespace localization_analysis {
void SaveExtrapolatedStates(const std::vector<lc::CombinedNavState>& extrapolated_states, rosbag::Bag& bag) {
  for (const auto& state : extrapolated_states) {
    geometry_msgs::PoseStamped pose_msg;
    lc::PoseToMsg(state.pose(), pose_msg.pose);
    lc::TimeToHeader(state.timestamp(), pose_msg.header);
    const ros::Time timestamp = lc::RosTimeFromHeader(pose_msg.header);
    bag.write("/" + std::string(TOPIC_IMU_BIAS_EXTRAPOLATOR_POSE), timestamp, pose_msg);
    geometry_msgs::Vector3Stamped velocity_msg;
    mc::VectorToMsg(state.velocity(), velocity_msg.vector);
    lc::TimeToHeader(state.timestamp(), velocity_msg.header);
    bag.write("/" + std::string(TOPIC_IMU_BIAS_EXTRAPOLATOR_VELOCITY), timestamp, velocity_msg);
  }
}

ImuBiasExtrapolator::ImuBiasExtrapolator(const std::string& input_bag_name, const std::string& output_bag_name)
    : input_bag_(input_bag_name, rosbag::bagmode::Read),
      output_bag_(output_bag_name, rosbag::bagmode::Write),
      initialized_(false) {
  config_reader::ConfigReader config;
  config.AddFile("localization/imu_filter.config");
  config.AddFile("localization/imu_integrator.config");
  config.AddFile("transforms.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  ii::ImuIntegratorParams params;
  pr::LoadImuIntegratorParams(config, params);
  imu_integrator_.reset(new ii::ImuIntegrator(params));
}

bool ImuBiasExtrapolator::Initialized() { return initialized_; }

void ImuBiasExtrapolator::Initialize(const lc::CombinedNavState& combined_nav_state) {
  // Initialize first extrapolated state using pose/velocity/bias of first VIO state
  latest_extrapolated_state_ = combined_nav_state;
  initialized_ = true;
}

std::vector<lc::CombinedNavState> ImuBiasExtrapolator::VIOStateCallback(
  const ff_msgs::GraphVIOState& graph_vio_state_msg) {
  const auto& latest_state_msg = graph_vio_state_msg.combined_nav_states.combined_nav_states.back();
  const auto combined_nav_state = lc::CombinedNavStateFromMsg(latest_state_msg);
  combined_nav_states_.Add(combined_nav_state.timestamp(), combined_nav_state);
  if (imu_integrator_->empty()) return {};
  // Remove old combined nav states
  const auto oldest_imu_time = *(imu_integrator_->OldestTimestamp());
  combined_nav_states_.RemoveBelowLowerBoundValues(oldest_imu_time);
  std::vector<lc::CombinedNavState> extrapolated_states;
  // Create integrated imu states between successive combined nav states using the
  // older combined nav state's IMU bias values.
  // Make sure an upper bound combined nav state exists before generating imu states
  // so each generated imu state uses the closest lower bound nav state biases.
  // Wait until imu measurement timestamps exceed the upper bound state before removing
  // combined nav states.
  while (combined_nav_states_.size() >= 2) {
    auto lower_bound_state_it = combined_nav_states_.set().begin();
    const auto upper_bound_timestamp = std::next(lower_bound_state_it)->first;
    if (*(imu_integrator_->LatestTimestamp()) < upper_bound_timestamp) break;
    if (!Initialized()) Initialize(lower_bound_state_it->second);
    imu_integrator_->RemoveOldValues(lower_bound_state_it->second.timestamp());
    // Initialize starting state using latest extrapolated pose/velocity/timestamp
    // but latest received nav state bias so the new bias is used for integration.
    const lc::CombinedNavState starting_state(latest_extrapolated_state_.nav_state(),
                                              lower_bound_state_it->second.bias(),
                                              latest_extrapolated_state_.timestamp());
    latest_extrapolated_state_ = *(imu_integrator_->Extrapolate(starting_state, upper_bound_timestamp));
    extrapolated_states.emplace_back(latest_extrapolated_state_);
    combined_nav_states_.set().erase(lower_bound_state_it);
  }
  return extrapolated_states;
}

void ImuBiasExtrapolator::AddExtrapolatedStates() {
  rosbag::View view(input_bag_);
  for (const rosbag::MessageInstance msg : view) {
    if (string_ends_with(msg.getTopic(), TOPIC_GRAPH_VIO_STATE)) {
      const ff_msgs::GraphVIOState::ConstPtr vio_msg = msg.instantiate<ff_msgs::GraphVIOState>();
      const auto extrapolated_states = VIOStateCallback(*vio_msg);
      SaveExtrapolatedStates(extrapolated_states, output_bag_);
    } else if (string_ends_with(msg.getTopic(), TOPIC_HARDWARE_IMU)) {
      sensor_msgs::ImuConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
      imu_integrator_->AddImuMeasurement(lm::ImuMeasurement(*imu_msg));
    }
    output_bag_.write(msg.getTopic(), msg.getTime(), msg);
  }
}
}  // namespace localization_analysis
