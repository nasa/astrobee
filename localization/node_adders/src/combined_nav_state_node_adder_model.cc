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

#include <imu_integration/utilities.h>
#include <node_adders/combined_nav_state_node_adder_model.h>
#include <node_adders/utilities.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

namespace node_adders {
namespace no = nodes;
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
CombinedNavStateNodeAdderModel::CombinedNavStateNodeAdderModel(const CombinedNavStateNodeAdderModelParams& params)
    : Base(params), imu_integrator_(params.imu_integrator) {}

void CombinedNavStateNodeAdderModel::AddPriors(const lc::CombinedNavState& node,
                                               const std::vector<gtsam::SharedNoiseModel>& noise_models,
                                               const lc::Time timestamp, const no::CombinedNavStateNodes& nodes,
                                               gtsam::NonlinearFactorGraph& factors) const {
  const auto keys = nodes.Keys(timestamp);
  if (keys.empty()) {
    LogError("AddPriors: Failed to get keys.");
    return;
  }

  const auto& pose_key = keys[0];
  const auto& velocity_key = keys[1];
  const auto& bias_key = keys[2];

  const auto& pose_noise = noise_models[0];
  const auto& velocity_noise = noise_models[1];
  const auto& bias_noise = noise_models[2];

  gtsam::PriorFactor<gtsam::Pose3> pose_prior_factor(pose_key, node.pose(), pose_noise);
  factors.push_back(pose_prior_factor);
  gtsam::PriorFactor<gtsam::Velocity3> velocity_prior_factor(velocity_key, node.velocity(), velocity_noise);
  factors.push_back(velocity_prior_factor);
  gtsam::PriorFactor<gtsam::imuBias::ConstantBias> bias_prior_factor(bias_key, node.bias(), bias_noise);
  factors.push_back(bias_prior_factor);
}

bool CombinedNavStateNodeAdderModel::AddNodesAndRelativeFactors(const lc::Time timestamp_a, const lc::Time timestamp_b,
                                                                no::CombinedNavStateNodes& nodes,
                                                                gtsam::NonlinearFactorGraph& factors) const {
  if (!nodes.Contains(timestamp_b)) {
    const auto node_a = nodes.Node(timestamp_a);
    if (!node_a) {
      LogError("AddNodesAndRelativeFactors: Failed to get node a.");
      return false;
    }

    const auto node_b = imu_integrator_.Extrapolate(*node_a, timestamp_b);
    if (!node_b) {
      LogError("AddNodesAndRelativeFactors: Failed to get node b by extrapolating node a.");
      return false;
    }

    const auto keys = nodes.Add(timestamp_b, *node_b);
    if (keys.empty()) {
      LogError("AddNodesAndRelativeFactors: Failed to add node b.");
      return false;
    }
  }

  if (!AddRelativeFactors(timestamp_a, timestamp_b, nodes, factors)) {
    LogError("AddNodesAndRelativeFactors: Failed to add relative factor.");
    return false;
  }
  return true;
}

bool CombinedNavStateNodeAdderModel::AddRelativeFactors(const lc::Time timestamp_a, const lc::Time timestamp_b,
                                                        const no::CombinedNavStateNodes& nodes,
                                                        gtsam::NonlinearFactorGraph& factors) const {
  const auto keys_a = nodes.Keys(timestamp_a);
  if (keys_a.empty()) {
    LogError("AddRelativeFactors: Failed to get keys a.");
    return false;
  }
  const auto keys_b = nodes.Keys(timestamp_b);
  if (keys_b.empty()) {
    LogError("AddRelativeFactors: Failed to get keys b.");
    return false;
  }

  const auto& pose_key_a = keys_a[0];
  const auto& velocity_key_a = keys_a[1];
  const auto& bias_key_a = keys_a[2];

  const auto& pose_key_b = keys_b[0];
  const auto& velocity_key_b = keys_b[1];
  const auto& bias_key_b = keys_b[2];

  const auto node_a = nodes.Node(timestamp_a);
  const auto node_b = nodes.Node(timestamp_b);
  if (!node_a || !node_b) {
    LogError("AddRelativeFactors: Failed to get nodes a and b.");
    return false;
  }

  auto pim = imu_integrator_.IntegratedPim(node_a->bias(), timestamp_a, timestamp_b);
  if (!pim) {
    LogError("AddRelativeFactors: Failed to create pim.");
    return false;
  }

  const gtsam::CombinedImuFactor::shared_ptr combined_imu_factor(
    new gtsam::CombinedImuFactor(pose_key_a, velocity_key_a, pose_key_b, velocity_key_b, bias_key_a, bias_key_b, *pim));
  factors.push_back(combined_imu_factor);
  return true;
}

void CombinedNavStateNodeAdderModel::AddMeasurement(const lm::ImuMeasurement& measurement) {
  imu_integrator_.AddImuMeasurement(measurement);
}

void CombinedNavStateNodeAdderModel::RemoveMeasurements(const lc::Time oldest_allowed_time) {
  // Don't need lower bound since imu integration doesn't use measurements
  // at same timestamp as start time for integration
  imu_integrator_.RemoveOldValues(oldest_allowed_time);
}

bool CombinedNavStateNodeAdderModel::CanAddNode(const localization_common::Time timestamp) const {
  return imu_integrator_.WithinBounds(timestamp);
}

bool CombinedNavStateNodeAdderModel::RemoveRelativeFactors(const localization_common::Time timestamp_a,
                                                           const localization_common::Time timestamp_b,
                                                           const NodesType& nodes,
                                                           gtsam::NonlinearFactorGraph& factors) const {
  return RemoveRelativeFactor<gtsam::CombinedImuFactor, NodesType>(timestamp_a, timestamp_b, nodes, factors);
}

void CombinedNavStateNodeAdderModel::SetFanSpeedMode(const localization_measurements::FanSpeedMode& fan_speed_mode) {
  imu_integrator_.SetFanSpeedMode(fan_speed_mode);
}
}  // namespace node_adders
