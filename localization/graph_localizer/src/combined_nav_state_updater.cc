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

#include <graph_localizer/combined_nav_state_node_updater.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/PriorFactor.h>

namespace graph_localizer {
namespace sym = gtsam::symbol_shorthand;
explicit CombinedNavStateNodeUpdater::CombinedNavStateNodeUpdater(
  const CombinedNavStateNodeUpdaterParams& params,
  std::shared_ptr<imu_integration::LatestImuIntegrator> latest_imu_integrator)
    : params_(params), latest_imu_integrator_(std::move(latest_imu_integrator)) {}

void CombinedNavStateNodeUpdater::AddInitialValuesAndPriors(gtsam::NonlinearFactorGraph& graph,
                                                            GraphValues& graph_values) {
  AddInitialValuesAndPriors(params_.global_N_body_start, params_.global_N_body_start_noise, graph, graph_values);
}

void CombinedNavStateNodeUpdater::AddInitialValuesAndPriors(const localization_common::CombinedNavState& global_N_body,
                                                            const localization_common::CombinedNavStateNoise& noise,
                                                            gtsam::NonlinearFactorGraph& graph,
                                                            GraphValues& graph_values) {
  const int key_index = GenerateKeyIndex();
  graph_values_->AddCombinedNavState(global_N_body, key_index);
  AddPriors(global_N_body, noise, graph_values, graph);
}

void CombinedNavStateNodeUpdater::AddPriors(const localization_common::CombinedNavState& global_N_body,
                                            const localization_common::CombinedNavStateNoise& noise,
                                            const GraphValues& graph_values, gtsam::NonlinearFactorGraph& factors) {
  const auto key_index = graph_values_.KeyIndex(global_N_body.timestamp());
  if (!key_index) {
    LogError("AddPriors: Failed to get key index.");
    return;
  }
  // TODO(rsoussan): Ensure symbols not used by other node updaters
  gtsam::PriorFactor<gtsam::Pose3> pose_prior_factor(sym::P(*key_index), global_N_body.pose(), noise.pose_noise);
  factors.push_back(pose_prior_factor);
  gtsam::PriorFactor<gtsam::Velocity3> velocity_prior_factor(sym::V(*key_index), global_N_body.velocity(),
                                                             noise.velocity_noise);
  factors.push_back(velocity_prior_factor);
  gtsam::PriorFactor<gtsam::imuBias::ConstantBias> bias_prior_factor(sym::B(*key_index), global_N_body.bias(),
                                                                     noise.bias_noise);
  factors.push_back(bias_prior_factor);
}

void CombinedNavStateNodeUpdater::AddFactors(const FactorToAdd& measurement, gtsam::NonlinearFactorGraph& graph,
                                             GraphValues& graph_values);

void CombinedNavStateNodeUpdater::SlideWindow(const localization_common::Timestamp oldest_allowed_timestamp,
                                              gtsam::NonlinearFactorGraph& factors, GraphValues& graph_values);
int CombinedNavStateNodeUpdater::GenerateKeyIndex() { return key_index++; }

bool CombinedNavStateNodeUpdater::Update(const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors,
                                         GraphValues& graph_values) {
  AddOrSplitImuFactorIfNeeded(timestamp, factors, graph_values);
}

bool CombinedNavStateNodeUpdater::AddOrSplitImuFactorIfNeeded(const lc::Time timestamp,
                                                              gtsam::NonlinearFactorGraph& factors,
                                                              GraphValues& graph_values) {
  if (graph_values.HasKey(timestamp)) {
    LogDebug(
      "AddOrSplitImuFactorIfNeeded: CombinedNavState exists at "
      "timestamp, nothing to do.");
    return true;
  }

  const auto latest_timestamp = graph_values.LatestTimestamp();
  if (!latest_timestamp) {
    LogError("AddOrSplitImuFactorIfNeeded: Failed to get latest timestamp.");
    return false;
  }

  if (timestamp > *latest_timestamp) {
    LogDebug(
      "AddOrSplitImuFactorIfNeeded: Creating and adding latest imu "
      "factor and nav state.");
    return CreateAndAddLatestImuFactorAndCombinedNavState(timestamp, factors, graph_values);
  } else {
    LogDebug("AddOrSplitImuFactorIfNeeded: Splitting old imu factor.");
    return SplitOldImuFactorAndAddCombinedNavState(timestamp, factors, graph_values);
  }
}

bool CombinedNavStateNodeUpdater::CreateAndAddLatestImuFactorAndCombinedNavState(const lc::Time timestamp,
                                                                                 gtsam::NonlinearFactorGraph& factors,
                                                                                 GraphValues& graph_values) {
  if (!latest_imu_integrator_->IntegrateLatestImuMeasurements(timestamp)) {
    LogError("CreateAndAddLatestImuFactorAndCombinedNavState: Failed to integrate latest imu measurements.");
    return false;
  }

  const auto latest_combined_nav_state = graph_values.LatestCombinedNavState();
  if (!latest_combined_nav_state) {
    LogError("CreateAndAddLatestImuFactorAndCombinedNavState: Failed to get latest combined nav state.");
    return false;
  }
  if (!CreateAndAddImuFactorAndPredictedCombinedNavState(*latest_combined_nav_state, latest_imu_integrator_->pim(),
                                                         factors, graph_values)) {
    LogError("CreateAndAddLatestImuFactorAndCombinedNavState: Failed to create and add imu factor.");
    return false;
  }

  const auto latest_bias = graph_values.LatestBias();
  if (!latest_bias) {
    LogError("CreateAndAddLatestImuFactorAndCombinedNavState: Failed to get latest bias.");
    return false;
  }

  latest_imu_integrator_->ResetPimIntegrationAndSetBias(latest_bias->first);
  return true;
}

bool CombinedNavStateUpdater::CreateAndAddImuFactorAndPredictedCombinedNavState(
  const lc::CombinedNavState& global_N_body, const gtsam::PreintegratedCombinedMeasurements& pim,
  gtsam::NonlinearFactorGraph& factors, GraphValues& graph_values) {
  const auto key_index_0 = graph_values.KeyIndex(global_N_body.timestamp());
  if (!key_index_0) {
    LogError("CreateAndAddImuFactorAndPredictedCombinedNavState: Failed to get first key index.");
    return false;
  }

  const lc::CombinedNavState global_N_body_predicted = ii::PimPredict(global_N_body, pim);
  const int key_index_1 = GenerateKeyIndex();
  const auto combined_imu_factor = ii::MakeCombinedImuFactor(*key_index_0, key_index_1, pim);
  factors.push_back(combined_imu_factor);
  graph_values.AddCombinedNavState(global_N_body_predicted, key_index_1);
  return true;
}

bool CombinedNavStateUpdater::SplitOldImuFactorAndAddCombinedNavState(const lc::Time timestamp,
                                                                      gtsam::NonlinearFactorGraph& factors,
                                                                      GraphValues& graph_values) {
  const auto timestamp_bounds = graph_values.LowerAndUpperBoundTimestamp(timestamp);
  if (!timestamp_bounds.first || !timestamp_bounds.second) {
    LogError("SplitOldImuFactorAndAddCombinedNavState: Failed to get upper and lower bound timestamp.");
    return false;
  }

  const lc::Time lower_bound_time = *(timestamp_bounds.first);
  const lc::Time upper_bound_time = *(timestamp_bounds.second);

  if (timestamp < lower_bound_time || timestamp > upper_bound_time) {
    LogError("SplitOldImuFactorAndAddCombinedNavState: Timestamp is not within bounds of existing timestamps.");
    return false;
  }

  const auto lower_bound_key_index = graph_values.KeyIndex(lower_bound_time);
  const auto upper_bound_key_index = graph_values.KeyIndex(upper_bound_time);
  if (!lower_bound_key_index || !upper_bound_key_index) {
    LogError("SplitOldImuFactorAndAddCombinedNavState: Failed to get lower and upper bound key indices.");
    return false;
  }

  // get old imu factor, delete it
  bool removed_old_imu_factor = false;
  for (auto factor_it = factors.begin(); factor_it != factors.end();) {
    if (dynamic_cast<gtsam::CombinedImuFactor*>(factor_it->get()) &&
        graph_values.ContainsCombinedNavStateKey(**factor_it, *lower_bound_key_index) &&
        graph_values.ContainsCombinedNavStateKey(**factor_it, *upper_bound_key_index)) {
      factors.erase(factor_it);
      removed_old_imu_factor = true;
      break;
    }
    ++factor_it;
  }
  if (!removed_old_imu_factor) {
    LogError(
      "SplitOldImuFactorAndAddCombinedNavState: Failed to remove "
      "old imu factor.");
    return false;
  }

  const auto lower_bound_bias = graph_values.at<gtsam::imuBias::ConstantBias>(sym::B(*lower_bound_key_index));
  if (!lower_bound_bias) {
    LogError("SplitOldImuFactorAndAddCombinedNavState: Failed to get lower bound bias.");
    return false;
  }

  // Add first factor and new nav state at timestamp
  auto first_integrated_pim = latest_imu_integrator_->IntegratedPim(*lower_bound_bias, lower_bound_time, timestamp,
                                                                    latest_imu_integrator_->pim_params());
  if (!first_integrated_pim) {
    LogError("SplitOldImuFactorAndAddCombinedNavState: Failed to create first integrated pim.");
    return false;
  }

  const auto lower_bound_combined_nav_state = graph_values.GetCombinedNavState(lower_bound_time);
  if (!lower_bound_combined_nav_state) {
    LogError("SplitOldImuFactorAndAddCombinedNavState: Failed to get lower bound combined nav state.");
    return false;
  }

  if (!CreateAndAddImuFactorAndPredictedCombinedNavState(*lower_bound_combined_nav_state, *first_integrated_pim)) {
    LogError("SplitOldImuFactorAndAddCombinedNavState: Failed to create and add imu factor.");
    return false;
  }

  // Add second factor, use lower_bound_bias as starting bias since that is the
  // best estimate available
  auto second_integrated_pim = latest_imu_integrator_->IntegratedPim(*lower_bound_bias, timestamp, upper_bound_time,
                                                                     latest_imu_integrator_->pim_params());
  if (!second_integrated_pim) {
    LogError("SplitOldImuFactorAndAddCombinedNavState: Failed to create second integrated pim.");
    return false;
  }

  // New nav state already added so just get its key index
  const auto new_key_index = graph_values.KeyIndex(timestamp);
  if (!new_key_index) {
    LogError("SplitOldImuFactorAndAddCombinedNavState: Failed to get new key index.");
    return false;
  }

  const auto combined_imu_factor =
    ii::MakeCombinedImuFactor(*new_key_index, *upper_bound_key_index, *second_integrated_pim);
  factors.push_back(combined_imu_factor);
  return true;
}
}  // namespace graph_localizer
