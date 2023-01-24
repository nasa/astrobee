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

#include <graph_optimizer/utilities.h>
#include <node_updaters/pose_node_updater.h>
#include <node_updaters/pose_node_updater_params.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>

namespace node_updaters {
namespace go = graph_optimizer;
namespace lc = localization_common;
namespace sym = gtsam::symbol_shorthand;
PoseNodeUpdater::PoseNodeUpdater(std::shared_ptr<TimestampedPoseNodes> nodes)
    : nodes_(nodes) {
  const gtsam::Vector6 pose_prior_noise_sigmas(
    (gtsam::Vector(6) << params_.starting_prior_translation_stddev, params_.starting_prior_translation_stddev,
     params_.starting_prior_translation_stddev, params_.starting_prior_quaternion_stddev,
     params_.starting_prior_quaternion_stddev, params_.starting_prior_quaternion_stddev)
      .finished());
  global_T_body_start_noise_ = go::Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_prior_noise_sigmas)), params_.huber_k);
}

void PoseNodeUpdater::AddInitialValuesAndPriors(gtsam::NonlinearFactorGraph& factors) {
  AddInitialValuesAndPriors(params_.global_T_body_start, global_T_body_start_noise_, params_.starting_time, factors);
}

void PoseNodeUpdater::AddInitialValuesAndPriors(const gtsam::Pose3& global_T_body, const gtsam::SharedNoiseModel& noise,
                                                const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  nodes_->Add(timestamp, global_T_body);
  AddPriors(global_T_body, noise, timestamp, factors);
}

void PoseNodeUpdater::AddPriors(const gtsam::Pose3& global_T_body, const gtsam::SharedNoiseModel& noise,
                                const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  const auto key = nodes_->Key(timestamp);
  if (!key) {
    LogError("AddPriors: Failed to get key.");
    return;
  }
  // TODO(rsoussan): Ensure symbols not used by other node updaters
  gtsam::PriorFactor<gtsam::Pose3> pose_prior_factor(*key, global_T_body, noise);
  factors.push_back(pose_prior_factor);
}

bool PoseNodeUpdater::SlideWindow(const lc::Time oldest_allowed_timestamp,
                                              const boost::optional<gtsam::Marginals>& marginals,
                                              const gtsam::KeyVector& old_keys, const double huber_k,
                                              gtsam::NonlinearFactorGraph& factors) {
  nodes_->RemoveOldNodes(oldest_allowed_timestamp);
  if (params_.add_priors) {
    // Add prior to oldest pose using covariances from last round of
    // optimization
    const auto global_T_body_oldest = nodes_->OldestNode();
    const auto oldest_timestamp = nodes_->OldestTimestamp();
    if (!global_T_body_oldest || !oldest_timestamp) {
      LogError("SlideWindow: Failed to get oldest pose and timestamp.");
      return false;
    }

    // Make sure priors are removed before adding new ones
    RemovePriors(old_keys, factors);
    if (marginals) {
      const auto key = nodes_->Key(*oldest_timestamp);
    if (!key) {
      LogError("SlideWindow: Failed to get oldest key.");
      return false;
    }
      const auto pose_noise =
        go::Robust(gtsam::noiseModel::Gaussian::Covariance(marginals->marginalCovariance(*key)), huber_k);
      AddPriors(*global_T_body_oldest, pose_noise, *oldest_timestamp, factors);
    } else {
      // TODO(rsoussan): Add seperate marginal fallback sigmas instead of relying on starting prior sigmas
      AddPriors(*global_T_body_oldest, global_T_body_start_noise_, *oldest_timestamp, factors);
    }
  }

  return true;
}

go::NodeUpdaterType PoseNodeUpdater::type() const { return go::NodeUpdaterType::Pose; }

boost::optional<lc::Time> PoseNodeUpdater::SlideWindowNewOldestTime() const {
  // TODO(rsoussan): Generalize this with CombinedNavStateGraphValues
  if (nodes_->empty()) {
    LogDebug("SlideWindowOldestTime: No states in map.");
    return boost::none;
  }

  const size_t size = nodes_->size();
  if (size <= params_.min_num_states) {
    LogDebug("SlideWindowOldestTime: Not enough states to remove.");
    return boost::none;
  }

  const double total_duration = nodes_->Duration();
  LogDebug("SlideWindowOldestTime: Starting total num states: " << nodes_->size());
  LogDebug("SlideWindowOldestTime: Starting total duration is " << total_duration);
  const lc::Time ideal_oldest_allowed_state =
    std::max(0.0, *(nodes_->LatestTimestamp()) - params_.ideal_duration);

  int num_states_to_be_removed = 0;
  // Ensures that new oldest time is consistent with a number of states <= max_num_states
  // and >= min_num_states.
  // Assumes min_num_states < max_num_states.
  for (const auto& timestamp : nodes_->Timestamps()) {
    ++num_states_to_be_removed;
    const int new_num_states = size - num_states_to_be_removed;
    if (new_num_states > params_.max_num_states) continue;
    if (new_num_states <= params_.min_num_states) return timestamp;
    if (timestamp >= ideal_oldest_allowed_state) return timestamp;
  }

  // Shouldn't occur
  return boost::none;
}

gtsam::KeyVector PoseNodeUpdater::OldKeys(const lc::Time oldest_allowed_time,
                                                      const gtsam::NonlinearFactorGraph& graph) const {
  return nodes_->OldKeys(oldest_allowed_time);
}

// TODO(rsoussan): Change this interface
boost::optional<gtsam::Key> PoseNodeUpdater::GetKey(go::KeyCreatorFunction key_creator_function,
                                                                const lc::Time timestamp) const {
  return nodes_->Key(timestamp);
}

boost::optional<lc::Time> PoseNodeUpdater::OldestTimestamp() const {
  return nodes_->OldestTimestamp();
}

boost::optional<lc::Time> PoseNodeUpdater::LatestTimestamp() const {
  return nodes_->LatestTimestamp();
}

void PoseNodeUpdater::RemovePriors(const gtsam::KeyVector& old_keys, gtsam::NonlinearFactorGraph& factors) {
  int removed_factors = 0;
  for (auto factor_it = factors.begin(); factor_it != factors.end();) {
    bool erase_factor = false;
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factor_it->get());
    if (pose_prior_factor) {
    // Erase factor if it contains an old key
    for (const auto& old_key : old_keys) {
      if (pose_prior_factor->key() == old_key) {
        erase_factor = true;
        factor_it = factors.erase(factor_it);
        ++removed_factors;
      }
    }
    if (!erase_factor) {
      ++factor_it;
    }
  }
  }
  LogDebug("RemovePriors: Erase " << removed_factors << " factors.");
}

// TODO(rsoussan): Generalize this, use in CombinedNavStateNodeUpdater
bool PoseNodeUpdater::Update(const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  if (nodes_->Contains(timestamp)) {
    LogDebug(
      "Update: Node exists at "
      "timestamp, nothing to do.");
    return true;
  }

  const auto latest_timestamp = nodes_->LatestTimestamp();
  if (!latest_timestamp) {
    LogError("Update: Failed to get latest timestamp.");
    return false;
  }

  if (timestamp > *latest_timestamp) {
    LogDebug(
      "Update: Adding latest node and relative factor.");
    return AddLatestNodeAndRelativeFactor(timestamp, factors);
  } else {
    LogDebug("Update: Splitting old relative factor.");
    return SplitOldRelativeFactor(timestamp, factors, graph_values);
  }
}

bool PoseNodeUpdater::AddLatestNodeAndRelativeFactor(
  const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  // TODO(rsoussan): Need to make container for rel odom poses!  (already have this?)
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
  return true;
}

bool PoseNodeUpdater::CreateAndAddImuFactorAndPredictedCombinedNavState(
  const lc::CombinedNavState& global_N_body, const gtsam::PreintegratedCombinedMeasurements& pim,
  gtsam::NonlinearFactorGraph& factors, gv::CombinedNavStateGraphValues& graph_values) {
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

bool PoseNodeUpdater::SplitOldImuFactorAndAddCombinedNavState(
  const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors, gv::CombinedNavStateGraphValues& graph_values) {
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

  if (!CreateAndAddImuFactorAndPredictedCombinedNavState(*lower_bound_combined_nav_state, *first_integrated_pim,
                                                         factors, graph_values)) {
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
}  // namespace node_updaters
