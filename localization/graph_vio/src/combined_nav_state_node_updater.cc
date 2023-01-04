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

#include <graph_vio/combined_nav_state_node_updater.h>
#include <graph_vio/utilities.h>
#include <imu_integration/utilities.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/PriorFactor.h>

namespace graph_vio {
namespace go = graph_optimizer;
namespace gv = graph_values;
namespace ii = imu_integration;
namespace lc = localization_common;
namespace sym = gtsam::symbol_shorthand;
CombinedNavStateNodeUpdater::CombinedNavStateNodeUpdater(
  const gv::CombinedNavStateNodeUpdaterParams& params,
  std::shared_ptr<imu_integration::LatestImuIntegrator> latest_imu_integrator, std::shared_ptr<gtsam::Values> values)
    : params_(params),
      latest_imu_integrator_(std::move(latest_imu_integrator)),
      graph_values_(new gv::gv::CombinedNavStateGraphValues(params.graph_values, std::move(values))),
      key_index_(0) {
  const gtsam::Vector6 pose_prior_noise_sigmas(
    (gtsam::Vector(6) << params_.starting_prior_translation_stddev, params_.starting_prior_translation_stddev,
     params_.starting_prior_translation_stddev, params_.starting_prior_quaternion_stddev,
     params_.starting_prior_quaternion_stddev, params_.starting_prior_quaternion_stddev)
      .finished());
  const gtsam::Vector3 velocity_prior_noise_sigmas((gtsam::Vector(3) << params_.starting_prior_velocity_stddev,
                                                    params_.starting_prior_velocity_stddev,
                                                    params_.starting_prior_velocity_stddev)
                                                     .finished());
  const gtsam::Vector6 bias_prior_noise_sigmas(
    (gtsam::Vector(6) << params_.starting_prior_accel_bias_stddev, params_.starting_prior_accel_bias_stddev,
     params_.starting_prior_accel_bias_stddev, params_.starting_prior_gyro_bias_stddev,
     params_.starting_prior_gyro_bias_stddev, params_.starting_prior_gyro_bias_stddev)
      .finished());
  global_N_body_start_noise_.pose_noise = go::Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_prior_noise_sigmas)), params_.huber_k);
  global_N_body_start_noise_.velocity_noise =
    go::Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)),
           params_.huber_k);
  global_N_body_start_noise_.bias_noise = go::Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(bias_prior_noise_sigmas)), params_.huber_k);
}

void CombinedNavStateNodeUpdater::AddInitialValuesAndPriors(gtsam::NonlinearFactorGraph& factors) {
  AddInitialValuesAndPriors(params_.global_N_body_start, global_N_body_start_noise_, factors);
}

void CombinedNavStateNodeUpdater::AddInitialValuesAndPriors(const lc::CombinedNavState& global_N_body,
                                                            const lc::CombinedNavStateNoise& noise,
                                                            gtsam::NonlinearFactorGraph& factors) {
  const int key_index = GenerateKeyIndex();
  graph_values_->AddCombinedNavState(global_N_body, key_index);
  AddPriors(global_N_body, noise, factors);
}

void CombinedNavStateNodeUpdater::AddPriors(const lc::CombinedNavState& global_N_body,
                                            const lc::CombinedNavStateNoise& noise,
                                            gtsam::NonlinearFactorGraph& factors) {
  const auto key_index = graph_values_->KeyIndex(global_N_body.timestamp());
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

bool CombinedNavStateNodeUpdater::SlideWindow(const lc::Time oldest_allowed_timestamp,
                                              const boost::optional<gtsam::Marginals>& marginals,
                                              const gtsam::KeyVector& old_keys, const double huber_k,
                                              gtsam::NonlinearFactorGraph& factors) {
  graph_values_->RemoveOldCombinedNavStates(oldest_allowed_timestamp);
  if (params_.add_priors) {
    // Add prior to oldest nav state using covariances from last round of
    // optimization
    const auto global_N_body_oldest = graph_values_->OldestCombinedNavState();
    if (!global_N_body_oldest) {
      LogError("SlideWindow: Failed to get oldest combined nav state.");
      return false;
    }

    LogDebug("SlideWindow: Oldest state time: " << global_N_body_oldest->timestamp());

    const auto key_index = graph_values_->OldestCombinedNavStateKeyIndex();
    if (!key_index) {
      LogError("SlideWindow: Failed to get oldest combined nav state key index.");
      return false;
    }

    LogDebug("SlideWindow: key index: " << *key_index);

    // Make sure priors are removed before adding new ones
    RemovePriors(*key_index, factors);
    if (marginals) {
      lc::CombinedNavStateNoise noise;
      noise.pose_noise =
        go::Robust(gtsam::noiseModel::Gaussian::Covariance(marginals->marginalCovariance(sym::P(*key_index))), huber_k);
      noise.velocity_noise =
        go::Robust(gtsam::noiseModel::Gaussian::Covariance(marginals->marginalCovariance(sym::V(*key_index))), huber_k);
      auto bias_covariance = marginals->marginalCovariance(sym::B(*key_index));
      if (params_.threshold_bias_uncertainty) ThresholdBiasUncertainty(bias_covariance);
      noise.bias_noise = go::Robust(gtsam::noiseModel::Gaussian::Covariance(bias_covariance), huber_k);
      AddPriors(*global_N_body_oldest, noise, factors);
    } else {
      // TODO(rsoussan): Add seperate marginal fallback sigmas instead of relying on starting prior sigmas
      AddPriors(*global_N_body_oldest, global_N_body_start_noise_, factors);
    }
  }

  return true;
}

void CombinedNavStateNodeUpdater::ThresholdBiasUncertainty(gtsam::Matrix& bias_covariance) const {
  // Only checking sigmas for now
  const auto bias_covariance_sigmas = bias_covariance.diagonal().cwiseSqrt();
  bool valid_sigmas = true;
  for (int i = 0; i < 3; ++i) {
    if (bias_covariance_sigmas[i] > params_.accel_bias_stddev_threshold) {
      valid_sigmas = false;
      break;
    }
  }
  if (valid_sigmas) {
    for (int i = 3; i < 6; ++i) {
      if (bias_covariance_sigmas[i] > params_.gyro_bias_stddev_threshold) {
        valid_sigmas = false;
        break;
      }
    }
  }
  if (valid_sigmas) return;
  gtsam::Vector6 new_sigmas;
  for (int i = 0; i < 3; ++i) {
    new_sigmas[i] = std::min(bias_covariance_sigmas[i], params_.accel_bias_stddev_threshold);
  }
  for (int i = 3; i < 6; ++i) {
    new_sigmas[i] = std::min(bias_covariance_sigmas[i], params_.gyro_bias_stddev_threshold);
  }
  const gtsam::Vector6 new_variances = new_sigmas.cwiseAbs2();
  bias_covariance = new_variances.asDiagonal();
}

go::NodeUpdaterType CombinedNavStateNodeUpdater::type() const { return go::NodeUpdaterType::CombinedNavState; }

boost::optional<lc::Time> CombinedNavStateNodeUpdater::SlideWindowNewOldestTime() const {
  return graph_values_->SlideWindowNewOldestTime();
}

gtsam::KeyVector CombinedNavStateNodeUpdater::OldKeys(const lc::Time oldest_allowed_time,
                                                      const gtsam::NonlinearFactorGraph& graph) const {
  return graph_values_->OldKeys(oldest_allowed_time, graph);
}

boost::optional<gtsam::Key> CombinedNavStateNodeUpdater::GetKey(go::KeyCreatorFunction key_creator_function,
                                                                const lc::Time timestamp) const {
  return graph_values_->GetKey(key_creator_function, timestamp);
}

boost::optional<lc::Time> CombinedNavStateNodeUpdater::OldestTimestamp() const {
  return graph_values_->OldestTimestamp();
}

boost::optional<lc::Time> CombinedNavStateNodeUpdater::LatestTimestamp() const {
  return graph_values_->LatestTimestamp();
}

std::shared_ptr<const gv::CombinedNavStateGraphValues> CombinedNavStateNodeUpdater::shared_graph_values() const {
  return graph_values_;
}

std::shared_ptr<gv::CombinedNavStateGraphValues> CombinedNavStateNodeUpdater::shared_graph_values() {
  return graph_values_;
}

const gv::CombinedNavStateGraphValues& CombinedNavStateNodeUpdater::graph_values() const { return *graph_values_; }

void CombinedNavStateNodeUpdater::RemovePriors(const int key_index, gtsam::NonlinearFactorGraph& factors) {
  int removed_factors = 0;
  for (auto factor_it = factors.begin(); factor_it != factors.end();) {
    bool erase_factor = false;
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factor_it->get());
    if (pose_prior_factor && pose_prior_factor->key() == sym::P(key_index)) {
      erase_factor = true;
    }
    const auto velocity_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Velocity3>*>(factor_it->get());
    if (velocity_prior_factor && velocity_prior_factor->key() == sym::V(key_index)) {
      erase_factor = true;
    }
    const auto bias_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>*>(factor_it->get());
    if (bias_prior_factor && bias_prior_factor->key() == sym::B(key_index)) {
      erase_factor = true;
    }

    if (erase_factor) {
      factor_it = factors.erase(factor_it);
      ++removed_factors;
    } else {
      ++factor_it;
      continue;
    }
  }
  LogDebug("RemovePriors: Erase " << removed_factors << " factors.");
}

int CombinedNavStateNodeUpdater::GenerateKeyIndex() { return key_index_++; }

bool CombinedNavStateNodeUpdater::Update(const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  return AddOrSplitImuFactorIfNeeded(timestamp, factors, *graph_values_);
}

bool CombinedNavStateNodeUpdater::AddOrSplitImuFactorIfNeeded(const lc::Time timestamp,
                                                              gtsam::NonlinearFactorGraph& factors,
                                                              gv::CombinedNavStateGraphValues& graph_values) {
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

bool CombinedNavStateNodeUpdater::CreateAndAddLatestImuFactorAndCombinedNavState(
  const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors, gv::CombinedNavStateGraphValues& graph_values) {
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

bool CombinedNavStateNodeUpdater::CreateAndAddImuFactorAndPredictedCombinedNavState(
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

bool CombinedNavStateNodeUpdater::SplitOldImuFactorAndAddCombinedNavState(
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
}  // namespace graph_vio
