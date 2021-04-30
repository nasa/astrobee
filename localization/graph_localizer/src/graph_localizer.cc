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

#include <graph_localizer/graph_localizer.h>
#include <graph_localizer/loc_projection_factor.h>
#include <graph_localizer/loc_pose_factor.h>
#include <graph_localizer/pose_rotation_factor.h>
#include <graph_localizer/utilities.h>
#include <imu_integration/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <unistd.h>

#include <chrono>
#include <unordered_set>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;

GraphLocalizer::GraphLocalizer(const GraphLocalizerParams& params)
    : GraphOptimizer(params.graph_optimizer),
      feature_tracker_(new FeatureTracker(params.feature_tracker)),
      latest_imu_integrator_(new ii::LatestImuIntegrator(params.graph_initializer)),
      log_on_destruction_(true),
      params_(params) {
  latest_imu_integrator_->SetFanSpeedMode(params_.initial_fan_speed_mode);

  // Initialize smart projection factor params
  // TODO(rsoussan): access this from smart cumulative adder?
  smart_projection_params_.verboseCheirality = params_.factor.smart_projection_adder.verbose_cheirality;
  smart_projection_params_.setRankTolerance(1e-9);
  smart_projection_params_.setLandmarkDistanceThreshold(
    params_.factor.smart_projection_adder.landmark_distance_threshold);
  smart_projection_params_.setDynamicOutlierRejectionThreshold(
    params_.factor.smart_projection_adder.dynamic_outlier_rejection_threshold);
  smart_projection_params_.setRetriangulationThreshold(params_.factor.smart_projection_adder.retriangulation_threshold);
  smart_projection_params_.setEnableEPI(params_.factor.smart_projection_adder.enable_EPI);

  // Initialize lm params
  if (params_.verbose) {
    levenberg_marquardt_params_.verbosityLM = gtsam::LevenbergMarquardtParams::VerbosityLM::TRYDELTA;
    levenberg_marquardt_params_.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::LINEAR;
  }
  if (params_.use_ceres_params) {
    gtsam::LevenbergMarquardtParams::SetCeresDefaults(&levenberg_marquardt_params_);
  }

  levenberg_marquardt_params_.maxIterations = params_.max_iterations;

  if (params_.marginals_factorization == "qr") {
    marginals_factorization_ = gtsam::Marginals::Factorization::QR;
  } else if (params_.marginals_factorization == "cholesky") {
    marginals_factorization_ = gtsam::Marginals::Factorization::CHOLESKY;
  } else {
    LogError("GraphLocalizer: No marginals factorization entered, defaulting to qr.");
    marginals_factorization_ = gtsam::Marginals::Factorization::QR;
  }

  // Initialize Factor Adders
  ar_tag_loc_factor_adder_.reset(
    new LocFactorAdder(params_.factor.ar_tag_loc_adder, GraphActionCompleterType::ARTagLocProjectionFactor));
  graph_action_completers_.emplace_back(ar_tag_loc_factor_adder_);
  loc_factor_adder_.reset(new LocFactorAdder(params_.factor.loc_adder, GraphActionCompleterType::LocProjectionFactor));
  graph_action_completers_.emplace_back(loc_factor_adder_);
  projection_factor_adder_.reset(
    new ProjectionFactorAdder(params_.factor.projection_adder, feature_tracker_, graph_values_));
  graph_action_completers_.emplace_back(projection_factor_adder_);
  rotation_factor_adder_.reset(new RotationFactorAdder(params_.factor.rotation_adder, feature_tracker_));
  smart_projection_cumulative_factor_adder_.reset(
    new SmartProjectionCumulativeFactorAdder(params_.factor.smart_projection_adder, feature_tracker_));
  graph_action_completers_.emplace_back(smart_projection_cumulative_factor_adder_);
  standstill_factor_adder_.reset(new StandstillFactorAdder(params_.factor.standstill_adder, feature_tracker_));

  // Initialize Node Updaters
  CombinedNavStateNodeUpdaterParams combined_nav_state_node_updater_params;
  // Assumes zero initial velocity
  const lc::CombinedNavState global_N_body_start(params_.graph_initializer.global_T_body_start,
                                                 gtsam::Velocity3::Zero(), params_.graph_initializer.initial_imu_bias,
                                                 params_.graph_initializer.start_time);
  combined_nav_state_node_updater_params.global_N_body_start = global_N_body_start;
  const gtsam::Vector6 pose_prior_noise_sigmas(
    (gtsam::Vector(6) << params_.noise.starting_prior_translation_stddev,
     params_.noise.starting_prior_translation_stddev, params_.noise.starting_prior_translation_stddev,
     params_.noise.starting_prior_quaternion_stddev, params_.noise.starting_prior_quaternion_stddev,
     params_.noise.starting_prior_quaternion_stddev)
      .finished());
  const gtsam::Vector3 velocity_prior_noise_sigmas((gtsam::Vector(3) << params_.noise.starting_prior_velocity_stddev,
                                                    params_.noise.starting_prior_velocity_stddev,
                                                    params_.noise.starting_prior_velocity_stddev)
                                                     .finished());
  const gtsam::Vector6 bias_prior_noise_sigmas(
    (gtsam::Vector(6) << params_.noise.starting_prior_accel_bias_stddev, params_.noise.starting_prior_accel_bias_stddev,
     params_.noise.starting_prior_accel_bias_stddev, params_.noise.starting_prior_gyro_bias_stddev,
     params_.noise.starting_prior_gyro_bias_stddev, params_.noise.starting_prior_gyro_bias_stddev)
      .finished());
  lc::CombinedNavStateNoise global_N_body_start_noise;
  global_N_body_start_noise.pose_noise = Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_prior_noise_sigmas)), params_.huber_k);
  global_N_body_start_noise.velocity_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)),
           params_.huber_k);
  global_N_body_start_noise.bias_noise = Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(bias_prior_noise_sigmas)), params_.huber_k);
  combined_nav_state_node_updater_params.global_N_body_start_noise = global_N_body_start_noise;
  combined_nav_state_node_updater_params.add_priors = params_.add_priors;
  combined_nav_state_node_updater_.reset(
    new CombinedNavStateNodeUpdater(combined_nav_state_node_updater_params, latest_imu_integrator_));
  combined_nav_state_node_updater_->AddInitialValuesAndPriors(graph_, *graph_values_);
  timestamped_node_updaters_.emplace_back(combined_nav_state_node_updater_);
}

GraphLocalizer::~GraphLocalizer() {
  if (log_on_destruction_) graph_stats_.Log();
}

boost::optional<std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>>
GraphLocalizer::LatestCombinedNavStateAndCovariances() const {
  if (!marginals_) {
    LogDebugEveryN(50, "LatestCombinedNavStateAndCovariances: No marginals available.");
    return boost::none;
  }
  const auto state_covariance_pair = LatestCombinedNavStateAndCovariances(*marginals_);
  if (!state_covariance_pair) {
    LogDebug(
      "LatestCombinedNavStateAndCovariances: Failed to get latest combined nav state and "
      "covariances.");
    return boost::none;
  }

  return state_covariance_pair;
}

boost::optional<std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>>
GraphLocalizer::LatestCombinedNavStateAndCovariances(const gtsam::Marginals& marginals) const {
  const auto global_N_body_latest = graph_values_->LatestCombinedNavState();
  if (!global_N_body_latest) {
    LogError("LatestCombinedNavStateAndCovariance: Failed to get latest combined nav state.");
    return boost::none;
  }
  const auto latest_combined_nav_state_key_index = graph_values_->LatestCombinedNavStateKeyIndex();
  if (!latest_combined_nav_state_key_index) {
    LogError("LatestCombinedNavStateAndCovariance: Failed to get latest combined nav state.");
    return boost::none;
  }

  try {
    const auto pose_covariance = marginals.marginalCovariance(sym::P(*latest_combined_nav_state_key_index));
    const auto velocity_covariance = marginals.marginalCovariance(sym::V(*latest_combined_nav_state_key_index));
    const auto bias_covariance = marginals.marginalCovariance(sym::B(*latest_combined_nav_state_key_index));
    const lc::CombinedNavStateCovariances latest_combined_nav_state_covariances(pose_covariance, velocity_covariance,
                                                                                bias_covariance);
    return std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>{*global_N_body_latest,
                                                                            latest_combined_nav_state_covariances};
  } catch (...) {
    LogError("LatestCombinedNavStateAndCovariances: Failed to get marginal covariances.");
    return boost::none;
  }
}

boost::optional<lc::CombinedNavState> GraphLocalizer::LatestCombinedNavState() const {
  const auto global_N_body_latest = graph_values_->LatestCombinedNavState();
  if (!global_N_body_latest) {
    LogError("LatestCombinedNavState: Failed to get latest combined nav state.");
    return boost::none;
  }

  return global_N_body_latest;
}

boost::optional<lc::CombinedNavState> GraphLocalizer::GetCombinedNavState(const lc::Time time) const {
  const auto lower_bound_or_equal_combined_nav_state = graph_values_->LowerBoundOrEqualCombinedNavState(time);
  if (!lower_bound_or_equal_combined_nav_state) {
    LogDebug("GetCombinedNavState: Failed to get lower bound or equal combined nav state.");
    return boost::none;
  }

  if (lower_bound_or_equal_combined_nav_state->timestamp() == time) {
    return lower_bound_or_equal_combined_nav_state;
  }

  // Pim predict from lower bound state rather than closest state so there is no
  // need to reverse predict (going backwards in time) using a pim prediction which is not yet supported in gtsam.
  auto integrated_pim = latest_imu_integrator_->IntegratedPim(lower_bound_or_equal_combined_nav_state->bias(),
                                                              lower_bound_or_equal_combined_nav_state->timestamp(),
                                                              time, latest_imu_integrator_->pim_params());
  if (!integrated_pim) {
    LogError("GetCombinedNavState: Failed to create integrated pim.");
    return boost::none;
  }

  return ii::PimPredict(*lower_bound_or_equal_combined_nav_state, *integrated_pim);
}

boost::optional<std::pair<gtsam::imuBias::ConstantBias, lc::Time>> GraphLocalizer::LatestBiases() const {
  const auto latest_bias = graph_values_->LatestBias();
  if (!latest_bias) {
    LogError("LatestBiases: Failed to get latest biases.");
    return boost::none;
  }
  return latest_bias;
}

// Latest extrapolated pose time is limited by latest imu time
boost::optional<lc::Time> GraphLocalizer::LatestExtrapolatedPoseTime() const {
  return latest_imu_integrator_->LatestTime();
}

void GraphLocalizer::AddImuMeasurement(const lm::ImuMeasurement& imu_measurement) {
  latest_imu_integrator_->BufferImuMeasurement(imu_measurement);
}

bool GraphLocalizer::AddOpticalFlowMeasurement(
  const lm::FeaturePointsMeasurement& optical_flow_feature_points_measurement) {
  if (!MeasurementRecentEnough(optical_flow_feature_points_measurement.timestamp)) {
    LogDebug("AddOpticalFlowMeasurement: Measurement too old - discarding.");
    return false;
  }

  // TODO(rsoussan): This is a bug in optical flow node, fix there
  static lc::Time last_time = optical_flow_feature_points_measurement.timestamp;
  if (last_time == optical_flow_feature_points_measurement.timestamp) {
    LogDebug("AddOpticalFlowMeasurement: Same timestamp measurement, ignoring.");
    last_time = optical_flow_feature_points_measurement.timestamp;
    return false;
  }
  last_time = optical_flow_feature_points_measurement.timestamp;

  LogDebug("AddOpticalFlowMeasurement: Adding optical flow measurement.");
  feature_tracker_->UpdateFeatureTracks(optical_flow_feature_points_measurement.feature_points);

  if (optical_flow_feature_points_measurement.feature_points.empty()) {
    LogDebug("AddOpticalFlowMeasurement: Empty measurement.");
    return false;
  }

  // TODO(rsoussan): Enforce that projection and smart factor adders are not both enabled
  if (params_.factor.projection_adder.enabled) {
    BufferFactors(projection_factor_adder_->AddFactors(optical_flow_feature_points_measurement));
  }
  if (params_.factor.rotation_adder.enabled) {
    BufferFactors(rotation_factor_adder_->AddFactors(optical_flow_feature_points_measurement));
  }

  CheckForStandstill();
  if (standstill() && params_.factor.standstill_adder.enabled) {
    BufferFactors(standstill_factor_adder_->AddFactors(optical_flow_feature_points_measurement));
  }

  return true;
}

void GraphLocalizer::CheckForStandstill() {
  // Check for standstill via low disparity for all feature tracks
  double total_average_distance_from_mean = 0;
  int num_valid_feature_tracks = 0;
  for (const auto& feature_track : feature_tracker_->feature_tracks()) {
    const double average_distance_from_mean =
      AverageDistanceFromMean(feature_track.second->LatestPointsInWindow(params_.standstill_feature_track_duration));
    // Only consider long enough feature tracks for standstill candidates
    if (static_cast<int>(feature_track.second->size()) >= params_.standstill_min_num_points_per_track) {
      total_average_distance_from_mean += average_distance_from_mean;
      ++num_valid_feature_tracks;
    }
  }

  double average_distance_from_mean = 0;
  if (num_valid_feature_tracks > 0)
    average_distance_from_mean = total_average_distance_from_mean / num_valid_feature_tracks;

  standstill_ = (num_valid_feature_tracks >= 5 &&
                 average_distance_from_mean <= params_.max_standstill_feature_track_avg_distance_from_mean);
  if (*standstill_) LogDebug("CheckForStandstill: Standstill.");
}

void GraphLocalizer::AddARTagMeasurement(const lm::MatchedProjectionsMeasurement& matched_projections_measurement) {
  if (!MeasurementRecentEnough(matched_projections_measurement.timestamp)) {
    LogDebug("AddARTagMeasurement: Measurement too old - discarding.");
    return;
  }

  if (params_.factor.ar_tag_loc_adder.enabled &&
      static_cast<int>(matched_projections_measurement.matched_projections.size()) >=
        params_.factor.ar_tag_loc_adder.min_num_matches) {
    LogDebug("AddARTagMeasurement: Adding AR tag measurement.");
    BufferFactors(ar_tag_loc_factor_adder_->AddFactors(matched_projections_measurement));
  }
}

void GraphLocalizer::AddSparseMappingMeasurement(
  const lm::MatchedProjectionsMeasurement& matched_projections_measurement) {
  if (!MeasurementRecentEnough(matched_projections_measurement.timestamp)) {
    LogDebug("AddSparseMappingMeasurement: Measurement too old - discarding.");
    return;
  }

  if (params_.factor.loc_adder.enabled) {
    LogDebug("AddSparseMappingMeasurement: Adding sparse mapping measurement.");
    BufferFactors(loc_factor_adder_->AddFactors(matched_projections_measurement));
  }
}

bool GraphLocalizer::SlideWindow(const boost::optional<gtsam::Marginals>& marginals, const lc::Time last_latest_time) {
  const auto graph_values_ideal_new_oldest_time = graph_values_->SlideWindowNewOldestTime();
  if (!graph_values_ideal_new_oldest_time) {
    LogDebug("SlideWindow: No states removed. ");
    return true;
  }
  // Ensure that new oldest time isn't more recent than last latest time
  // since then priors couldn't be added for the new oldest state
  if (last_latest_time < *graph_values_ideal_new_oldest_time)
    LogError("SlideWindow: Ideal oldest time is more recent than last latest time.");
  const auto new_oldest_time = std::min(last_latest_time, *graph_values_ideal_new_oldest_time);

  // Add marginal factors for marginalized values
  auto old_keys = graph_values_->OldKeys(new_oldest_time);
  // Since cumlative factors have many keys and shouldn't be marginalized, need to remove old measurements depending on
  // old keys before marginalizing and sliding window
  RemoveOldMeasurementsFromCumulativeFactors(old_keys);
  auto old_factors = graph_values_->RemoveOldFactors(old_keys, graph_);
  gtsam::KeyVector old_feature_keys;
  if (params_.factor.projection_adder.enabled) {
    // Call remove old factors before old feature keys, since old feature keys depend on
    // number of factors per key remaining
    old_feature_keys = graph_values_->OldFeatureKeys(graph_);
    auto old_feature_factors = graph_values_->RemoveOldFactors(old_feature_keys, graph_);
    old_keys.insert(old_keys.end(), old_feature_keys.begin(), old_feature_keys.end());
    old_factors.push_back(old_feature_factors);
  }
  if (params_.add_marginal_factors) {
    const auto marginal_factors = MarginalFactors(old_factors, old_keys, gtsam::EliminateQR);
    for (const auto& marginal_factor : marginal_factors) {
      graph_factors().push_back(marginal_factor);
    }
  }

  for (auto& node_updater : timestamped_node_updaters_)
    node_updater->SlideWindow(new_oldest_time, marginals, params_.huber_k, graph_, *graph_values_);
  if (params_.factor.projection_adder.enabled) graph_values_->RemoveOldFeatures(old_feature_keys);

  // Remove old data from other containers
  // TODO(rsoussan): Just use new_oldest_time and don't bother getting oldest timestamp here?
  const auto oldest_timestamp = graph_values_->OldestTimestamp();
  if (!oldest_timestamp || *oldest_timestamp != new_oldest_time) {
    LogError("SlideWindow: Failed to get oldest timestamp.");
    return false;
  }

  feature_tracker_->RemoveOldFeaturePointsAndSlideWindow(*oldest_timestamp);
  latest_imu_integrator_->RemoveOldMeasurements(*oldest_timestamp);
  RemoveOldBufferedFactors(*oldest_timestamp);

  if (params_.factor.projection_adder.enabled && params_.factor.projection_adder.add_point_priors && marginals_) {
    UpdatePointPriors(*marginals_);
  }
  return true;
}

void GraphLocalizer::UpdatePointPriors(const gtsam::Marginals& marginals) {
  const auto feature_keys = graph_values_->FeatureKeys();
  for (const auto& feature_key : feature_keys) {
    const auto world_t_point = graph_values_->at<gtsam::Point3>(feature_key);
    if (!world_t_point) {
      LogError("UpdatePointPriors: Failed to get world_t_point.");
      continue;
    }
    for (auto factor_it = graph_factors().begin(); factor_it != graph_factors().end();) {
      const auto point_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Point3>*>(factor_it->get());
      if (point_prior_factor && (point_prior_factor->key() == feature_key)) {
        // Erase old prior
        factor_it = graph_factors().erase(factor_it);
        // Add updated one
        const auto point_prior_noise =
          Robust(gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(feature_key)), params_.huber_k);
        const gtsam::PriorFactor<gtsam::Point3> point_prior_factor(feature_key, *world_t_point, point_prior_noise);
        graph_factors().push_back(point_prior_factor);
        // Only one point prior per feature
        break;
      } else {
        ++factor_it;
      }
    }
  }
}

void GraphLocalizer::BufferCumulativeFactors() {
  // Remove measurements here so they are more likely to fit in sliding window duration when optimized
  feature_tracker_->RemoveOldFeaturePointsAndSlideWindow();
  if (params_.factor.smart_projection_adder.enabled) {
    BufferFactors(smart_projection_cumulative_factor_adder_->AddFactors());
  }
}

void GraphLocalizer::RemoveOldMeasurementsFromCumulativeFactors(const gtsam::KeyVector& old_keys) {
  for (auto factor_it = graph_factors().begin(); factor_it != graph_factors().end();) {
    const auto smart_factor = dynamic_cast<const RobustSmartFactor*>(factor_it->get());
    // Currently the only cumulative factors are smart factors
    // TODO(rsoussan): Generalize this
    if (!smart_factor) {
      ++factor_it;
      continue;
    }
    const auto& factor_keys = (*factor_it)->keys();
    std::unordered_set<int> factor_key_indices_to_remove;
    for (int i = 0; i < static_cast<int>(factor_keys.size()); ++i) {
      for (const auto& old_key : old_keys) {
        if (factor_keys[i] == old_key) {
          factor_key_indices_to_remove.emplace(i);
          break;
        }
      }
    }
    if (factor_key_indices_to_remove.empty()) {
      ++factor_it;
      continue;
    } else {
      if (static_cast<int>(factor_keys.size() - factor_key_indices_to_remove.size()) <
          params_.factor.smart_projection_adder.min_num_points) {
        factor_it = graph_factors().erase(factor_it);
        continue;
      } else {
        const auto new_smart_factor = RemoveSmartFactorMeasurements(
          *smart_factor, factor_key_indices_to_remove, params_.factor.smart_projection_adder, smart_projection_params_);
        *factor_it = new_smart_factor;
        continue;
      }
    }
  }
}

bool GraphLocalizer::ReadyToAddFactors(const localization_common::Time timestamp) const {
  const auto latest_time = latest_imu_integrator_->LatestTime();
  if (!latest_time) {
    LogError("ReadyToAddMeasurement: Failed to get latet imu time.");
    return false;
  }

  return (timestamp <= *latest_time);
}

bool GraphLocalizer::MeasurementRecentEnough(const lc::Time timestamp) const {
  if (!latest_imu_integrator_->OldestTime()) {
    LogDebug("MeasurementRecentEnough: Waiting until imu measurements have been received.");
    return false;
  }
  if (timestamp < graph_values_->OldestTimestamp()) return false;
  if (timestamp < latest_imu_integrator_->OldestTime()) return false;
  return true;
}

void GraphLocalizer::PrintFactorDebugInfo() const {
  for (const auto& factor : graph_factors()) {
    const auto smart_factor = dynamic_cast<const RobustSmartFactor*>(factor.get());
    if (smart_factor) {
      smart_factor->print();
      if (smart_factor->isValid())
        LogDebug("PrintFactorDebugInfo: SmartFactor valid.");
      else
        LogDebug("PrintFactorDebugInfo: SmartFactor invalid.");
      if (smart_factor->isDegenerate()) LogDebug("PrintFactorDebugInfo: SmartFactor degenerate.");
      if (smart_factor->isPointBehindCamera()) LogDebug("PrintFactorDebugInfo: SmartFactor point behind camera.");
      if (smart_factor->isOutlier()) LogDebug("PrintFactorDebugInfo: SmartFactor is outlier.");
      if (smart_factor->isFarPoint()) LogDebug("PrintFactorDebugInfo: SmartFactor is far point.");
    }
    const auto imu_factor = dynamic_cast<gtsam::CombinedImuFactor*>(factor.get());
    if (imu_factor) {
      LogDebug("PrintFactorDebugInfo: CombinedImuFactor: " << *imu_factor);
      LogDebug("PrintFactorDebugInfo: CombinedImuFactor PIM: " << imu_factor->preintegratedMeasurements());
    }
  }
}

void GraphLocalizer::SetFanSpeedMode(const lm::FanSpeedMode fan_speed_mode) {
  latest_imu_integrator_->SetFanSpeedMode(fan_speed_mode);
}

const lm::FanSpeedMode GraphLocalizer::fan_speed_mode() const { return latest_imu_integrator_->fan_speed_mode(); }

const GraphLocalizerParams& GraphLocalizer::params() const { return params_; }

int GraphLocalizer::NumFeatures() const { return graph_values_->NumFeatures(); }

// TODO(rsoussan): fix this call to happen before of factors are removed!
int GraphLocalizer::NumOFFactors(const bool check_valid) const {
  if (params_.factor.smart_projection_adder.enabled) return NumSmartFactors(check_valid);
  if (params_.factor.projection_adder.enabled) return NumProjectionFactors(check_valid);
  return 0;
}

int GraphLocalizer::NumProjectionFactors(const bool check_valid) const {
  int num_factors = 0;
  for (const auto& factor : graph_) {
    const auto projection_factor = dynamic_cast<const ProjectionFactor*>(factor.get());
    if (projection_factor) {
      if (check_valid) {
        const auto world_t_point = graph_values_->at<gtsam::Point3>(projection_factor->key2());
        if (!world_t_point) {
          LogError("NumProjectionFactors: Failed to get point.");
          continue;
        }
        const auto world_T_body = graph_values_->at<gtsam::Pose3>(projection_factor->key1());
        if (!world_T_body) {
          LogError("NumProjectionFactors: Failed to get pose.");
          continue;
        }
        const auto world_T_camera = *world_T_body * params_.calibration.body_T_nav_cam;
        const auto camera_t_point = world_T_camera.inverse() * *world_t_point;
        if (camera_t_point.z() <= 0) {
          LogDebug("NumProjectionFactors: Behind camera.");
          continue;
        }
        ++num_factors;
      } else {
        ++num_factors;
      }
    }
  }
  return num_factors;
}

int GraphLocalizer::NumSmartFactors(const bool check_valid) const {
  int num_of_factors = 0;
  for (const auto& factor : graph_) {
    const auto smart_factor = dynamic_cast<const RobustSmartFactor*>(factor.get());
    if (smart_factor) {
      if (check_valid) {
        if (smart_factor->isValid()) ++num_of_factors;
      } else {
        ++num_of_factors;
      }
    }
  }
  return num_of_factors;
}

const GraphStats& GraphLocalizer::graph_stats() const { return graph_stats_; }

void GraphLocalizer::LogOnDestruction(const bool log_on_destruction) { log_on_destruction_ = log_on_destruction; }

bool GraphLocalizer::standstill() const {
  // If uninitialized, return not at standstill
  // TODO(rsoussan): Is this the appropriate behavior?
  if (!standstill_) return false;
  return *standstill_;
}

void DoPostOptimizeActions() {
  // Update imu integrator bias
  const auto latest_bias = graph_values_->LatestBias();
  if (!latest_bias) {
    LogError("Update: Failed to get latest bias.");
    return false;
  }

  latest_imu_integrator_->ResetPimIntegrationAndSetBias(latest_bias->first);
}
}  // namespace graph_localizer
