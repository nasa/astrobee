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
#include <graph_localizer/utilities.h>
#include <imu_integration/utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/PriorFactor.h>

#include <glog/logging.h>

#include <iomanip>

namespace graph_localizer {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;

GraphLocalizer::GraphLocalizer(const GraphLocalizerParams& params)
    : feature_tracker_(params.feature_tracker),
      latest_imu_integrator_(params.graph_initialization),
      graph_values_(params.graph_values),
      params_(params) {
  // Assumes zero initial velocity
  const lc::CombinedNavState global_cgN_body_start(
      params_.graph_initialization.global_T_body_start, gtsam::Velocity3::Zero(),
      params_.graph_initialization.initial_imu_bias, params_.graph_initialization.start_time);

  // Add first nav state and priors to graph
  const int key_index = GenerateKeyIndex();
  graph_values_.AddCombinedNavState(global_cgN_body_start, key_index);
  AddStartingPriors(global_cgN_body_start, key_index, graph_values_.values(), graph_);

  // Initialize smart projection factor params
  smart_projection_params_.verboseCheirality = true;
  smart_projection_params_.setDegeneracyMode(gtsam::DegeneracyMode::ZERO_ON_DEGENERACY);
  smart_projection_params_.setRankTolerance(1e-9);
  smart_projection_params_.setLandmarkDistanceThreshold(100);
  smart_projection_params_.setDynamicOutlierRejectionThreshold(5);
}

void GraphLocalizer::AddStartingPriors(const lc::CombinedNavState& global_cgN_body_start, const int key_index,
                                       const gtsam::Values& values, gtsam::NonlinearFactorGraph& graph) {
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
      (gtsam::Vector(6) << params_.noise.starting_prior_accel_bias_stddev,
       params_.noise.starting_prior_accel_bias_stddev, params_.noise.starting_prior_accel_bias_stddev,
       params_.noise.starting_prior_gyro_bias_stddev, params_.noise.starting_prior_gyro_bias_stddev,
       params_.noise.starting_prior_gyro_bias_stddev)
          .finished());
  lc::CombinedNavStateNoise noise;
  noise.pose_noise =
      Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_prior_noise_sigmas)));
  noise.velocity_noise =
      Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)));
  noise.bias_noise =
      Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(bias_prior_noise_sigmas)));
  AddPriors(global_cgN_body_start, noise, key_index, values, graph);
}

void GraphLocalizer::AddPriors(const lc::CombinedNavState& global_cgN_body, const lc::CombinedNavStateNoise& noise,
                               const int key_index, const gtsam::Values& values, gtsam::NonlinearFactorGraph& graph) {
  gtsam::PriorFactor<gtsam::Pose3> pose_prior_factor(sym::P(key_index), global_cgN_body.pose(), noise.pose_noise);
  graph.push_back(pose_prior_factor);
  gtsam::PriorFactor<gtsam::Velocity3> velocity_prior_factor(sym::V(key_index), global_cgN_body.velocity(),
                                                             noise.velocity_noise);
  graph.push_back(velocity_prior_factor);
  gtsam::PriorFactor<gtsam::imuBias::ConstantBias> bias_prior_factor(sym::B(key_index), global_cgN_body.bias(),
                                                                     noise.bias_noise);
  graph.push_back(bias_prior_factor);
}

boost::optional<std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>>
GraphLocalizer::LatestCombinedNavStateAndCovariances() const {
  if (!marginals_) {
    LOG(ERROR) << "LatestCombinedNavStateAndCovariances: No marginals available.";
    return boost::none;
  }
  const auto state_covariance_pair = LatestCombinedNavStateAndCovariances(*marginals_);
  if (!state_covariance_pair) {
    LOG(ERROR) << "LatestCombinedNavStateAndCovariances: Failed to get latest combined nav state and "
                  "covariances.";
    return boost::none;
  }

  return state_covariance_pair;
}

boost::optional<std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>>
GraphLocalizer::LatestCombinedNavStateAndCovariances(const gtsam::Marginals& marginals) const {
  const auto global_cgN_body_latest = graph_values_.LatestCombinedNavState();
  if (!global_cgN_body_latest) {
    LOG(ERROR) << "LatestCombinedNavStateAndCovariance: Failed to get latest combined nav state.";
    return boost::none;
  }
  const auto latest_combined_nav_state_key_index = graph_values_.LatestCombinedNavStateKeyIndex();
  if (!latest_combined_nav_state_key_index) {
    LOG(ERROR) << "LatestCombinedNavStateAndCovariance: Failed to get latest combined nav state.";
    return boost::none;
  }

  const auto pose_covariance = marginals.marginalCovariance(sym::P(*latest_combined_nav_state_key_index));
  const auto velocity_covariance = marginals.marginalCovariance(sym::V(*latest_combined_nav_state_key_index));
  const auto bias_covariance = marginals.marginalCovariance(sym::B(*latest_combined_nav_state_key_index));
  const lc::CombinedNavStateCovariances latest_combined_nav_state_covariances(pose_covariance, velocity_covariance,
                                                                              bias_covariance);
  return std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>{*global_cgN_body_latest,
                                                                          latest_combined_nav_state_covariances};
}

boost::optional<lc::CombinedNavState> GraphLocalizer::LatestCombinedNavState() const {
  const auto global_cgN_body_latest = graph_values_.LatestCombinedNavState();
  if (!global_cgN_body_latest) {
    LOG(ERROR) << "LatestCombinedNavState: Failed to get latest combined nav state.";
    return boost::none;
  }

  return global_cgN_body_latest;
}

boost::optional<lc::CombinedNavState> GraphLocalizer::GetCombinedNavState(const lc::Time time) const {
  const auto lower_bound_or_equal_combined_nav_state = graph_values_.LowerBoundOrEqualCombinedNavState(time);
  if (!lower_bound_or_equal_combined_nav_state) {
    LOG(ERROR) << "GetCombinedNavState: Failed to get lower bound or equal combined nav state.";
    return boost::none;
  }

  if (lower_bound_or_equal_combined_nav_state->timestamp() == time) {
    return lower_bound_or_equal_combined_nav_state;
  }

  // Pim predict from lower bound state rather than closest state so there is no
  // need to reverse predict (going backwards in time) using a pim prediction which is not yet supported in gtsam.
  // TODO(rsoussan): Add this
  auto integrated_pim = latest_imu_integrator_.IntegratedPim(lower_bound_or_equal_combined_nav_state->bias(),
                                                             lower_bound_or_equal_combined_nav_state->timestamp(), time,
                                                             latest_imu_integrator_.pim_params());
  if (!integrated_pim) {
    LOG(ERROR) << "GetCombinedNavState: Failed to create integrated pim.";
    return boost::none;
  }

  return ii::PimPredict(*lower_bound_or_equal_combined_nav_state, *integrated_pim);
}

boost::optional<std::pair<gtsam::imuBias::ConstantBias, lc::Time>> GraphLocalizer::LatestBiases() const {
  const auto latest_bias = graph_values_.LatestBias();
  if (!latest_bias) {
    LOG(ERROR) << "LatestBiases: Failed to get latest biases.";
    return boost::none;
  }
  return latest_bias;
}

void GraphLocalizer::AddImuMeasurement(const lm::ImuMeasurement& imu_measurement) {
  latest_imu_integrator_.BufferImuMeasurement(imu_measurement);
}

bool GraphLocalizer::AddOpticalFlowMeasurement(
    const lm::FeaturePointsMeasurement& optical_flow_feature_points_measurement) {
  if (!MeasurementRecentEnough(optical_flow_feature_points_measurement.timestamp)) {
    LOG(WARNING) << "AddOpticalFlowMeasurement: Measurement too old - discarding.";
    return false;
  }

  LOG(INFO) << "AddOpticalFlowMeasurement: Adding optical flow measurement.";
  feature_tracker_.UpdateFeatureTracks(optical_flow_feature_points_measurement.feature_points);

  if (optical_flow_feature_points_measurement.feature_points.empty()) {
    LOG(WARNING) << "AddOpticalFlowMeasurement: Empty measurement.";
    return false;
  }

  // Add smart factor for each valid feature track
  FactorsToAdd smart_factors_to_add(GraphAction::kDeleteExistingSmartFactors);
  // Add prior factor if there is low disparity for all feature tracks, indicating standstill, since a smart factor in
  // this case would be ill-constrained
  FactorsToAdd prior_factors_to_add(GraphAction::kFillPriors);
  double potential_standstill_feature_tracks_average_distance_from_mean = 0;
  int num_potential_standstill_feature_tracks = 0;
  for (const auto& feature_track : feature_tracker_.feature_tracks()) {
    const double feature_track_average_distance_from_mean = AverageDistanceFromMean(feature_track.second.points);
    if (ValidPointSet(feature_track.second.points, feature_track_average_distance_from_mean,
                      params_.factor.min_valid_feature_track_avg_distance_from_mean)) {
      AddSmartFactor(feature_track.second, smart_factors_to_add);
    } else if (feature_track.second.points.size() >
               5) {  // Only consider long enough feature tracks for standstill candidates
      potential_standstill_feature_tracks_average_distance_from_mean += feature_track_average_distance_from_mean;
      ++num_potential_standstill_feature_tracks;
    }
  }

  if (num_potential_standstill_feature_tracks > 0)
    potential_standstill_feature_tracks_average_distance_from_mean /= num_potential_standstill_feature_tracks;

  if (!smart_factors_to_add.empty()) {
    smart_factors_to_add.SetTimestamp(optical_flow_feature_points_measurement.timestamp);
    BufferFactors(smart_factors_to_add);
    VLOG(2) << "AddOpticalFLowMeasurement: Buffered " << smart_factors_to_add.size() << " smart factors.";
  } else if (ShouldAddStandstillPrior(
                 potential_standstill_feature_tracks_average_distance_from_mean,
                 num_potential_standstill_feature_tracks,
                 params_.factor)) {  // Only add a standstill prior if no smart factors added and other conditions met
    AddStandstillPriorFactor(optical_flow_feature_points_measurement.timestamp, prior_factors_to_add);
    prior_factors_to_add.SetTimestamp(optical_flow_feature_points_measurement.timestamp);
    BufferFactors(prior_factors_to_add);
    VLOG(2) << "AddOpticalFLowMeasurement: Buffered a pose and velocity prior factor.";
  }

  return true;
}

void GraphLocalizer::AddSmartFactor(const FeatureTrack& feature_track, FactorsToAdd& smart_factors_to_add) {
  // TODO(rsoussan): Modify smart factor to allow for robust kernel
  SharedSmartFactor smart_factor(new SmartFactor(params_.noise.nav_cam_noise, params_.calibration.nav_cam_intrinsics,
                                                 params_.calibration.body_T_nav_cam, smart_projection_params_));

  KeyInfos key_infos;
  key_infos.reserve(feature_track.points.size());
  // Gtsam requires unique key indices for each key, even though these will be replaced later
  int uninitialized_key_index = 0;
  for (const auto& feature_point : feature_track.points) {
    const KeyInfo key_info(&sym::P, feature_point.timestamp);
    key_infos.emplace_back(key_info);
    smart_factor->add(Camera::Measurement(feature_point.image_point), key_info.MakeKey(uninitialized_key_index++));
    // if (feature_point.timestamp > latest_timestamp) latest_timestamp = feature_point.timestamp;
  }
  smart_factors_to_add.push_back({key_infos, smart_factor});
}

void GraphLocalizer::AddStandstillPriorFactor(const lc::Time timestamp, FactorsToAdd& standstill_prior_factors_to_add) {
  LOG_EVERY_N(INFO, 1) << "AddStandstillPriorFactor: Adding standstill priors.";

  if (params_.factor.optical_flow_standstill_pose_prior) {
    const gtsam::Vector6 pose_prior_noise_sigmas(
        (gtsam::Vector(6) << params_.noise.optical_flow_prior_translation_stddev,
         params_.noise.optical_flow_prior_translation_stddev, params_.noise.optical_flow_prior_translation_stddev,
         params_.noise.optical_flow_prior_quaternion_stddev, params_.noise.optical_flow_prior_quaternion_stddev,
         params_.noise.optical_flow_prior_quaternion_stddev)
            .finished());

    const auto pose_noise =
        Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_prior_noise_sigmas)));

    const KeyInfo pose_key_info(&sym::P, timestamp);
    // Pose will be filled in with current values by GraphAction when buffered factors are added to graph
    gtsam::PriorFactor<gtsam::Pose3>::shared_ptr pose_prior_factor(
        new gtsam::PriorFactor<gtsam::Pose3>(pose_key_info.UninitializedKey(), gtsam::Pose3(), pose_noise));

    standstill_prior_factors_to_add.push_back({{pose_key_info}, pose_prior_factor});

    LOG_EVERY_N(INFO, 1) << "AddStandstillPriorFactor: Added pose standstill prior.";
  }

  if (params_.factor.optical_flow_standstill_velocity_prior) {
    const gtsam::Vector3 velocity_prior_noise_sigmas(
        (gtsam::Vector(3) << params_.noise.optical_flow_prior_velocity_stddev,
         params_.noise.optical_flow_prior_velocity_stddev, params_.noise.optical_flow_prior_velocity_stddev)
            .finished());
    const auto velocity_noise =
        Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)));

    const KeyInfo velocity_key_info(&sym::V, timestamp);
    // Velocity will be filled in with current values by GraphAction when buffered factors are added to graph
    gtsam::PriorFactor<gtsam::Velocity3>::shared_ptr velocity_prior_factor(new gtsam::PriorFactor<gtsam::Velocity3>(
        velocity_key_info.UninitializedKey(), gtsam::Velocity3(), velocity_noise));
    standstill_prior_factors_to_add.push_back({{velocity_key_info}, velocity_prior_factor});
    LOG_EVERY_N(INFO, 1) << "AddStandstillPriorFactor: Added velocity standstill prior.";
  }
}

void GraphLocalizer::AddARTagMeasurement(const lm::MatchedProjectionsMeasurement& matched_projections_measurement,
                                         const gtsam::Pose3& dock_cam_T_dock) {
  if (!MeasurementRecentEnough(matched_projections_measurement.timestamp)) {
    LOG(WARNING) << "AddARTagMeasurement: Measurement too old - discarding.";
    return;
  }

  LOG(INFO) << "AddARTagMeasurement: Adding AR tag measurement.";

  dock_cam_T_dock_estimates_.emplace(matched_projections_measurement.timestamp, dock_cam_T_dock);
  AddProjectionMeasurement(matched_projections_measurement, params_.calibration.body_T_dock_cam,
                           params_.calibration.dock_cam_intrinsics, params_.noise.dock_cam_noise,
                           GraphAction::kTransformARMeasurementAndUpdateDockTWorld);
}

void GraphLocalizer::AddSparseMappingMeasurement(
    const lm::MatchedProjectionsMeasurement& matched_projections_measurement) {
  if (!MeasurementRecentEnough(matched_projections_measurement.timestamp)) {
    LOG(WARNING) << "AddSparseMappingMeasurement: Measurement too old - discarding.";
    return;
  }

  LOG(INFO) << "AddSparseMappingMeasurement: Adding sparse mapping measurement.";
  AddProjectionMeasurement(matched_projections_measurement, params_.calibration.body_T_nav_cam,
                           params_.calibration.nav_cam_intrinsics, params_.noise.nav_cam_noise);
}

void GraphLocalizer::AddProjectionMeasurement(const lm::MatchedProjectionsMeasurement& matched_projections_measurement,
                                              const gtsam::Pose3& body_T_cam,
                                              const boost::shared_ptr<gtsam::Cal3_S2>& cam_intrinsics,
                                              const gtsam::SharedNoiseModel& cam_noise,
                                              const GraphAction& graph_action) {
  if (matched_projections_measurement.matched_projections.empty()) {
    LOG(WARNING) << "AddProjectionMeasurement: Empty measurement.";
    return;
  }

  // TODO(rsoussan): Make this a config variable
  constexpr int kMinNumMatches = 5;
  if (matched_projections_measurement.matched_projections.size() < kMinNumMatches) {
    LOG(WARNING) << "AddProjectionMeasurement: Not enough matches in projection measurement.";
    return;
  }

  int num_buffered_loc_projection_factors = 0;
  FactorsToAdd factors_to_add(graph_action);
  factors_to_add.reserve(matched_projections_measurement.matched_projections.size());
  for (const auto& matched_projection : matched_projections_measurement.matched_projections) {
    const KeyInfo key_info(&sym::P, matched_projections_measurement.timestamp);
    gtsam::LocProjectionFactor<>::shared_ptr loc_projection_factor(
        new gtsam::LocProjectionFactor<>(matched_projection.image_point, matched_projection.map_point,
                                         Robust(cam_noise), key_info.UninitializedKey(), cam_intrinsics, body_T_cam));
    factors_to_add.push_back({{key_info}, loc_projection_factor});
    ++num_buffered_loc_projection_factors;
  }
  factors_to_add.SetTimestamp(matched_projections_measurement.timestamp);
  BufferFactors(factors_to_add);

  VLOG(2) << "AddProjectionMeasurement: Buffered " << num_buffered_loc_projection_factors << " loc projection factors.";
}

bool GraphLocalizer::AddOrSplitImuFactorIfNeeded(const lc::Time timestamp) {
  if (graph_values_.HasKey(timestamp)) {
    VLOG(2) << "AddOrSplitImuFactorIfNeeded: CombinedNavState exists at "
               "timestamp, nothing to do.";
    return true;
  }

  const auto latest_timestamp = graph_values_.LatestTimestamp();
  if (!latest_timestamp) {
    LOG(ERROR) << "AddOrSplitImuFactorIfNeeded: Failed to get latest timestamp.";
    return false;
  }

  if (timestamp > *latest_timestamp) {
    VLOG(2) << "AddOrSplitImuFactorIfNeeded: Creating and adding latest imu "
               "factor and nav state.";
    return CreateAndAddLatestImuFactorAndCombinedNavState(timestamp);
  } else {
    VLOG(2) << "AddOrSplitImuFactorIfNeeded: Splitting old imu factor.";
    return SplitOldImuFactorAndAddCombinedNavState(timestamp);
  }
}

bool GraphLocalizer::SplitOldImuFactorAndAddCombinedNavState(const lc::Time timestamp) {
  const auto timestamp_bounds = graph_values_.LowerAndUpperBoundTimestamp(timestamp);
  if (!timestamp_bounds.first || !timestamp_bounds.second) {
    LOG(ERROR) << "SplitOldImuFactorAndAddCombinedNavState: Failed to get upper and lower bound timestamp.";
    return false;
  }

  const lc::Time lower_bound_time = *(timestamp_bounds.first);
  const lc::Time upper_bound_time = *(timestamp_bounds.second);

  if (timestamp < lower_bound_time || timestamp > upper_bound_time) {
    LOG(ERROR) << "SplitOldImuFactorAndAddCombinedNavState: Timestamp is not within bounds of existing timestamps.";
    return false;
  }

  const auto lower_bound_key_index = graph_values_.KeyIndex(lower_bound_time);
  const auto upper_bound_key_index = graph_values_.KeyIndex(upper_bound_time);
  if (!lower_bound_key_index || !upper_bound_key_index) {
    LOG(ERROR) << "SplitOldImuFactorAndAddCombinedNavState: Failed to get lower and upper bound key indices.";
    return false;
  }

  // get old imu factor, delete it
  bool removed_old_imu_factor = false;
  for (auto factor_it = graph_.begin(); factor_it != graph_.end();) {
    if (dynamic_cast<gtsam::CombinedImuFactor*>(factor_it->get()) &&
        graph_values_.ContainsCombinedNavStateKey(**factor_it, *lower_bound_key_index) &&
        graph_values_.ContainsCombinedNavStateKey(**factor_it, *upper_bound_key_index)) {
      graph_.erase(factor_it);
      removed_old_imu_factor = true;
      break;
    }
    ++factor_it;
  }
  if (!removed_old_imu_factor) {
    LOG(ERROR) << "SplitOldImuFactorAndAddCombinedNavState: Failed to remove "
                  "old imu factor.";
    return false;
  }

  const auto lower_bound_bias = graph_values_.at<gtsam::imuBias::ConstantBias>(sym::B(*lower_bound_key_index));
  if (!lower_bound_bias) {
    LOG(ERROR) << "SplitOldImuFactorAndAddCombinedNavState: Failed to get lower bound bias.";
    return false;
  }

  // Add first factor and new nav state at timestamp
  auto first_integrated_pim = latest_imu_integrator_.IntegratedPim(*lower_bound_bias, lower_bound_time, timestamp,
                                                                   latest_imu_integrator_.pim_params());
  if (!first_integrated_pim) {
    LOG(ERROR) << "SplitOldImuFactorAndAddCombinedNavState: Failed to create first integrated pim.";
    return false;
  }

  const auto lower_bound_combined_nav_state = graph_values_.GetCombinedNavState(lower_bound_time);
  if (!lower_bound_combined_nav_state) {
    LOG(ERROR) << "SplitOldImuFactorAndAddCombinedNavState: Failed to get lower bound combined nav state.";
    return false;
  }

  if (!CreateAndAddImuFactorAndPredictedCombinedNavState(*lower_bound_combined_nav_state, *first_integrated_pim)) {
    LOG(ERROR) << "SplitOldImuFactorAndAddCombinedNavState: Failed to create and add imu factor.";
    return false;
  }

  // Add second factor, use lower_bound_bias as starting bias since that is the
  // best estimate available
  auto second_integrated_pim = latest_imu_integrator_.IntegratedPim(*lower_bound_bias, timestamp, upper_bound_time,
                                                                    latest_imu_integrator_.pim_params());
  if (!second_integrated_pim) {
    LOG(ERROR) << "SplitOldImuFactorAndAddCombinedNavState: Failed to create second integrated pim.";
    return false;
  }

  // New nav state already added so just get its key index
  const auto new_key_index = graph_values_.KeyIndex(timestamp);
  if (!new_key_index) {
    LOG(ERROR) << "SplitOldImuFactorAndAddCombinedNavState: Failed to get new key index.";
    return false;
  }

  const auto combined_imu_factor =
      ii::MakeCombinedImuFactor(*new_key_index, *upper_bound_key_index, *second_integrated_pim);
  graph_.push_back(combined_imu_factor);
  return true;
}

bool GraphLocalizer::CreateAndAddLatestImuFactorAndCombinedNavState(const lc::Time timestamp) {
  if (!latest_imu_integrator_.IntegrateLatestImuMeasurements(timestamp)) {
    LOG(ERROR) << "CreateAndAddLatestImuFactorAndCombinedNavState: Failed to integrate latest imu measurements.";
    return false;
  }

  const auto latest_combined_nav_state = graph_values_.LatestCombinedNavState();
  if (!latest_combined_nav_state) {
    LOG(ERROR) << "CreateAndAddLatestImuFactorAndCombinedNavState: Failed to get latest combined nav state.";
    return false;
  }
  if (!CreateAndAddImuFactorAndPredictedCombinedNavState(*latest_combined_nav_state, latest_imu_integrator_.pim())) {
    LOG(ERROR) << "CreateAndAddLatestImuFactorAndCombinedNavState: Failed to create and add imu factor.";
    return false;
  }

  const auto latest_bias = graph_values_.LatestBias();
  if (!latest_bias) {
    LOG(ERROR) << "CreateAndAddLatestImuFactorAndCombinedNavState: Failed to get latest bias.";
    return false;
  }

  latest_imu_integrator_.ResetPimIntegrationAndSetBias(latest_bias->first);
  return true;
}

bool GraphLocalizer::CreateAndAddImuFactorAndPredictedCombinedNavState(
    const lc::CombinedNavState& global_cgN_body, const gtsam::PreintegratedCombinedMeasurements& pim) {
  const auto key_index_0 = graph_values_.KeyIndex(global_cgN_body.timestamp());
  if (!key_index_0) {
    LOG(ERROR) << "CreateAndAddImuFactorAndPredictedCombinedNavState: Failed to get first key index.";
    return false;
  }

  const lc::CombinedNavState global_cgN_body_predicted = ii::PimPredict(global_cgN_body, pim);
  const int key_index_1 = GenerateKeyIndex();
  const auto combined_imu_factor = ii::MakeCombinedImuFactor(*key_index_0, key_index_1, pim);
  graph_.push_back(combined_imu_factor);
  graph_values_.AddCombinedNavState(global_cgN_body_predicted, key_index_1);
  return true;
}

bool GraphLocalizer::SlideWindow(const gtsam::Marginals& marginals) {
  if (graph_values_.SlideWindow(graph_) == 0) {
    VLOG(2) << "SlideWindow: No states removed. ";
    return true;
  }

  const auto oldest_timestamp = graph_values_.OldestTimestamp();
  if (!oldest_timestamp) {
    LOG(ERROR) << "SlideWindow: Failed to get oldest timestamp.";
    return false;
  }

  feature_tracker_.RemoveOldFeaturePoints(*oldest_timestamp);
  latest_imu_integrator_.RemoveOldMeasurements(*oldest_timestamp);
  // Currently this only applies to optical flow smart factors.  Remove if no longer use these
  RemoveOldBufferedFactors(*oldest_timestamp);

  // Add prior to oldest nav state using covariances from last round of
  // optimization
  const auto global_cgN_body_oldest = graph_values_.OldestCombinedNavState();
  if (!global_cgN_body_oldest) {
    LOG(ERROR) << "SlideWindow: Failed to get oldest combined nav state.";
    return false;
  }

  VLOG(2) << "SlideWindow: Oldest state time: " << global_cgN_body_oldest->timestamp();

  const auto key_index = graph_values_.OldestCombinedNavStateKeyIndex();
  if (!key_index) {
    LOG(ERROR) << "SlideWindow: Failed to get oldest combined nav state key index.";
    return false;
  }

  VLOG(2) << "SlideWindow: key index: " << *key_index;

  lc::CombinedNavStateNoise noise;
  noise.pose_noise = Robust(gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(sym::P(*key_index))));
  noise.velocity_noise =
      Robust(gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(sym::V(*key_index))));
  noise.bias_noise = Robust(gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(sym::B(*key_index))));
  AddPriors(*global_cgN_body_oldest, noise, *key_index, graph_values_.values(), graph_);
  return true;
}

void GraphLocalizer::BufferFactors(const FactorsToAdd& factors_to_add) {
  buffered_factors_to_add_.emplace(factors_to_add.timestamp(), factors_to_add);
}

void GraphLocalizer::RemoveOldBufferedFactors(const lc::Time oldest_allowed_timestamp) {
  for (auto factors_to_add_it = buffered_factors_to_add_.begin();
       factors_to_add_it != buffered_factors_to_add_.end();) {
    auto& factors_to_add = factors_to_add_it->second.Get();
    for (auto factor_to_add_it = factors_to_add.begin(); factor_to_add_it != factors_to_add.end();) {
      bool removed_factor = false;
      for (const auto& key_info : factor_to_add_it->key_infos) {
        if (key_info.timestamp() < oldest_allowed_timestamp) {
          LOG(INFO) << "RemoveOldBufferedFactors: Removing old factor from buffered factors.";
          factor_to_add_it = factors_to_add.erase(factor_to_add_it);
          removed_factor = true;
          break;
        }
      }
      if (!removed_factor) ++factor_to_add_it;
    }
    if (factors_to_add_it->second.Get().empty()) {
      LOG(INFO) << "RemoveOldBufferedFactors: Removing old factors from buffered factors.";
      factors_to_add_it = buffered_factors_to_add_.erase(factors_to_add_it);
    } else {
      ++factors_to_add_it;
    }
  }
}

void GraphLocalizer::AddBufferedFactors() {
  LOG(INFO) << "AddBufferedfactors: Adding buffered factors.";
  VLOG(2) << "AddBufferedFactors: Num buffered factors to add: " << buffered_factors_to_add_.size();

  int num_added_factors = 0;
  for (auto factors_to_add_it = buffered_factors_to_add_.begin();
       factors_to_add_it != buffered_factors_to_add_.end() && latest_imu_integrator_.LatestTime() &&
       factors_to_add_it->first <= *(latest_imu_integrator_.LatestTime());) {
    auto& factors_to_add = factors_to_add_it->second;
    for (auto& factor_to_add : factors_to_add.Get()) {
      // Add combined nav states and connecting imu factors for each key in factor if necessary
      // TODO(rsoussan): make this more efficient for factors with multiple keys with the same timestamp?
      for (const auto& key_info : factor_to_add.key_infos) {
        if (!AddOrSplitImuFactorIfNeeded(key_info.timestamp())) {
          LOG(DFATAL) << "AddBufferedFactor: Failed to add or split imu factors necessary for adding factor.";
          continue;
        }
      }

      if (!Rekey(factor_to_add)) {
        LOG(DFATAL) << "AddBufferedMeasurements: Failed to rekey factor to add.";
        continue;
      }
    }

    // Do graph action after adding necessary imu factors and nav states so these are available
    if (!DoGraphAction(factors_to_add)) {
      LOG(ERROR) << "AddBufferedFactors: Failed to complete graph action.";
      continue;
    }

    for (auto& factor_to_add : factors_to_add.Get()) {
      graph_.push_back(factor_to_add.factor);
      ++num_added_factors;
    }
    factors_to_add_it = buffered_factors_to_add_.erase(factors_to_add_it);
  }

  LOG(INFO) << "AddBufferedFactors: Added " << num_added_factors << " factors.";
}

bool GraphLocalizer::DoGraphAction(FactorsToAdd& factors_to_add) {
  switch (factors_to_add.graph_action()) {
    case GraphAction::kNone:
      return true;
    case GraphAction::kDeleteExistingSmartFactors:
      VLOG(2) << "DoGraphAction: Deleting smart factors.";
      DeleteFactors<SmartFactor>();
      return true;
    case GraphAction::kTransformARMeasurementAndUpdateDockTWorld:
      return TransformARMeasurementAndUpdateDockTWorld(factors_to_add);
    case GraphAction::kFillPriors:
      return FillPriorFactors(factors_to_add);
  }

  // Shouldn't occur
  return true;
}

bool GraphLocalizer::Rekey(FactorToAdd& factor_to_add) {
  gtsam::KeyVector new_keys;
  for (const auto& key_info : factor_to_add.key_infos) {
    const auto new_key = graph_values_.GetKey(key_info.key_creator_function(), key_info.timestamp());
    if (!new_key) {
      LOG(ERROR) << "ReKey: Failed to find new key for timestamp.";
      return false;
    }
    new_keys.emplace_back(*new_key);
  }
  factor_to_add.factor->keys() = new_keys;
  return true;
}

bool GraphLocalizer::ReadyToAddMeasurement(const localization_common::Time timestamp) const {
  const auto latest_time = latest_imu_integrator_.LatestTime();
  if (!latest_time) {
    LOG(ERROR) << "ReadyToAddMeasurement: Failed to get latet imu time.";
    return false;
  }

  return (timestamp <= *latest_time);
}

bool GraphLocalizer::TransformARMeasurementAndUpdateDockTWorld(FactorsToAdd& factors_to_add) {
  // Get world_T_dock using current loc estimate since dock can be moved on the ISS.
  // TODO(rsoussan): Optimize this online using config value as a prior, update this
  // config value periodically after running localizer.
  // TODO(rsoussan): Fix timestamp issue of ar tag measurements (uses ros::now instead of timestamp of measurement)
  const auto combined_nav_state = GetCombinedNavState(factors_to_add.timestamp());
  if (!combined_nav_state) {
    LOG(ERROR) << "AddARTagMeasurement: Failed to get combined nav state.";
    return false;
  }

  auto estimated_dock_cam_T_dock_it = dock_cam_T_dock_estimates_.find(factors_to_add.timestamp());
  if (estimated_dock_cam_T_dock_it == dock_cam_T_dock_estimates_.end()) {
    LOG(ERROR) << "TransformARMeasurementAndUpdateDockTWorld: Failed to find dock_cam_T_dock estimate at timestamp.";
    return false;
  }

  const gtsam::Pose3 world_T_body = combined_nav_state->pose();
  estimated_world_T_dock_ =
      std::make_pair(world_T_body * params_.calibration.body_T_dock_cam * estimated_dock_cam_T_dock_it->second,
                     factors_to_add.timestamp());

  // Frame change dock frame of landmark point using updated estimate of world_T_dock_
  for (auto& factor_to_add : factors_to_add.Get()) {
    gtsam::LocProjectionFactor<>* loc_factor = dynamic_cast<gtsam::LocProjectionFactor<>*>(factor_to_add.factor.get());
    if (!loc_factor) {
      LOG(ERROR) << "TransformARMeasurementAndUpdateDockTWorld: Failed to cast factor to loc projection factor.";
      return false;
    }
    loc_factor->landmark_point() = estimated_world_T_dock_->first * loc_factor->landmark_point();
  }

  return true;
}

bool GraphLocalizer::FillPriorFactors(FactorsToAdd& factors_to_add) {
  for (auto& factor_to_add : factors_to_add.Get()) {
    const auto combined_nav_state = GetCombinedNavState(factors_to_add.timestamp());
    if (!combined_nav_state) {
      LOG(ERROR) << "FillPriorFactors: Failed to get combined nav state.";
      return false;
    }

    // TODO(rsoussan): Generalize this better
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factor_to_add.factor.get());
    const auto velocity_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Velocity3>*>(factor_to_add.factor.get());

    // Pose Prior
    if (pose_prior_factor) {
      const auto pose_key = pose_prior_factor->key();
      const auto pose_noise = pose_prior_factor->noiseModel();
      factor_to_add.factor.reset(
          new gtsam::PriorFactor<gtsam::Pose3>(pose_key, combined_nav_state->pose(), pose_noise));
      continue;
    } else if (velocity_prior_factor) {
      const auto velocity_key = velocity_prior_factor->key();
      const auto velocity_noise = velocity_prior_factor->noiseModel();
      factor_to_add.factor.reset(
          new gtsam::PriorFactor<gtsam::Velocity3>(velocity_key, combined_nav_state->velocity(), velocity_noise));
      continue;
    } else {
      LOG(ERROR) << "FillPriorFactors: Unrecognized factor.";
      return false;
    }
  }
  return true;
}

bool GraphLocalizer::MeasurementRecentEnough(const lc::Time timestamp) const {
  if (!latest_imu_integrator_.OldestTime()) {
    LOG(WARNING) << "MeasurementRecentEnough: Waiting until imu measurements have been received.";
    return false;
  }
  if (timestamp < graph_values_.OldestTimestamp()) return false;
  if (timestamp < latest_imu_integrator_.OldestTime()) return false;
  return true;
}

void GraphLocalizer::PrintFactorDebugInfo() const {
  for (auto factor_it = graph_.begin(); factor_it != graph_.end();) {
    const auto factor = dynamic_cast<const SmartFactor*>(factor_it->get());
    if (factor) {
      factor->print();
      VLOG(2) << "PrintFactorDebugInfo: SmartPose Error: " << factor->error(graph_values_.values());
    }
    ++factor_it;
  }
}

int GraphLocalizer::NumOFFactors() const { return NumFactors<SmartFactor>(); }

int GraphLocalizer::NumVLFactors() const { return NumFactors<gtsam::LocProjectionFactor<>>(); }

const GraphValues& GraphLocalizer::graph_values() const { return graph_values_; }

const gtsam::NonlinearFactorGraph& GraphLocalizer::factor_graph() const { return graph_; }

void GraphLocalizer::SaveGraphDotFile(const std::string& output_path) const {
  std::ofstream of(output_path.c_str());
  graph_.saveGraph(of, graph_values_.values());
}

boost::optional<std::pair<Eigen::Isometry3d, lc::Time>> GraphLocalizer::estimated_world_T_dock() const {
  if (!estimated_world_T_dock_) {
    LOG_EVERY_N(WARNING, 50) << "estimated_world_T_dock: Failed to get estimated_world_T_dock.";
    return boost::none;
  }
  return std::make_pair(lc::EigenPose(estimated_world_T_dock_->first), estimated_world_T_dock_->second);
}

bool GraphLocalizer::Update() {
  LOG(INFO) << "Update: Updating.";

  AddBufferedFactors();

  // Optimize
  gtsam::LevenbergMarquardtParams params;
  if (params_.verbose) {
    params.verbosityLM = gtsam::LevenbergMarquardtParams::VerbosityLM::TRYDELTA;
    params.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::LINEAR;
  }
  // TODO(rsoussan): change lin solver?
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, graph_values_.values(), params);
  graph_values_.UpdateValues(optimizer.optimize());

  // Update imu integrator bias
  const auto latest_bias = graph_values_.LatestBias();
  if (!latest_bias) {
    LOG(ERROR) << "Update: Failed to get latest bias.";
    return false;
  }

  latest_imu_integrator_.ResetPimIntegrationAndSetBias(latest_bias->first);

  // Calculate marginals before sliding window since this depends on values that
  // would be removed in SlideWindow()
  marginals_.reset(new gtsam::Marginals(graph_, graph_values_.values()));
  if (!SlideWindow(*marginals_)) {
    LOG(ERROR) << "Update: Failed to slide window.";
    return false;
  }

  return true;
}
}  // namespace graph_localizer
