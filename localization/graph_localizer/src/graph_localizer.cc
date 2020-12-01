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

#include <glog/logging.h>

#include <chrono>
#include <iomanip>

namespace {
// TODO(rsoussan): Is this necessary? Just use DFATAL and compile with debug?
// Avoid having to compile with DEBUG to toggle between fatal and non-fatal failures
void log(const bool fatal_failure, const std::string& description) {
  if (fatal_failure) {
    LOG(FATAL) << description;
  } else {
    LOG(ERROR) << description;
  }
}
}  // namespace

namespace graph_localizer {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;

GraphLocalizer::GraphLocalizer(const GraphLocalizerParams& params)
    : feature_tracker_(new FeatureTracker(params.feature_tracker)),
      latest_imu_integrator_(params.graph_initialization),
      graph_values_(params.graph_values),
      params_(params) {
  // Assumes zero initial velocity
  const lc::CombinedNavState global_N_body_start(
    params_.graph_initialization.global_T_body_start, gtsam::Velocity3::Zero(),
    params_.graph_initialization.initial_imu_bias, params_.graph_initialization.start_time);

  // Add first nav state and priors to graph
  const int key_index = GenerateKeyIndex();
  graph_values_.AddCombinedNavState(global_N_body_start, key_index);
  AddStartingPriors(global_N_body_start, key_index, graph_values_.values(), graph_);

  // Initialize smart projection factor params
  smart_projection_params_.verboseCheirality = params_.factor.verbose_cheirality;
  if (params_.factor.degeneracy_mode == "zero_on_degeneracy") {
    smart_projection_params_.setDegeneracyMode(gtsam::DegeneracyMode::ZERO_ON_DEGENERACY);
  } else if (params_.factor.degeneracy_mode == "handle_infinity") {
    smart_projection_params_.setDegeneracyMode(gtsam::DegeneracyMode::HANDLE_INFINITY);
  } else {
    LOG(WARNING) << "GraphLocalizer: No degeneracy mode entered, defaulting to zero on degeneracy.";
    smart_projection_params_.setDegeneracyMode(gtsam::DegeneracyMode::ZERO_ON_DEGENERACY);
  }
  if (params_.factor.linearization_mode == "jacobian_svd") {
    smart_projection_params_.setLinearizationMode(gtsam::LinearizationMode::JACOBIAN_SVD);
  } else if (params_.factor.linearization_mode == "hessian") {
    smart_projection_params_.setLinearizationMode(gtsam::LinearizationMode::HESSIAN);
  } else {
    LOG(WARNING) << "GraphLocalizer: No linearization mode entered, defaulting to jacobian svd.";
    smart_projection_params_.setLinearizationMode(gtsam::LinearizationMode::JACOBIAN_SVD);
  }
  smart_projection_params_.setRankTolerance(1e-9);
  smart_projection_params_.setLandmarkDistanceThreshold(params_.factor.landmark_distance_threshold);
  smart_projection_params_.setDynamicOutlierRejectionThreshold(params_.factor.dynamic_outlier_rejection_threshold);
  smart_projection_params_.setRetriangulationThreshold(params_.factor.retriangulation_threshold);
  smart_projection_params_.setEnableEPI(params_.factor.enable_EPI);

  // Initialize projection triangulation params
  projection_triangulation_params_.rankTolerance = 1e-9;
  projection_triangulation_params_.enableEPI = params_.factor.triangulation_enable_EPI;
  projection_triangulation_params_.landmarkDistanceThreshold = params_.factor.triangulation_landmark_distance_threshold;
  projection_triangulation_params_.dynamicOutlierRejectionThreshold =
    params_.factor.triangulation_dynamic_outlier_rejection_threshold;

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
    LOG(WARNING) << "GraphLocalizer: No marginals factorization entered, defaulting to qr.";
    marginals_factorization_ = gtsam::Marginals::Factorization::QR;
  }

  // Initialize Factor Adders
  ar_tag_loc_factor_adder_.reset(
    new LocFactorAdder(params_.factor.loc_adder, GraphAction::kTransformARMeasurementAndUpdateDockTWorld));
  loc_factor_adder_.reset(new LocFactorAdder(params_.factor.loc_adder));
  rotation_factor_adder_.reset(new RotationFactorAdder(params_.factor.rotation_adder, feature_tracker_));
}

void GraphLocalizer::AddStartingPriors(const lc::CombinedNavState& global_N_body_start, const int key_index,
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
    (gtsam::Vector(6) << params_.noise.starting_prior_accel_bias_stddev, params_.noise.starting_prior_accel_bias_stddev,
     params_.noise.starting_prior_accel_bias_stddev, params_.noise.starting_prior_gyro_bias_stddev,
     params_.noise.starting_prior_gyro_bias_stddev, params_.noise.starting_prior_gyro_bias_stddev)
      .finished());
  lc::CombinedNavStateNoise noise;
  noise.pose_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_prior_noise_sigmas)));
  noise.velocity_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)));
  noise.bias_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(bias_prior_noise_sigmas)));
  AddPriors(global_N_body_start, noise, key_index, values, graph);
}

void GraphLocalizer::AddPriors(const lc::CombinedNavState& global_N_body, const lc::CombinedNavStateNoise& noise,
                               const int key_index, const gtsam::Values& values, gtsam::NonlinearFactorGraph& graph) {
  gtsam::PriorFactor<gtsam::Pose3> pose_prior_factor(sym::P(key_index), global_N_body.pose(), noise.pose_noise);
  graph.push_back(pose_prior_factor);
  gtsam::PriorFactor<gtsam::Velocity3> velocity_prior_factor(sym::V(key_index), global_N_body.velocity(),
                                                             noise.velocity_noise);
  graph.push_back(velocity_prior_factor);
  gtsam::PriorFactor<gtsam::imuBias::ConstantBias> bias_prior_factor(sym::B(key_index), global_N_body.bias(),
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
  const auto global_N_body_latest = graph_values_.LatestCombinedNavState();
  if (!global_N_body_latest) {
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
  return std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>{*global_N_body_latest,
                                                                          latest_combined_nav_state_covariances};
}

boost::optional<lc::CombinedNavState> GraphLocalizer::LatestCombinedNavState() const {
  const auto global_N_body_latest = graph_values_.LatestCombinedNavState();
  if (!global_N_body_latest) {
    LOG(ERROR) << "LatestCombinedNavState: Failed to get latest combined nav state.";
    return boost::none;
  }

  return global_N_body_latest;
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

  // TODO(rsoussan): This is a bug in optical flow node, fix there
  static lc::Time last_time = optical_flow_feature_points_measurement.timestamp;
  if (last_time == optical_flow_feature_points_measurement.timestamp) {
    LOG(ERROR) << "AddOpticalFlowMeasurement: Same timestamp measurement, ignoring.";
    last_time = optical_flow_feature_points_measurement.timestamp;
    return false;
  }
  last_time = optical_flow_feature_points_measurement.timestamp;

  LOG(INFO) << "AddOpticalFlowMeasurement: Adding optical flow measurement.";
  feature_tracker_->UpdateFeatureTracks(optical_flow_feature_points_measurement.feature_points);

  if (optical_flow_feature_points_measurement.feature_points.empty()) {
    LOG(WARNING) << "AddOpticalFlowMeasurement: Empty measurement.";
    return false;
  }

  if (params_.factor.add_smart_factors) AddSmartFactors(optical_flow_feature_points_measurement);
  if (params_.factor.add_projection_factors) AddProjectionFactorsAndPoints(optical_flow_feature_points_measurement);
  if (params_.factor.rotation_adder.enabled) {
    const auto rotation_factors_to_add = rotation_factor_adder_->AddFactors(optical_flow_feature_points_measurement);
    if (rotation_factors_to_add) BufferFactors(*rotation_factors_to_add);
  }

  CheckForStandstill(optical_flow_feature_points_measurement);
  if (standstill() && params_.factor.optical_flow_standstill_velocity_prior) {
    FactorsToAdd prior_factors_to_add;
    AddStandstillVelocityPriorFactor(optical_flow_feature_points_measurement.timestamp, prior_factors_to_add);
    prior_factors_to_add.SetTimestamp(optical_flow_feature_points_measurement.timestamp);
    BufferFactors(prior_factors_to_add);
    VLOG(2) << "AddOpticalFLowMeasurement: Buffered a velocity prior factor.";
  }
  return true;
}

bool GraphLocalizer::AddProjectionFactorsAndPoints(
  const lm::FeaturePointsMeasurement& optical_flow_feature_points_measurement) {
  // Add projection factors for new measurements of already existing features
  FactorsToAdd projection_factors_to_add;
  for (const auto& feature_point : optical_flow_feature_points_measurement.feature_points) {
    if (graph_values_.HasFeature(feature_point.feature_id)) {
      const KeyInfo pose_key_info(&sym::P, feature_point.timestamp);
      const KeyInfo static_point_key_info(&sym::F, feature_point.feature_id);
      const auto point_key = graph_values_.FeatureKey(feature_point.feature_id);
      if (!point_key) {
        LOG(ERROR) << "AddProjectionFactorsAndPoints: Failed to get point key.";
        continue;
      }
      const auto projection_factor = boost::make_shared<ProjectionFactor>(
        feature_point.image_point, Robust(params_.noise.optical_flow_nav_cam_noise), pose_key_info.UninitializedKey(),
        *point_key, params_.calibration.nav_cam_intrinsics, params_.calibration.body_T_nav_cam);
      projection_factors_to_add.push_back({{pose_key_info, static_point_key_info}, projection_factor});
    }
  }
  if (!projection_factors_to_add.empty()) {
    projection_factors_to_add.SetTimestamp(optical_flow_feature_points_measurement.timestamp);
    BufferFactors(projection_factors_to_add);
    VLOG(2) << "AddProjectionFactorsAndPoints: Buffered " << projection_factors_to_add.size()
            << " projection factors for existing features.";
  }

  // Add new feature tracks and measurements if possible
  int new_features = 0;
  for (const auto& feature_track_pair : feature_tracker_->feature_tracks()) {
    const auto& feature_track = feature_track_pair.second;
    if (feature_track.points.size() >= params_.factor.min_num_measurements_for_triangulation &&
        !graph_values_.HasFeature(feature_track.id) &&
        (new_features + graph_values_.NumFeatures()) < params_.factor.max_num_optical_flow_features) {
      // Create new factors to add for each feature track so the graph action can act on only that
      // feature track to triangulate a new point
      FactorsToAdd projection_factors_with_new_point_to_add(GraphAction::kTriangulateNewPoint);
      const auto point_key = graph_values_.CreateFeatureKey();
      for (const auto& feature_point : feature_track.points) {
        const KeyInfo pose_key_info(&sym::P, feature_point.timestamp);
        const KeyInfo static_point_key_info(&sym::F, feature_point.feature_id);
        const auto projection_factor = boost::make_shared<ProjectionFactor>(
          feature_point.image_point, Robust(params_.noise.optical_flow_nav_cam_noise), pose_key_info.UninitializedKey(),
          point_key, params_.calibration.nav_cam_intrinsics, params_.calibration.body_T_nav_cam);
        projection_factors_with_new_point_to_add.push_back({{pose_key_info, static_point_key_info}, projection_factor});
      }
      projection_factors_with_new_point_to_add.SetTimestamp(optical_flow_feature_points_measurement.timestamp);
      BufferFactors(projection_factors_with_new_point_to_add);
      ++new_features;
    }
  }
  if (new_features > 0) VLOG(2) << "AddProjectionFactorsAndPoints: Buffered " << new_features << " new features.";
  return true;
}

void GraphLocalizer::CheckForStandstill(const lm::FeaturePointsMeasurement& optical_flow_feature_points_measurement) {
  // Check for standstill via low disparity for all feature tracks
  double total_average_distance_from_mean = 0;
  int num_valid_feature_tracks = 0;
  for (const auto& feature_track : feature_tracker_->feature_tracks()) {
    const double average_distance_from_mean = AverageDistanceFromMean(feature_track.second.points);
    // Only consider long enough feature tracks for standstill candidates
    if (feature_track.second.points.size() > 5) {
      total_average_distance_from_mean += average_distance_from_mean;
      ++num_valid_feature_tracks;
    }
  }

  double average_distance_from_mean = 0;
  if (num_valid_feature_tracks > 0)
    average_distance_from_mean = total_average_distance_from_mean / num_valid_feature_tracks;

  standstill_ = (num_valid_feature_tracks >= 5 &&
                 average_distance_from_mean <= params_.max_standstill_feature_track_avg_distance_from_mean);
}

bool GraphLocalizer::AddSmartFactors(const lm::FeaturePointsMeasurement& optical_flow_feature_points_measurement) {
  // Add smart factor for each valid feature track
  FactorsToAdd smart_factors_to_add(GraphAction::kDeleteExistingSmartFactors);
  int num_added_smart_factors = 0;
  for (const auto& feature_track : feature_tracker_->feature_tracks()) {
    const double average_distance_from_mean = AverageDistanceFromMean(feature_track.second.points);
    if (ValidPointSet(feature_track.second.points, average_distance_from_mean,
                      params_.factor.min_valid_feature_track_avg_distance_from_mean) &&
        num_added_smart_factors < params_.factor.max_num_optical_flow_factors) {
      AddSmartFactor(feature_track.second, smart_factors_to_add);
      ++num_added_smart_factors;
    }
  }

  if (!smart_factors_to_add.empty()) {
    smart_factors_to_add.SetTimestamp(optical_flow_feature_points_measurement.timestamp);
    BufferFactors(smart_factors_to_add);
    VLOG(2) << "AddOpticalFLowMeasurement: Buffered " << smart_factors_to_add.size() << " smart factors.";
  }
  return true;
}

void GraphLocalizer::AddSmartFactor(const FeatureTrack& feature_track, FactorsToAdd& smart_factors_to_add) {
  SharedRobustSmartFactor smart_factor;
  smart_factor = boost::make_shared<RobustSmartFactor>(
    params_.noise.optical_flow_nav_cam_noise, params_.calibration.nav_cam_intrinsics,
    params_.calibration.body_T_nav_cam, smart_projection_params_, params_.factor.robust_smart_factor,
    params_.factor.enable_rotation_only_fallback);

  KeyInfos key_infos;
  key_infos.reserve(feature_track.points.size());
  // Gtsam requires unique key indices for each key, even though these will be replaced later
  int uninitialized_key_index = 0;
  for (const auto& feature_point : feature_track.points) {
    const KeyInfo key_info(&sym::P, feature_point.timestamp);
    key_infos.emplace_back(key_info);
    smart_factor->add(Camera::Measurement(feature_point.image_point), key_info.MakeKey(uninitialized_key_index++));
  }
  smart_factors_to_add.push_back({key_infos, smart_factor});
}

void GraphLocalizer::AddStandstillVelocityPriorFactor(const lc::Time timestamp,
                                                      FactorsToAdd& standstill_prior_factors_to_add) {
  const gtsam::Vector3 velocity_prior_noise_sigmas(
    (gtsam::Vector(3) << params_.noise.optical_flow_prior_velocity_stddev,
     params_.noise.optical_flow_prior_velocity_stddev, params_.noise.optical_flow_prior_velocity_stddev)
      .finished());
  const auto velocity_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)));

  const KeyInfo velocity_key_info(&sym::V, timestamp);
  gtsam::PriorFactor<gtsam::Velocity3>::shared_ptr velocity_prior_factor(new gtsam::PriorFactor<gtsam::Velocity3>(
    velocity_key_info.UninitializedKey(), gtsam::Velocity3::Zero(), velocity_noise));
  standstill_prior_factors_to_add.push_back({{velocity_key_info}, velocity_prior_factor});
  LOG_EVERY_N(INFO, 1) << "AddStandstillVelocityPriorFactor: Added velocity standstill prior.";
}

void GraphLocalizer::AddARTagMeasurement(const lm::MatchedProjectionsMeasurement& matched_projections_measurement) {
  if (!MeasurementRecentEnough(matched_projections_measurement.timestamp)) {
    LOG(WARNING) << "AddARTagMeasurement: Measurement too old - discarding.";
    return;
  }

  if (params_.factor.ar_tag_loc_adder.enabled) {
    LOG(INFO) << "AddARTagMeasurement: Adding AR tag measurement.";
    // AR projections measurement global frame is dock frame
    dock_cam_T_dock_estimates_.emplace(matched_projections_measurement.timestamp,
                                       matched_projections_measurement.global_T_cam.inverse());
    const auto ar_tag_loc_factors_to_add = ar_tag_loc_factor_adder_->AddFactors(matched_projections_measurement);
    if (ar_tag_loc_factors_to_add) BufferFactors(*ar_tag_loc_factors_to_add);
  }
}

void GraphLocalizer::AddSparseMappingMeasurement(
  const lm::MatchedProjectionsMeasurement& matched_projections_measurement) {
  if (!MeasurementRecentEnough(matched_projections_measurement.timestamp)) {
    LOG(WARNING) << "AddSparseMappingMeasurement: Measurement too old - discarding.";
    return;
  }

  if (params_.factor.loc_adder.enabled) {
    LOG(INFO) << "AddSparseMappingMeasurement: Adding sparse mapping measurement.";
    const auto loc_factors_to_add = loc_factor_adder_->AddFactors(matched_projections_measurement);
    if (loc_factors_to_add) BufferFactors(*loc_factors_to_add);
  }
}

// TODO(rsoussan): Clean this function up (duplicate code), address other todo's
void GraphLocalizer::SplitSmartFactorsIfNeeded(FactorsToAdd& factors_to_add) {
  for (auto& factor_to_add : factors_to_add.Get()) {
    auto smart_factor = dynamic_cast<RobustSmartFactor*>(factor_to_add.factor.get());
    if (!smart_factor) continue;
    // Can't remove measurements if there are only 2 or fewer
    if (smart_factor->measured().size() <= 2) continue;
    const auto point = smart_factor->triangulateSafe(smart_factor->cameras(graph_values_.values()));
    if (point.valid()) continue;
    {
      const auto fixed_smart_factor =
        FixSmartFactorByRemovingIndividualMeasurements(params_, *smart_factor, smart_projection_params_, graph_values_);
      if (fixed_smart_factor) {
        factor_to_add.factor = *fixed_smart_factor;
        continue;
      }
    }
    {
      const auto fixed_smart_factor =
        FixSmartFactorByRemovingMeasurementSequence(params_, *smart_factor, smart_projection_params_, graph_values_);
      if (fixed_smart_factor) {
        factor_to_add.factor = *fixed_smart_factor;
        continue;
      }
    }
    LOG(ERROR) << "SplitSmartFactorsIfNeeded: Failed to fix smart factor";
  }
}

bool GraphLocalizer::TriangulateNewPoint(FactorsToAdd& factors_to_add) {
  gtsam::CameraSet<Camera> camera_set;
  Camera::MeasurementVector measurements;
  gtsam::Key point_key;
  for (const auto& factor_to_add : factors_to_add.Get()) {
    const auto& factor = factor_to_add.factor;
    const auto projection_factor = dynamic_cast<ProjectionFactor*>(factor.get());
    if (!projection_factor) {
      LOG(ERROR) << "TriangulateNewPoint: Failed to cast to projection factor.";
      return false;
    }
    const auto world_T_body = graph_values_.at<gtsam::Pose3>(projection_factor->key1());
    if (!world_T_body) {
      LOG(ERROR) << "TriangulateNewPoint: Failed to get pose.";
      return false;
    }

    const gtsam::Pose3 world_T_camera = *world_T_body * params_.calibration.body_T_nav_cam;
    camera_set.emplace_back(world_T_camera, *params_.calibration.nav_cam_intrinsics);
    measurements.emplace_back(projection_factor->measured());
    point_key = projection_factor->key2();
  }
  gtsam::TriangulationResult world_t_triangulated_point;
  // TODO(rsoussan): Gtsam shouldn't be throwing exceptions for this, but needed if enable_epi enabled.
  // Is there a build setting that prevents cheirality from being thrown in this case?
  try {
    world_t_triangulated_point = gtsam::triangulateSafe(camera_set, measurements, projection_triangulation_params_);
  } catch (...) {
    VLOG(2) << "TriangulateNewPoint: Exception occurred during triangulation";
    return false;
  }

  if (!world_t_triangulated_point.valid()) {
    VLOG(2) << "TriangulateNewPoint: Failed to triangulate point";
    return false;
  }
  // TODO(rsoussan): clean this up
  const auto feature_id = factors_to_add.Get().front().key_infos[1].id();
  graph_values_.AddFeature(feature_id, *world_t_triangulated_point, point_key);
  if (params_.factor.add_point_priors) {
    const gtsam::Vector3 point_prior_noise_sigmas((gtsam::Vector(3) << params_.noise.point_prior_translation_stddev,
                                                   params_.noise.point_prior_translation_stddev,
                                                   params_.noise.point_prior_translation_stddev)
                                                    .finished());
    const auto point_noise =
      Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(point_prior_noise_sigmas)));
    gtsam::PriorFactor<gtsam::Point3> point_prior_factor(point_key, *world_t_triangulated_point, point_noise);
    graph_.push_back(point_prior_factor);
  }
  return true;
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
    const auto timestamps_to_add = TimestampsToAdd(timestamp, *latest_timestamp);
    if (timestamps_to_add.size() > 1)
      VLOG(2) << "AddOrSplitImuFactorIfNeeded: Adding extra imu factors and nav states due to large time difference.";
    bool added_timestamps = true;
    for (const auto timestamp_to_add : timestamps_to_add) {
      added_timestamps &= CreateAndAddLatestImuFactorAndCombinedNavState(timestamp_to_add);
    }
    return added_timestamps;
  } else {
    VLOG(2) << "AddOrSplitImuFactorIfNeeded: Splitting old imu factor.";
    return SplitOldImuFactorAndAddCombinedNavState(timestamp);
  }
}

std::vector<lc::Time> GraphLocalizer::TimestampsToAdd(const lc::Time timestamp, const lc::Time last_added_timestamp) {
  if (!params_.limit_imu_factor_spacing) return {timestamp};
  const double timestamp_difference = timestamp - last_added_timestamp;
  if (timestamp_difference < params_.max_imu_factor_spacing) return {timestamp};
  // Evenly distribute timestamps so that the min number is added such that each spacing is <= max_imu_factor_spacing
  const int num_timestamps_to_add = std::ceil(timestamp_difference / params_.max_imu_factor_spacing);
  const double timestamps_to_add_spacing = timestamp_difference / num_timestamps_to_add;
  std::vector<lc::Time> timestamps_to_add;
  // Add up to final timestamp, insert timestamp argument as final timestamp to ensure no floating point errors occur,
  // since the final timestamp should exactly match the given timestamp
  for (int i = 1; i < num_timestamps_to_add; ++i) {
    const double timestamp_to_add = last_added_timestamp + i * timestamps_to_add_spacing;
    // TODO(rsoussan): Account for recent enough when calculating spacing?
    if (MeasurementRecentEnough(timestamp_to_add)) timestamps_to_add.emplace_back(timestamp_to_add);
  }
  timestamps_to_add.emplace_back(timestamp);

  return timestamps_to_add;
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
  const lc::CombinedNavState& global_N_body, const gtsam::PreintegratedCombinedMeasurements& pim) {
  const auto key_index_0 = graph_values_.KeyIndex(global_N_body.timestamp());
  if (!key_index_0) {
    LOG(ERROR) << "CreateAndAddImuFactorAndPredictedCombinedNavState: Failed to get first key index.";
    return false;
  }

  const lc::CombinedNavState global_N_body_predicted = ii::PimPredict(global_N_body, pim);
  const int key_index_1 = GenerateKeyIndex();
  const auto combined_imu_factor = ii::MakeCombinedImuFactor(*key_index_0, key_index_1, pim);
  graph_.push_back(combined_imu_factor);
  graph_values_.AddCombinedNavState(global_N_body_predicted, key_index_1);
  return true;
}

// Adapted from gtsam::BatchFixedLagSmoother
gtsam::NonlinearFactorGraph GraphLocalizer::MarginalFactors(
  const gtsam::NonlinearFactorGraph& old_factors, const gtsam::KeyVector& old_keys,
  const gtsam::GaussianFactorGraph::Eliminate& eliminate_function) const {
  // Old keys not present in old factors.  This shouldn't occur.
  if (old_keys.size() == 0) {
    VLOG(2) << "MarginalFactors: No old keys provided.";
    return old_factors;
  }

  // Linearize Graph
  const auto linearized_graph = old_factors.linearize(graph_values_.values());
  const auto linear_marginal_factors =
    *(linearized_graph->eliminatePartialMultifrontal(old_keys, eliminate_function).second);
  return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_marginal_factors, graph_values_.values());
}

bool GraphLocalizer::SlideWindow(const boost::optional<gtsam::Marginals>& marginals) {
  const auto new_oldest_time = graph_values_.SlideWindowNewOldestTime();
  if (!new_oldest_time) {
    VLOG(2) << "SlideWindow: No states removed. ";
    return true;
  }

  // Add marginal factors for marginalized values
  auto old_keys = graph_values_.OldKeys(*new_oldest_time);
  auto old_factors = graph_values_.RemoveOldFactors(old_keys, graph_);
  gtsam::KeyVector old_feature_keys;
  if (params_.factor.add_projection_factors) {
    // Call remove old factors before old feature keys, since old feature keys depend on
    // number of factors per key remaining
    old_feature_keys = graph_values_.OldFeatureKeys(graph_);
    auto old_feature_factors = graph_values_.RemoveOldFactors(old_feature_keys, graph_);
    old_keys.insert(old_keys.end(), old_feature_keys.begin(), old_feature_keys.end());
    old_factors.push_back(old_feature_factors);
  }
  if (params_.add_marginal_factors) {
    const auto marginal_factors = MarginalFactors(old_factors, old_keys, gtsam::EliminateQR);
    for (const auto& marginal_factor : marginal_factors) {
      graph_.push_back(marginal_factors);
    }
  }

  graph_values_.RemoveOldCombinedNavStates(*new_oldest_time);
  if (params_.factor.add_projection_factors) graph_values_.RemoveOldFeatures(old_feature_keys, graph_);

  // Remove old data from other containers
  const auto oldest_timestamp = graph_values_.OldestTimestamp();
  if (!oldest_timestamp || oldest_timestamp != *new_oldest_time) {
    LOG(ERROR) << "SlideWindow: Failed to get oldest timestamp.";
    return false;
  }

  feature_tracker_->RemoveOldFeaturePoints(*oldest_timestamp);
  latest_imu_integrator_.RemoveOldMeasurements(*oldest_timestamp);
  // Currently this only applies to optical flow smart factors.  Remove if no longer use these
  RemoveOldBufferedFactors(*oldest_timestamp);

  if (params_.factor.add_projection_factors && params_.factor.add_point_priors && marginals_) {
    UpdatePointPriors(*marginals_);
  }

  if (params_.add_priors) {
    // Add prior to oldest nav state using covariances from last round of
    // optimization
    const auto global_N_body_oldest = graph_values_.OldestCombinedNavState();
    if (!global_N_body_oldest) {
      LOG(ERROR) << "SlideWindow: Failed to get oldest combined nav state.";
      return false;
    }

    VLOG(2) << "SlideWindow: Oldest state time: " << global_N_body_oldest->timestamp();

    const auto key_index = graph_values_.OldestCombinedNavStateKeyIndex();
    if (!key_index) {
      LOG(ERROR) << "SlideWindow: Failed to get oldest combined nav state key index.";
      return false;
    }

    VLOG(2) << "SlideWindow: key index: " << *key_index;

    // Make sure priors are removed before adding new ones
    RemovePriors(*key_index);
    if (marginals) {
      lc::CombinedNavStateNoise noise;
      noise.pose_noise =
        Robust(gtsam::noiseModel::Gaussian::Covariance(marginals->marginalCovariance(sym::P(*key_index))));
      noise.velocity_noise =
        Robust(gtsam::noiseModel::Gaussian::Covariance(marginals->marginalCovariance(sym::V(*key_index))));
      noise.bias_noise =
        Robust(gtsam::noiseModel::Gaussian::Covariance(marginals->marginalCovariance(sym::B(*key_index))));
      AddPriors(*global_N_body_oldest, noise, *key_index, graph_values_.values(), graph_);
    } else {
      // TODO(rsoussan): Add seperate marginal fallback sigmas instead of relying on starting prior sigmas
      AddStartingPriors(*global_N_body_oldest, *key_index, graph_values_.values(), graph_);
    }
  }

  return true;
}

void GraphLocalizer::UpdatePointPriors(const gtsam::Marginals& marginals) {
  const auto feature_keys = graph_values_.FeatureKeys();
  for (const auto& feature_key : feature_keys) {
    const auto world_t_point = graph_values_.at<gtsam::Point3>(feature_key);
    if (!world_t_point) {
      LOG(ERROR) << "UpdatePointPriors: Failed to get world_t_point.";
      continue;
    }
    for (auto factor_it = graph_.begin(); factor_it != graph_.end();) {
      const auto point_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Point3>*>(factor_it->get());
      if (point_prior_factor && (point_prior_factor->key() == feature_key)) {
        // Erase old prior
        factor_it = graph_.erase(factor_it);
        // Add updated one
        const auto point_prior_noise =
          Robust(gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(feature_key)));
        const gtsam::PriorFactor<gtsam::Point3> point_prior_factor(feature_key, *world_t_point, point_prior_noise);
        graph_.push_back(point_prior_factor);
        // Only one point prior per feature
        break;
      } else {
        ++factor_it;
      }
    }
  }
}

void GraphLocalizer::RemovePriors(const int key_index) {
  int removed_factors = 0;
  for (auto factor_it = graph_.begin(); factor_it != graph_.end();) {
    bool erase_factor = false;
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factor_it->get());
    const auto loc_pose_factor = dynamic_cast<gtsam::LocPoseFactor*>(factor_it->get());
    if (pose_prior_factor && !loc_pose_factor && pose_prior_factor->key() == sym::P(key_index)) {
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
      factor_it = graph_.erase(factor_it);
      ++removed_factors;
    } else {
      ++factor_it;
      continue;
    }
  }
  VLOG(2) << "RemovePriors: Erase " << removed_factors << " factors.";
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
        // Ignore static keys
        if (key_info.is_static()) continue;
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
        // Ignore static keys
        if (key_info.is_static()) continue;
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
      VLOG(2) << "AddBufferedFactors: Failed to complete graph action.";
      factors_to_add_it = buffered_factors_to_add_.erase(factors_to_add_it);
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
      DeleteFactors<RobustSmartFactor>();
      // TODO(rsoussan): rename this graph action to handle smart factors
      if (params_.factor.smart_factor_splitting) SplitSmartFactorsIfNeeded(factors_to_add);
      return true;
    case GraphAction::kTransformARMeasurementAndUpdateDockTWorld:
      return TransformARMeasurementAndUpdateDockTWorld(factors_to_add);
    case GraphAction::kTriangulateNewPoint:
      return TriangulateNewPoint(factors_to_add);
  }

  // Shouldn't occur
  return true;
}

bool GraphLocalizer::Rekey(FactorToAdd& factor_to_add) {
  gtsam::KeyVector new_keys;
  const auto& old_keys = factor_to_add.factor->keys();
  for (int i = 0; i < factor_to_add.key_infos.size(); ++i) {
    const auto& key_info = factor_to_add.key_infos[i];
    if (key_info.is_static()) {
      // Don't change static keys. Assumes static key currently in factor is correct
      new_keys.emplace_back(old_keys[i]);
    } else {
      const auto new_key = graph_values_.GetKey(key_info.key_creator_function(), key_info.timestamp());
      if (!new_key) {
        LOG(ERROR) << "ReKey: Failed to find new key for timestamp.";
        return false;
      }
      new_keys.emplace_back(*new_key);
    }
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
  std::vector<FactorToAdd> frame_changed_pose_prior_factors;
  for (auto factor_to_add_it = factors_to_add.Get().begin(); factor_to_add_it != factors_to_add.Get().end();) {
    gtsam::LocProjectionFactor<>* loc_proj_factor =
      dynamic_cast<gtsam::LocProjectionFactor<>*>(factor_to_add_it->factor.get());
    gtsam::LocPoseFactor* loc_pose_factor = dynamic_cast<gtsam::LocPoseFactor*>(factor_to_add_it->factor.get());
    if (!loc_proj_factor && !loc_pose_factor) {
      LOG(ERROR)
        << "TransformARMeasurementAndUpdateDockTWorld: Failed to cast factor to loc projection or prior factor.";
      return false;
    }
    if (loc_proj_factor) {
      loc_proj_factor->landmark_point() = estimated_world_T_dock_->first * loc_proj_factor->landmark_point();
      ++factor_to_add_it;
    } else {
      // Make new factor with changed frame and erase old one since gtsam doesn't allow modifying PriorFactor estimate
      gtsam::LocPoseFactor::shared_ptr frame_changed_pose_prior_factor(
        new gtsam::LocPoseFactor(loc_pose_factor->key(), estimated_world_T_dock_->first * loc_pose_factor->prior(),
                                 loc_pose_factor->noiseModel()));
      frame_changed_pose_prior_factors.emplace_back(factor_to_add_it->key_infos, frame_changed_pose_prior_factor);
      factor_to_add_it = factors_to_add.Get().erase(factor_to_add_it);
    }
  }
  for (const auto& frame_changed_pose_prior_factor : frame_changed_pose_prior_factors) {
    factors_to_add.push_back(frame_changed_pose_prior_factor);
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
  for (const auto& factor : graph_) {
    const auto smart_factor = dynamic_cast<const RobustSmartFactor*>(factor.get());
    if (smart_factor) {
      smart_factor->print();
      if (smart_factor->isValid())
        LOG(WARNING) << "PrintFactorDebugInfo: SmartFactor valid.";
      else
        LOG(WARNING) << "PrintFactorDebugInfo: SmartFactor invalid.";
      if (smart_factor->isDegenerate()) LOG(WARNING) << "PrintFactorDebugInfo: SmartFactor degenerate.";
      if (smart_factor->isPointBehindCamera()) LOG(WARNING) << "PrintFactorDebugInfo: SmartFactor point behind camera.";
      if (smart_factor->isOutlier()) LOG(WARNING) << "PrintFactorDebugInfo: SmartFactor is outlier.";
      if (smart_factor->isFarPoint()) LOG(WARNING) << "PrintFactorDebugInfo: SmartFactor is far point.";
    }
    const auto imu_factor = dynamic_cast<gtsam::CombinedImuFactor*>(factor.get());
    if (imu_factor) {
      LOG(INFO) << "PrintFactorDebugInfo: CombinedImuFactor: " << *imu_factor;
      LOG(INFO) << "PrintFactorDebugInfo: CombinedImuFactor PIM: " << imu_factor->preintegratedMeasurements();
    }
  }
}

void GraphLocalizer::LogErrors() {
  double total_error = 0;
  double optical_flow_factor_error = 0;
  double loc_proj_error = 0;
  double imu_factor_error = 0;
  double rotation_factor_error = 0;
  double pose_prior_error = 0;
  double velocity_prior_error = 0;
  double bias_prior_error = 0;
  for (const auto& factor : graph_) {
    const double error = factor->error(graph_values_.values());
    total_error += error;
    if (params_.factor.add_smart_factors) {
      const auto smart_factor = dynamic_cast<const RobustSmartFactor*>(factor.get());
      if (smart_factor) {
        optical_flow_factor_error += error;
      }
    }
    if (params_.factor.add_projection_factors) {
      const auto projection_factor = dynamic_cast<const ProjectionFactor*>(factor.get());
      if (projection_factor) {
        optical_flow_factor_error += error;
      }
    }
    const auto imu_factor = dynamic_cast<gtsam::CombinedImuFactor*>(factor.get());
    if (imu_factor) {
      imu_factor_error += error;
    }
    const auto loc_factor = dynamic_cast<gtsam::LocProjectionFactor<>*>(factor.get());
    const auto loc_pose_factor = dynamic_cast<gtsam::LocPoseFactor*>(factor.get());
    if (loc_factor || loc_pose_factor) {
      loc_proj_error += error;
    }
    const auto rotation_factor = dynamic_cast<gtsam::PoseRotationFactor*>(factor.get());
    if (rotation_factor) {
      rotation_factor_error += error;
    }
    // Prior Factors
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factor.get());
    if (pose_prior_factor && !loc_pose_factor) {
      pose_prior_error += error;
    }
    const auto velocity_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Velocity3>*>(factor.get());
    if (velocity_prior_factor) {
      velocity_prior_error += error;
    }
    const auto bias_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>*>(factor.get());
    if (bias_prior_factor) {
      bias_prior_error += error;
    }
  }
  total_error_averager_.UpdateAndLog(total_error);
  of_error_averager_.UpdateAndLog(optical_flow_factor_error);
  loc_proj_error_averager_.UpdateAndLog(loc_proj_error);
  imu_error_averager_.UpdateAndLog(imu_factor_error);
  rotation_error_averager_.UpdateAndLog(rotation_factor_error);
  pose_prior_error_averager_.UpdateAndLog(pose_prior_error);
  velocity_prior_error_averager_.UpdateAndLog(velocity_prior_error);
  bias_prior_error_averager_.UpdateAndLog(bias_prior_error);
}

void GraphLocalizer::LogStats() {
  num_states_averager_.UpdateAndLog(graph_values_.NumStates());
  duration_averager_.UpdateAndLog(graph_values_.Duration());
  num_optical_flow_factors_averager_.UpdateAndLog(NumOFFactors());
  num_loc_factors_averager_.UpdateAndLog(NumVLFactors());
  num_imu_factors_averager_.UpdateAndLog(NumFactors<gtsam::CombinedImuFactor>());
  num_rotation_factors_averager_.UpdateAndLog(NumFactors<gtsam::PoseRotationFactor>());
  num_vel_prior_factors_averager_.UpdateAndLog(NumFactors<gtsam::PriorFactor<gtsam::Velocity3>>());
  num_marginal_factors_averager_.UpdateAndLog(NumFactors<gtsam::LinearContainerFactor>());
  if (params_.factor.add_projection_factors) num_features_averager_.UpdateAndLog(NumFeatures());
  num_factors_averager_.UpdateAndLog(graph_.size());
}

int GraphLocalizer::NumFeatures() const { return graph_values_.NumFeatures(); }

// TODO(rsoussan): fix this call to happen before of factors are removed!
int GraphLocalizer::NumOFFactors(const bool check_valid) const {
  if (params_.factor.add_smart_factors) return NumSmartFactors(check_valid);
  if (params_.factor.add_projection_factors) return NumProjectionFactors(check_valid);
  return 0;
}

int GraphLocalizer::NumProjectionFactors(const bool check_valid) const {
  int num_factors = 0;
  for (const auto& factor : graph_) {
    const auto projection_factor = dynamic_cast<const ProjectionFactor*>(factor.get());
    if (projection_factor) {
      if (check_valid) {
        const auto world_t_point = graph_values_.at<gtsam::Point3>(projection_factor->key2());
        if (!world_t_point) {
          LOG(ERROR) << "NumProjectionFactors: Failed to get point.";
          continue;
        }
        const auto world_T_body = graph_values_.at<gtsam::Pose3>(projection_factor->key1());
        if (!world_T_body) {
          LOG(ERROR) << "NumProjectionFactors: Failed to get pose.";
          continue;
        }
        const auto world_T_camera = *world_T_body * params_.calibration.body_T_nav_cam;
        const auto camera_t_point = world_T_camera.inverse() * *world_t_point;
        if (camera_t_point.z() <= 0) {
          VLOG(2) << "NumProjectionFactors: Behind camera.";
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

int GraphLocalizer::NumVLFactors() const {
  if (params_.factor.loc_adder.add_pose_priors)
    return NumFactors<gtsam::LocPoseFactor>();
  else if (params_.factor.loc_adder.add_projections)
    return NumFactors<gtsam::LocProjectionFactor<>>();
  else
    return 0;
}

const GraphValues& GraphLocalizer::graph_values() const { return graph_values_; }

const gtsam::NonlinearFactorGraph& GraphLocalizer::factor_graph() const { return graph_; }

void GraphLocalizer::SaveGraphDotFile(const std::string& output_path) const {
  std::ofstream of(output_path.c_str());
  graph_.saveGraph(of, graph_values_.values());
}

boost::optional<std::pair<gtsam::Pose3, lc::Time>> GraphLocalizer::estimated_world_T_dock() const {
  if (!estimated_world_T_dock_) {
    LOG_EVERY_N(WARNING, 50) << "estimated_world_T_dock: Failed to get estimated_world_T_dock.";
    return boost::none;
  }
  return std::make_pair(estimated_world_T_dock_->first, estimated_world_T_dock_->second);
}

bool GraphLocalizer::standstill() const {
  // If uninitialized, return not at standstill
  // TODO(rsoussan): Is this the appropriate behavior?
  if (!standstill_) return false;
  return *standstill_;
}

bool GraphLocalizer::Update() {
  LOG(INFO) << "Update: Updating.";

  AddBufferedFactors();

  // TODO(rsoussan): Is ordering required? if so clean these calls open and unify with marginalization
  if (params_.add_marginal_factors) {
    // Add graph ordering to place keys that will be marginalized in first group
    const auto new_oldest_time = graph_values_.SlideWindowNewOldestTime();
    if (new_oldest_time) {
      const auto old_keys = graph_values_.OldKeys(*new_oldest_time);
      const auto ordering = gtsam::Ordering::ColamdConstrainedFirst(graph_, old_keys);
      levenberg_marquardt_params_.setOrdering(ordering);
    } else {
      levenberg_marquardt_params_.orderingType = gtsam::Ordering::COLAMD;
    }
  }

  // Optimize
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, graph_values_.values(), levenberg_marquardt_params_);

  optimization_timer_.Start();
  // TODO(rsoussan): Indicate if failure occurs in state msg, perhaps using confidence value for localizer
  try {
    graph_values_.UpdateValues(optimizer.optimize());
  } catch (gtsam::IndeterminantLinearSystemException) {
    log(params_.fatal_failures, "Update: Indeterminant linear system error during optimization, keeping old values.");
  } catch (...) {
    log(params_.fatal_failures, "Update: Graph optimization failed, keeping old values.");
  }
  optimization_timer_.StopAndLog();
  iterations_averager_.UpdateAndLog(optimizer.iterations());
  // TODO(rsoussan): Add options for these?
  LogStats();
  LogErrors();

  if (params_.print_factor_info) PrintFactorDebugInfo();

  // Update imu integrator bias
  const auto latest_bias = graph_values_.LatestBias();
  if (!latest_bias) {
    LOG(ERROR) << "Update: Failed to get latest bias.";
    return false;
  }

  latest_imu_integrator_.ResetPimIntegrationAndSetBias(latest_bias->first);

  // Calculate marginals for covariances
  try {
    marginals_ = gtsam::Marginals(graph_, graph_values_.values(), marginals_factorization_);
  } catch (gtsam::IndeterminantLinearSystemException) {
    log(params_.fatal_failures, "Update: Indeterminant linear system error during computation of marginals.");
    marginals_ = boost::none;
  } catch (...) {
    log(params_.fatal_failures, "Update: Computing marginals failed.");
    marginals_ = boost::none;
  }

  if (!SlideWindow(marginals_)) {
    LOG(ERROR) << "Update: Failed to slide window.";
    return false;
  }

  return true;
}
}  // namespace graph_localizer
