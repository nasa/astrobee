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
#include <imu_integration/imu_utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <glog/logging.h>

namespace graph_localizer {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;

GraphLocalizer::GraphLocalizer(const GraphLocalizerParams& params)
    : latest_imu_integrator_(params.body_T_imu(), params.gyro_bias(), params.accelerometer_bias(), params.start_time(),
                             params.gravity()),
      body_T_nav_cam_(lc::GtPose(params.body_T_nav_cam())),
      nav_cam_intrinsics_(new gtsam::Cal3_S2(params.nav_cam_focal_lengths().x(), params.nav_cam_focal_lengths().y(), 0,
                                             params.nav_cam_principal_point().x(),
                                             params.nav_cam_principal_point().y())),
      nav_cam_noise_(gtsam::noiseModel::Isotropic::Sigma(2, 0.1)),
      body_T_dock_cam_(lc::GtPose(params.body_T_dock_cam())),
      dock_cam_intrinsics_(new gtsam::Cal3_S2(params.dock_cam_focal_lengths().x(), params.dock_cam_focal_lengths().y(),
                                              0, params.dock_cam_principal_point().x(),
                                              params.dock_cam_principal_point().y())),
      dock_cam_noise_(gtsam::noiseModel::Isotropic::Sigma(2, 0.1)),
      graph_values_(params.sliding_window_duration(), params.min_num_sliding_window_states()),
      min_of_avg_distance_from_mean_(params.min_of_avg_distance_from_mean()),
      world_T_dock_(lc::GtPose(params.world_T_dock())) {
  // Assumes zero initial velocity
  const lm::CombinedNavState global_cgN_body_start(
      lc::GtPose(params.global_T_body_start()), gtsam::Velocity3::Zero(),
      gtsam::imuBias::ConstantBias(params.accelerometer_bias(), params.gyro_bias()), params.start_time());

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

void GraphLocalizer::AddStartingPriors(const lm::CombinedNavState& global_cgN_body_start, const int key_index,
                                       const gtsam::Values& values, gtsam::NonlinearFactorGraph& graph) {
  // TODO(rsoussan): tune these
  constexpr double kPoseTranslationPriorSigma = 0.02;
  constexpr double kPoseQuaternionPriorSigma = 0.01;
  constexpr double kVelPriorSigma = 0.01;
  constexpr double kAccelBiasPriorSigma = 0.001;
  constexpr double kGyroBiasPriorSigma = 0.001;
  const gtsam::Vector6 pose_prior_noise_sigmas(
      (gtsam::Vector(6) << kPoseTranslationPriorSigma, kPoseTranslationPriorSigma, kPoseTranslationPriorSigma,
       kPoseQuaternionPriorSigma, kPoseQuaternionPriorSigma, kPoseQuaternionPriorSigma)
          .finished());
  const gtsam::Vector3 velocity_prior_noise_sigmas(
      (gtsam::Vector(3) << kVelPriorSigma, kVelPriorSigma, kVelPriorSigma).finished());
  const gtsam::Vector6 bias_prior_noise_sigmas((gtsam::Vector(6) << kAccelBiasPriorSigma, kAccelBiasPriorSigma,
                                                kAccelBiasPriorSigma, kGyroBiasPriorSigma, kGyroBiasPriorSigma,
                                                kGyroBiasPriorSigma)
                                                   .finished());
  lm::CombinedNavStateNoise noise;
  noise.pose_noise = gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_prior_noise_sigmas));
  noise.velocity_noise =
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas));
  noise.bias_noise = gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(bias_prior_noise_sigmas));
  AddPriors(global_cgN_body_start, noise, key_index, values, graph);
}

void GraphLocalizer::AddPriors(const lm::CombinedNavState& global_cgN_body, const lm::CombinedNavStateNoise& noise,
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

bool GraphLocalizer::LatestCombinedNavStateAndCovariances(
    lm::CombinedNavState& latest_combined_nav_state,
    lm::CombinedNavStateCovariances& latest_combined_nav_state_covariances) const {
  if (!marginals_) {
    LOG(ERROR) << "LatestCombinedNavStateAndCovariances: No marginals available.";
    return false;
  }
  const auto state_covariance_pair = LatestCombinedNavStateAndCovariances(*marginals_);
  latest_combined_nav_state = state_covariance_pair.first;
  latest_combined_nav_state_covariances = state_covariance_pair.second;
  return true;
}

std::pair<lm::CombinedNavState, lm::CombinedNavStateCovariances> GraphLocalizer::LatestCombinedNavStateAndCovariances(
    const gtsam::Marginals& marginals) const {
  const lm::CombinedNavState global_cgN_body_latest = graph_values_.LatestCombinedNavState();
  const int latest_combined_nav_state_key_index = graph_values_.LatestCombinedNavStateKeyIndex();
  const auto pose_covariance = marginals.marginalCovariance(sym::P(latest_combined_nav_state_key_index));
  const auto velocity_covariance = marginals.marginalCovariance(sym::V(latest_combined_nav_state_key_index));
  const auto bias_covariance = marginals.marginalCovariance(sym::B(latest_combined_nav_state_key_index));
  const lm::CombinedNavStateCovariances latest_combined_nav_state_covariances(pose_covariance, velocity_covariance,
                                                                              bias_covariance);
  return {global_cgN_body_latest, latest_combined_nav_state_covariances};
}

bool GraphLocalizer::LatestPose(Eigen::Isometry3d& global_T_body_latest, lc::Time& timestamp) const {
  const lm::CombinedNavState global_cgN_body_latest = graph_values_.LatestCombinedNavState();
  global_T_body_latest = Eigen::Isometry3d(global_cgN_body_latest.pose().matrix());
  timestamp = global_cgN_body_latest.timestamp();
  return true;
}

void GraphLocalizer::LatestBiases(Eigen::Vector3d& accelerometer_bias, Eigen::Vector3d& gyro_bias,
                                  lc::Time& timestamp) const {
  const auto latest_bias = graph_values_.LatestBias();
  accelerometer_bias = latest_bias.accelerometer();
  gyro_bias = latest_bias.gyroscope();
  // Assumes each nav state includes bias estimation
  timestamp = graph_values_.LatestTimestamp();
}

void GraphLocalizer::AddImuMeasurement(const lm::ImuMeasurement& imu_measurement) {
  latest_imu_integrator_.BufferImuMeasurement(imu_measurement);
}

void GraphLocalizer::AddOpticalFlowMeasurement(
    const lm::FeaturePointsMeasurement& optical_flow_feature_points_measurement) {
  // TODO(rsoussan): put these somewhere else? in header?
  using Calibration = gtsam::Cal3_S2;
  using Camera = gtsam::PinholeCamera<Calibration>;
  using SmartFactor = gtsam::SmartProjectionPoseFactor<Calibration>;
  using SharedSmartFactor = boost::shared_ptr<SmartFactor>;

  LOG(INFO) << "AddOpticalFlowMeasurement: Adding optical flow measurement.";
  feature_tracker_.UpdateFeatureTracks(optical_flow_feature_points_measurement.feature_points);

  if (optical_flow_feature_points_measurement.feature_points.empty()) {
    LOG(WARNING) << "AddOpticalFlowMeasurement: Empty measurement.";
    return;
  }
  if (!AddOrSplitImuFactorIfNeeded(optical_flow_feature_points_measurement.timestamp)) {
    LOG(DFATAL) << "AddOpicalFlowMeasurement: Failed to add optical flow measurement.";
    return;
  }

  // Remove all camera factors from graph since SmartFactor does not allow for
  // the removal of a measurement Since measurements are linked to poses which
  // are pruned in the sliding window, need to create new factors each
  // iteration.  TODO(rsoussan): make this more efficient?
  int num_removed_smart_factors = 0;
  for (auto factor_it = graph_.begin(); factor_it != graph_.end();) {
    if (dynamic_cast<SmartFactor*>(factor_it->get())) {
      factor_it = graph_.erase(factor_it);
      ++num_removed_smart_factors;
      continue;
    }
    ++factor_it;
  }
  DLOG(INFO) << "AddOpticalFLowMeasurement: Num removed smart factors: " << num_removed_smart_factors;

  int num_added_smart_factors = 0;
  // Add smart factor for each feature track
  for (const auto& feature_track : feature_tracker_.feature_tracks()) {
    if (!ValidPointSet(feature_track.second.points, min_of_avg_distance_from_mean_)) continue;
    SharedSmartFactor smart_factor(
        new SmartFactor(nav_cam_noise_, nav_cam_intrinsics_, body_T_nav_cam_, smart_projection_params_));
    int num_points_added = 0;
    for (const auto& feature_point : feature_track.second.points) {
      smart_factor->add(Camera::Measurement(feature_point.image_point), graph_values_.PoseKey(feature_point.timestamp));
      ++num_points_added;
    }

    graph_.push_back(smart_factor);
    ++num_added_smart_factors;
  }
  DLOG(INFO) << "AddOpticalFLowMeasurement: Newly added " << num_added_smart_factors << " smart factors.";

  // Optimize Graph on receival of camera images
  // TODO(rsoussan): move this elsewhere???
  Update();
}

void GraphLocalizer::AddARTagMeasurement(const lm::MatchedProjectionsMeasurement& matched_projections_measurement) {
  LOG(INFO) << "AddARTagMeasurement: Adding AR tag measurement.";
  // Hack to delay AR tag measurements until timestamp issue is fixed (uses
  // Ros::now instead of image timestamp, causes AR tag measurements to arrive
  // "ahead" of imu measurements.
  // TODO(rsoussan): Remove this when AR tag timestamp is fixed
  static lm::MatchedProjectionsMeasurement buffered_measurement = matched_projections_measurement;
  static bool add_measurement = false;
  if (!add_measurement) {
    add_measurement = true;
    return;
  }

  lm::FrameChangeMatchedProjectionsMeasurement(buffered_measurement, world_T_dock_);
  AddProjectionMeasurement(buffered_measurement, body_T_dock_cam_, dock_cam_intrinsics_, dock_cam_noise_);
  buffered_measurement = matched_projections_measurement;
}

void GraphLocalizer::AddSparseMappingMeasurement(
    const lm::MatchedProjectionsMeasurement& matched_projections_measurement) {
  LOG(INFO) << "AddSparseMappingMeasurement: Adding sparse mapping measurement.";
  AddProjectionMeasurement(matched_projections_measurement, body_T_nav_cam_, nav_cam_intrinsics_, nav_cam_noise_);
}

void GraphLocalizer::AddProjectionMeasurement(const lm::MatchedProjectionsMeasurement& matched_projections_measurement,
                                              const gtsam::Pose3& body_T_cam,
                                              const boost::shared_ptr<gtsam::Cal3_S2>& cam_intrinsics,
                                              const gtsam::SharedIsotropic& cam_noise) {
  if (matched_projections_measurement.matched_projections.empty()) {
    LOG(WARNING) << "AddProjectionMeasurement: Empty measurement.";
    return;
  }

  if (!AddOrSplitImuFactorIfNeeded(matched_projections_measurement.timestamp)) {
    LOG(DFATAL) << "AddProjectionMeasurement: Failed to add projection measurement.";
    return;
  }

  int num_added_loc_projection_factors = 0;
  for (const auto& matched_projection : matched_projections_measurement.matched_projections) {
    gtsam::LocProjectionFactor<>::shared_ptr loc_projection_factor(new gtsam::LocProjectionFactor<>(
        matched_projection.image_point, matched_projection.map_point, cam_noise,
        graph_values_.PoseKey(matched_projections_measurement.timestamp), cam_intrinsics, body_T_cam));
    graph_.push_back(loc_projection_factor);
    ++num_added_loc_projection_factors;
  }

  DLOG(INFO) << "AddProjectionMeasurement: Added " << num_added_loc_projection_factors << " loc projection factors.";
}

bool GraphLocalizer::AddOrSplitImuFactorIfNeeded(const lc::Time timestamp) {
  if (timestamp < graph_values_.OldestTimestamp()) {
    LOG(WARNING) << "AddOrSplitImuFactorIfNeeded: Timestamp occured before "
                    "oldest time in graph, ignoring.";
    return false;
  }

  if (latest_imu_integrator_.Empty()) {
    LOG(WARNING) << "AddOrSplitImuFactorIfNeeded: No measurements available in "
                    "imu integrator, ignoring.";
    return false;
  }

  if (timestamp < latest_imu_integrator_.OldestTime() || timestamp > latest_imu_integrator_.LatestTime()) {
    LOG(WARNING) << "AddOrSplitImuFactorIfNeeded: Timestamp occured outside "
                    "times available in imu integrator, ignoring."
                 << std::endl
                 << std::setprecision(15) << "Timestamp: " << timestamp << std::endl
                 << "imu oldest: " << latest_imu_integrator_.OldestTime() << std::endl
                 << "imu latest: " << latest_imu_integrator_.LatestTime();
    return false;
  }

  if (graph_values_.HasKey(timestamp)) {
    DLOG(INFO) << "AddOrSplitImuFactorIfNeeded: CombinedNavState exists at "
                  "timestamp, nothing to do.";
    return true;
  }

  if (timestamp > graph_values_.LatestTimestamp()) {
    DLOG(INFO) << "AddOrSplitImuFactorIfNeeded: Creating and adding latest imu "
                  "factor and nav state.";
    CreateAndAddLatestImuFactorAndCombinedNavState(timestamp);
    return true;
  } else {
    DLOG(INFO) << "AddOrSplitImuFactorIfNeeded: Splitting old imu factor.";
    return SplitOldImuFactorAndAddCombinedNavState(timestamp);
  }
}

bool GraphLocalizer::SplitOldImuFactorAndAddCombinedNavState(const lc::Time timestamp) {
  const auto timestamp_bounds = graph_values_.LowerAndUpperBoundTimestamp(timestamp);
  const lc::Time lower_bound_time = timestamp_bounds.first;
  const lc::Time upper_bound_time = timestamp_bounds.second;
  const auto lower_bound_key_index = graph_values_.KeyIndex(lower_bound_time);
  const auto upper_bound_key_index = graph_values_.KeyIndex(upper_bound_time);

  // get old imu factor, delete it
  bool removed_old_imu_factor = false;
  for (auto factor_it = graph_.begin(); factor_it != graph_.end();) {
    if (dynamic_cast<gtsam::CombinedImuFactor*>(factor_it->get()) &&
        graph_values_.ContainsCombinedNavStateKey(**factor_it, lower_bound_key_index) &&
        graph_values_.ContainsCombinedNavStateKey(**factor_it, upper_bound_key_index)) {
      graph_.erase(factor_it);
      removed_old_imu_factor = true;
      break;
    }
    ++factor_it;
  }
  if (!removed_old_imu_factor) {
    LOG(DFATAL) << "SplitOldImuFactorAndAddCombinedNavState: Failed to remove "
                   "old imu factor.";
    return false;
  }

  const auto lower_bound_bias = graph_values_.at<gtsam::imuBias::ConstantBias>(sym::B(lower_bound_key_index));
  // Add first factor and new nav state at timestamp
  auto first_integrated_pim = latest_imu_integrator_.IntegratedPim(lower_bound_bias, lower_bound_time, timestamp,
                                                                   latest_imu_integrator_.pim_params());
  const auto lower_bound_combined_nav_state = graph_values_.GetCombinedNavState(lower_bound_time);
  CreateAndAddImuFactorAndPredictedCombinedNavState(lower_bound_combined_nav_state, first_integrated_pim);

  // Add second factor, use lower_bound_bias as starting bias since that is the
  // best estimate available
  auto second_integrated_pim = latest_imu_integrator_.IntegratedPim(lower_bound_bias, timestamp, upper_bound_time,
                                                                    latest_imu_integrator_.pim_params());
  // New nav state already added so just get its key index
  const auto new_key_index = graph_values_.KeyIndex(timestamp);
  const auto combined_imu_factor =
      ii::MakeCombinedImuFactor(new_key_index, upper_bound_key_index, second_integrated_pim);
  graph_.push_back(combined_imu_factor);
  return true;
}

void GraphLocalizer::CreateAndAddLatestImuFactorAndCombinedNavState(const lc::Time timestamp) {
  latest_imu_integrator_.IntegrateLatestImuMeasurements(timestamp);
  CreateAndAddImuFactorAndPredictedCombinedNavState(graph_values_.LatestCombinedNavState(),
                                                    latest_imu_integrator_.pim());
  latest_imu_integrator_.ResetPimIntegrationAndSetBias(graph_values_.LatestBias());
}

void GraphLocalizer::CreateAndAddImuFactorAndPredictedCombinedNavState(
    const lm::CombinedNavState& global_cgN_body, const gtsam::PreintegratedCombinedMeasurements& pim) {
  const int key_index_0 = graph_values_.KeyIndex(global_cgN_body.timestamp());
  const lm::CombinedNavState global_cgN_body_predicted = ii::PimPredict(global_cgN_body, pim);
  const int key_index_1 = GenerateKeyIndex();
  const auto combined_imu_factor = ii::MakeCombinedImuFactor(key_index_0, key_index_1, pim);
  graph_.push_back(combined_imu_factor);
  graph_values_.AddCombinedNavState(global_cgN_body_predicted, key_index_1);
}

void GraphLocalizer::SlideWindow(const gtsam::Marginals& marginals) {
  if (graph_values_.SlideWindow(graph_) == 0) {
    DLOG(INFO) << "SlideWindow: No states removed. ";
    return;
  }

  feature_tracker_.RemoveOldFeaturePoints(graph_values_.OldestTimestamp());
  latest_imu_integrator_.RemoveOldMeasurements(graph_values_.OldestTimestamp());

  // Add prior to oldest nav state using covariances from last round of
  // optimization
  const lm::CombinedNavState global_cgN_body_oldest = graph_values_.OldestCombinedNavState();
  DLOG(INFO) << "SlideWindow: Oldest state time: " << global_cgN_body_oldest.timestamp();
  const int key_index = graph_values_.OldestCombinedNavStateKeyIndex();
  DLOG(INFO) << "SlideWindow: key index: " << key_index;
  lm::CombinedNavStateNoise noise;
  noise.pose_noise = gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(sym::P(key_index)));
  noise.velocity_noise = gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(sym::V(key_index)));
  noise.bias_noise = gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(sym::B(key_index)));
  AddPriors(global_cgN_body_oldest, noise, key_index, graph_values_.values(), graph_);
}

void GraphLocalizer::PrintFactorDebugInfo() const {
  // TODO(rsoussan): put these using statements somewhere else?
  using Calibration = gtsam::Cal3_S2;
  using Camera = gtsam::PinholeCamera<Calibration>;
  using SmartFactor = gtsam::SmartProjectionPoseFactor<Calibration>;
  using SharedSmartFactor = boost::shared_ptr<SmartFactor>;
  for (auto factor_it = graph_.begin(); factor_it != graph_.end();) {
    if (dynamic_cast<const SmartFactor*>(factor_it->get())) {
      dynamic_cast<const SmartFactor*>(factor_it->get())->print();
      DLOG(INFO) << "PrintFactorDebugInfo: SmartPose Error: "
                 << dynamic_cast<const SmartFactor*>(factor_it->get())->error(graph_values_.values());
    }
    ++factor_it;
  }
}

void GraphLocalizer::Update() {
  LOG(INFO) << "Update: Updating.";

  // Optimize
  // TODO(rsoussan): change lin solver?
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, graph_values_.values());
  graph_values_.UpdateValues(optimizer.optimize());

  // PrintFactorDebugInfo();

  // TODO(rsoussan): put this somewhere else
  const std::string output_path("/home/rsoussan/graph_bag_tests/graph.dot");
  std::ofstream of(output_path.c_str());
  graph_.saveGraph(of, graph_values_.values());

  // Update imu integrator bias
  latest_imu_integrator_.ResetPimIntegrationAndSetBias(graph_values_.LatestBias());
  // Calculate marginals before sliding window since this depends on values that
  // would be removed in SlideWindow()
  marginals_.reset(new gtsam::Marginals(graph_, graph_values_.values()));
  SlideWindow(*marginals_);
}
}  // namespace graph_localizer
