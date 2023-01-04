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
#include <graph_localizer/point_to_handrail_endpoint_factor.h>
#include <graph_localizer/point_to_line_factor.h>
#include <graph_localizer/point_to_line_segment_factor.h>
#include <graph_localizer/point_to_plane_factor.h>
#include <graph_localizer/point_to_point_between_factor.h>
#include <graph_localizer/pose_rotation_factor.h>
#include <graph_localizer/utilities.h>
#include <graph_optimizer/utilities.h>
#include <imu_integration/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/linear/NoiseModel.h>
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
    : GraphOptimizer(params.graph_optimizer, std::unique_ptr<GraphLocalizerStats>(new GraphLocalizerStats())),
      params_(params) {
  // TODO(rsoussan): Add this param to imu integrator, set on construction
  InitializeNodeUpdaters();
  InitializeFactorAdders();
  InitializeGraphActionCompleters();
}

void GraphLocalizer::InitializeNodeUpdaters() {
/*  const lc::CombinedNavState global_N_body_start(
    params_.graph_initializer.global_T_body_start, params_.graph_initializer.global_V_body_start,
    params_.graph_initializer.initial_imu_bias, params_.graph_initializer.start_time);
  params_.combined_nav_state_node_updater.global_N_body_start = global_N_body_start;
  combined_nav_state_node_updater_.reset(
    new CombinedNavStateNodeUpdater(params_.combined_nav_state_node_updater, latest_imu_integrator_, shared_values()));
  combined_nav_state_node_updater_->AddInitialValuesAndPriors(graph_factors());
  AddNodeUpdater(combined_nav_state_node_updater_);
  // TODO(rsoussan): Clean this up
  dynamic_cast<GraphLocalizerStats*>(graph_stats())
    ->SetCombinedNavStateGraphValues(combined_nav_state_node_updater_->shared_graph_values());*/
}

void GraphLocalizer::InitializeFactorAdders() {
  ar_tag_loc_factor_adder_.reset(
    new LocFactorAdder(params_.factor.ar_tag_loc_adder, go::GraphActionCompleterType::ARTagLocProjectionFactor));
  depth_odometry_factor_adder_.reset(new DepthOdometryFactorAdder(params_.factor.depth_odometry_adder));
  handrail_factor_adder_.reset(new HandrailFactorAdder(params_.factor.handrail_adder));
  loc_factor_adder_.reset(
    new LocFactorAdder(params_.factor.loc_adder, go::GraphActionCompleterType::LocProjectionFactor));
  rotation_factor_adder_.reset(new RotationFactorAdder(params_.factor.rotation_adder, feature_tracker_));
}

void GraphLocalizer::InitializeGraphActionCompleters() {
  ar_tag_loc_graph_action_completer_.reset(
    new LocGraphActionCompleter(params_.factor.ar_tag_loc_adder, go::GraphActionCompleterType::ARTagLocProjectionFactor,
                                combined_nav_state_node_updater_->shared_graph_values()));
  AddGraphActionCompleter(ar_tag_loc_graph_action_completer_);

  loc_graph_action_completer_.reset(
    new LocGraphActionCompleter(params_.factor.loc_adder, go::GraphActionCompleterType::LocProjectionFactor,
                                combined_nav_state_node_updater_->shared_graph_values()));
  AddGraphActionCompleter(loc_graph_action_completer_);
}

boost::optional<std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>>
GraphLocalizer::LatestCombinedNavStateAndCovariances() const {
  if (!marginals()) {
    LogDebugEveryN(50, "LatestCombinedNavStateAndCovariances: No marginals available.");
    return boost::none;
  }
  const auto state_covariance_pair = LatestCombinedNavStateAndCovariances(*marginals());
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
  const auto global_N_body_latest = combined_nav_state_node_updater_->graph_values().LatestCombinedNavState();
  if (!global_N_body_latest) {
    LogError("LatestCombinedNavStateAndCovariance: Failed to get latest combined nav state.");
    return boost::none;
  }
  const auto latest_combined_nav_state_key_index =
    combined_nav_state_node_updater_->graph_values().LatestCombinedNavStateKeyIndex();
  if (!latest_combined_nav_state_key_index) {
    LogError("LatestCombinedNavStateAndCovariance: Failed to get latest combined nav state.");
    return boost::none;
  }

  try {
    const auto pose_covariance = marginals.marginalCovariance(sym::P(*latest_combined_nav_state_key_index));
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
  const auto global_N_body_latest = combined_nav_state_node_updater_->graph_values().LatestCombinedNavState();
  if (!global_N_body_latest) {
    LogError("LatestCombinedNavState: Failed to get latest combined nav state.");
    return boost::none;
  }

  return global_N_body_latest;
}

boost::optional<lc::CombinedNavState> GraphLocalizer::GetCombinedNavState(const lc::Time time) const {
  const auto lower_bound_or_equal_combined_nav_state =
    combined_nav_state_node_updater_->graph_values().LowerBoundOrEqualCombinedNavState(time);
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
    LogDebug("GetCombinedNavState: Failed to create integrated pim.");
    return boost::none;
  }

  return ii::PimPredict(*lower_bound_or_equal_combined_nav_state, *integrated_pim);
}

boost::optional<std::pair<gtsam::imuBias::ConstantBias, lc::Time>> GraphLocalizer::LatestBiases() const {
  const auto latest_bias = combined_nav_state_node_updater_->graph_values().LatestBias();
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

void GraphLocalizer::AddHandrailMeasurement(const lm::HandrailPointsMeasurement& handrail_points_measurement) {
  if (!MeasurementRecentEnough(handrail_points_measurement.timestamp)) {
    LogDebug("AddHandrailPointsMeasurement: Measurement too old - discarding.");
    return;
  }

  if (params_.factor.handrail_adder.enabled) {
    LogDebug("AddHandrailPointsMeasurement: Adding handrail measurement.");
    BufferFactors(handrail_factor_adder_->AddFactors(handrail_points_measurement));
  }
}

void GraphLocalizer::AddDepthOdometryMeasurement(const lm::DepthOdometryMeasurement& depth_odometry_measurement) {
  if (!MeasurementRecentEnough(depth_odometry_measurement.timestamp)) {
    LogDebug("AddDepthOdometryMeasurement: Measurement too old - discarding.");
    return;
  }

  if (params_.factor.depth_odometry_adder.enabled) {
    LogDebug("AddDepthOdometryMeasurement: Adding depth odometry measurement.");
    BufferFactors(depth_odometry_factor_adder_->AddFactors(depth_odometry_measurement));
  }
}

void GraphLocalizer::DoPostSlideWindowActions(const localization_common::Time oldest_allowed_time,
                                              const boost::optional<gtsam::Marginals>& marginals) {
  feature_tracker_->RemoveOldFeaturePointsAndSlideWindow(oldest_allowed_time);
}

void GraphLocalizer::BufferCumulativeFactors() {}

void GraphLocalizer::RemoveOldMeasurementsFromCumulativeFactors(const gtsam::KeyVector& old_keys) {}

bool GraphLocalizer::ValidGraph() const {
  // TODO(rsoussan): get num factors from gtsam
  return true;
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
  if (!GraphOptimizer::MeasurementRecentEnough(timestamp)) return false;
  if (!latest_imu_integrator_->OldestTime()) {
    LogDebug("MeasurementRecentEnough: Waiting until imu measurements have been received.");
    return false;
  }
  if (timestamp < latest_imu_integrator_->OldestTime()) return false;
  return true;
}

void GraphLocalizer::PrintFactorDebugInfo() const {
  for (const auto& factor : graph_factors()) {
    // TODO(rsoussan): Fill this in
  }
}

const CombinedNavStateGraphValues& GraphLocalizer::combined_nav_state_graph_values() const {
  return combined_nav_state_node_updater_->graph_values();
}

const CombinedNavStateNodeUpdater& GraphLocalizer::combined_nav_state_node_updater() const {
  return *combined_nav_state_node_updater_;
}

const GraphLocalizerParams& GraphLocalizer::params() const { return params_; }

const GraphLocalizerStats& GraphLocalizer::graph_localizer_stats() const {
  return *(static_cast<const GraphLocalizerStats*>(graph_stats()));
}

bool GraphLocalizer::DoPostOptimizeActions() {
  return true;
}
}  // namespace graph_localizer
