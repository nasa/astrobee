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
    : GraphOptimizer(params.graph_optimizer, std::unique_ptr<GraphLocalizerStats>(new GraphLocalizerStats())),
      feature_tracker_(new FeatureTracker(params.feature_tracker)),
      latest_imu_integrator_(new ii::LatestImuIntegrator(params.graph_initializer)),
      params_(params) {
  latest_imu_integrator_->SetFanSpeedMode(params_.initial_fan_speed_mode);

  // Initialize Node Updaters
  // Assumes zero initial velocity
  const lc::CombinedNavState global_N_body_start(params_.graph_initializer.global_T_body_start,
                                                 gtsam::Velocity3::Zero(), params_.graph_initializer.initial_imu_bias,
                                                 params_.graph_initializer.start_time);
  params_.combined_nav_state_node_updater.global_N_body_start = global_N_body_start;
  combined_nav_state_node_updater_.reset(
    new CombinedNavStateNodeUpdater(params_.combined_nav_state_node_updater, latest_imu_integrator_, values()));
  combined_nav_state_node_updater_->AddInitialValuesAndPriors(graph_factors());
  AddNodeUpdater(combined_nav_state_node_updater_);
  // TODO(rsoussan): Clean this up
  dynamic_cast<GraphLocalizerStats*>(graph_stats())
    ->SetCombinedNavStateGraphValues(combined_nav_state_node_updater_->shared_graph_values());

  feature_point_node_updater_.reset(new FeaturePointNodeUpdater(params.feature_point_node_updater, values()));
  AddNodeUpdater(feature_point_node_updater_);

  // Initialize Factor Adders
  ar_tag_loc_factor_adder_.reset(
    new LocFactorAdder(params_.factor.ar_tag_loc_adder, go::GraphActionCompleterType::ARTagLocProjectionFactor));
  handrail_factor_adder_.reset(new HandrailFactorAdder(params_.factor.handrail_adder));
  loc_factor_adder_.reset(
    new LocFactorAdder(params_.factor.loc_adder, go::GraphActionCompleterType::LocProjectionFactor));
  projection_factor_adder_.reset(
    new ProjectionFactorAdder(params_.factor.projection_adder, feature_tracker_,
                              feature_point_node_updater_->shared_feature_point_graph_values()));
  rotation_factor_adder_.reset(new RotationFactorAdder(params_.factor.rotation_adder, feature_tracker_));
  smart_projection_cumulative_factor_adder_.reset(
    new SmartProjectionCumulativeFactorAdder(params_.factor.smart_projection_adder, feature_tracker_));
  standstill_factor_adder_.reset(new StandstillFactorAdder(params_.factor.standstill_adder, feature_tracker_));

  // Initialize Graph Action Completers
  ar_tag_loc_graph_action_completer_.reset(
    new LocGraphActionCompleter(params_.factor.ar_tag_loc_adder, go::GraphActionCompleterType::ARTagLocProjectionFactor,
                                combined_nav_state_node_updater_->shared_graph_values()));
  AddGraphActionCompleter(ar_tag_loc_graph_action_completer_);

  loc_graph_action_completer_.reset(
    new LocGraphActionCompleter(params_.factor.loc_adder, go::GraphActionCompleterType::LocProjectionFactor,
                                combined_nav_state_node_updater_->shared_graph_values()));
  AddGraphActionCompleter(loc_graph_action_completer_);
  projection_graph_action_completer_.reset(new ProjectionGraphActionCompleter(
    params_.factor.projection_adder, combined_nav_state_node_updater_->shared_graph_values(),
    feature_point_node_updater_->shared_feature_point_graph_values()));
  AddGraphActionCompleter(projection_graph_action_completer_);
  smart_projection_graph_action_completer_.reset(new SmartProjectionGraphActionCompleter(
    params_.factor.smart_projection_adder, combined_nav_state_node_updater_->shared_graph_values()));
  AddGraphActionCompleter(smart_projection_graph_action_completer_);
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
    LogError("GetCombinedNavState: Failed to create integrated pim.");
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

void GraphLocalizer::AddHandrailMeasurement(const lm::HandrailPointsMeasurement& handrail_points_measurement) {
  if (!MeasurementRecentEnough(handrail_points_measurement.timestamp)) {
    LogDebug("AddHandrailPointsMeasurement: Measurement too old - discarding.");
    return;
  }

  if (params_.factor.handrail_adder.enabled) {
    LogDebug("AddSparseMappingMeasurement: Adding handrail measurement.");
    BufferFactors(handrail_factor_adder_->AddFactors(handrail_points_measurement));
  }
}

void GraphLocalizer::DoPostSlideWindowActions(const localization_common::Time oldest_allowed_time,
                                              const boost::optional<gtsam::Marginals>& marginals) {
  feature_tracker_->RemoveOldFeaturePointsAndSlideWindow(oldest_allowed_time);
  latest_imu_integrator_->RemoveOldMeasurements(oldest_allowed_time);
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
          *smart_factor, factor_key_indices_to_remove, params_.factor.smart_projection_adder,
          smart_projection_cumulative_factor_adder_->smart_projection_params());
        *factor_it = new_smart_factor;
        continue;
      }
    }
  }
}

bool GraphLocalizer::ValidGraph() const {
  // If graph consists of only priors and imu factors, consider it invalid and don't optimize.
  // Make sure smart factors are valid before including them.
  const int num_valid_non_imu_measurement_factors =
    NumOFFactors(true) + go::NumFactors<gtsam::LocPoseFactor>(graph_factors()) +
    go::NumFactors<gtsam::LocProjectionFactor<>>(graph_factors()) +
    go::NumFactors<gtsam::PointToLineFactor>(graph_factors()) +
    go::NumFactors<gtsam::PointToLineSegmentFactor>(graph_factors()) +
    go::NumFactors<gtsam::PointToPlaneFactor>(graph_factors()) +
    go::NumFactors<gtsam::PointToHandrailEndpointFactor>(graph_factors()) +
    go::NumFactors<gtsam::PoseRotationFactor>(graph_factors()) +
    go::NumFactors<gtsam::BetweenFactor<gtsam::Pose3>>(graph_factors());
  return num_valid_non_imu_measurement_factors > 0;
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

int GraphLocalizer::NumFeatures() const { return feature_point_node_updater_->NumFeatures(); }

// TODO(rsoussan): fix this call to happen before of factors are removed!
int GraphLocalizer::NumOFFactors(const bool check_valid) const {
  if (params_.factor.smart_projection_adder.enabled)
    return NumSmartFactors(graph_factors(), combined_nav_state_node_updater_->graph_values().values(), check_valid);
  if (params_.factor.projection_adder.enabled) return NumProjectionFactors(check_valid);
  return 0;
}

int GraphLocalizer::NumProjectionFactors(const bool check_valid) const {
  int num_factors = 0;
  for (const auto& factor : graph_factors()) {
    const auto projection_factor = dynamic_cast<const ProjectionFactor*>(factor.get());
    if (projection_factor) {
      if (check_valid) {
        const auto world_t_point =
          feature_point_node_updater_->feature_point_graph_values().at<gtsam::Point3>(projection_factor->key2());
        if (!world_t_point) {
          LogError("NumProjectionFactors: Failed to get point.");
          continue;
        }
        const auto world_T_body =
          combined_nav_state_node_updater_->graph_values().at<gtsam::Pose3>(projection_factor->key1());
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

const GraphLocalizerStats& GraphLocalizer::graph_localizer_stats() const {
  return *(static_cast<const GraphLocalizerStats*>(graph_stats()));
}

bool GraphLocalizer::standstill() const {
  // If uninitialized, return not at standstill
  // TODO(rsoussan): Is this the appropriate behavior?
  if (!standstill_) return false;
  return *standstill_;
}

bool GraphLocalizer::DoPostOptimizeActions() {
  // Update imu integrator bias
  const auto latest_bias = combined_nav_state_node_updater_->graph_values().LatestBias();
  if (!latest_bias) {
    LogError("Update: Failed to get latest bias.");
    return false;
  }

  latest_imu_integrator_->ResetPimIntegrationAndSetBias(latest_bias->first);
  return true;
}
}  // namespace graph_localizer
