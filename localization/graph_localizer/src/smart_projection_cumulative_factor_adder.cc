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

#include <graph_localizer/graph_action.h>
#include <graph_localizer/smart_projection_cumulative_factor_adder.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/base/Vector.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

namespace graph_localizer {
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
SmartProjectionCumulativeFactorAdder::SmartProjectionCumulativeFactorAdder(
  const SmartProjectionFactorAdderParams& params, std::shared_ptr<const FeatureTracker> feature_tracker)
    : SmartProjectionCumulativeFactorAdder::Base(params), feature_tracker_(feature_tracker) {
  smart_projection_params_.verboseCheirality = params.verbose_cheirality;
  smart_projection_params_.setRankTolerance(1e-9);
  smart_projection_params_.setLandmarkDistanceThreshold(params.landmark_distance_threshold);
  smart_projection_params_.setDynamicOutlierRejectionThreshold(params.dynamic_outlier_rejection_threshold);
  smart_projection_params_.setRetriangulationThreshold(params.retriangulation_threshold);
  smart_projection_params_.setEnableEPI(params.enable_EPI);
}

std::vector<FactorsToAdd> SmartProjectionCumulativeFactorAdder::AddFactors() {
  // Add smart factor for each valid feature track
  FactorsToAdd smart_factors_to_add(GraphAction::kDeleteExistingSmartFactors);
  int num_added_smart_factors = 0;
  for (const auto& feature_track : feature_tracker_->feature_tracks()) {
    const double average_distance_from_mean = AverageDistanceFromMean(feature_track.second.points);
    if (ValidPointSet(feature_track.second.points, average_distance_from_mean, params().min_avg_distance_from_mean,
                      params().min_num_points) &&
        num_added_smart_factors < params().max_num_factors) {
      AddSmartFactor(feature_track.second, smart_factors_to_add);
      ++num_added_smart_factors;
    }
  }

  if (smart_factors_to_add.empty()) return {};
  const auto latest_timestamp = feature_tracker_->latest_timestamp();
  if (!latest_timestamp) {
    LogError("AddFactors: Failed to get latest timestamp.");
    return {};
  }
  smart_factors_to_add.SetTimestamp(*latest_timestamp);
  LogDebug("AddFactors: Added " << smart_factors_to_add.size() << " smart factors.");
  return {smart_factors_to_add};
}

void SmartProjectionCumulativeFactorAdder::AddSmartFactor(const FeatureTrack& feature_track,
                                                          FactorsToAdd& smart_factors_to_add) const {
  SharedRobustSmartFactor smart_factor;
  const int num_feature_track_points = feature_track.points.size();
  const auto noise = params().scale_noise_with_num_points
                       ? gtsam::noiseModel::Isotropic::Sigma(
                           2, params().noise_scale * num_feature_track_points * params().cam_noise->sigma())
                       : params().cam_noise;
  smart_factor =
    boost::make_shared<RobustSmartFactor>(noise, params().cam_intrinsics, params().body_T_cam, smart_projection_params_,
                                          params().rotation_only_fallback, params().robust, params().huber_k);

  KeyInfos key_infos;
  key_infos.reserve(feature_track.points.size());
  // Gtsam requires unique key indices for each key, even though these will be replaced later
  int uninitialized_key_index = 0;
  for (int i = 0; i < feature_track.points.size(); ++i) {
    const auto& feature_point = feature_track.points[i];
    if (i >= params().max_num_points_per_factor) break;
    const KeyInfo key_info(&sym::P, feature_point.timestamp);
    key_infos.emplace_back(key_info);
    smart_factor->add(Camera::Measurement(feature_point.image_point), key_info.MakeKey(uninitialized_key_index++));
  }
  smart_factors_to_add.push_back({key_infos, smart_factor});
}
}  // namespace graph_localizer
