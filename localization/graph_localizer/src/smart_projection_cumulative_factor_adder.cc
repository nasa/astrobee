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

#include <graph_localizer/smart_projection_cumulative_factor_adder.h>
#include <graph_localizer/utilities.h>
#include <graph_optimizer/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/base/Vector.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

namespace graph_localizer {
namespace go = graph_optimizer;
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
  if (params.rotation_only_fallback) smart_projection_params_.setDegeneracyMode(gtsam::DegeneracyMode::HANDLE_INFINITY);
  smart_projection_params_.setEnableEPI(params.enable_EPI);
}

void SmartProjectionCumulativeFactorAdder::AddFactors(
  const FeatureTrackLengthMap& feature_tracks, const int spacing, const double feature_track_min_separation,
  go::FactorsToAdd& smart_factors_to_add, std::unordered_map<lm::FeatureId, lm::FeaturePoint>& added_points) {
  // Iterate in reverse order so longer feature tracks are prioritized
  for (auto feature_track_it = feature_tracks.crbegin(); feature_track_it != feature_tracks.crend();
       ++feature_track_it) {
    if (static_cast<int>(smart_factors_to_add.size()) >= params().max_num_factors) break;
    const auto& feature_track = *(feature_track_it->second);
    const auto points = feature_track.LatestPoints(spacing);
    // Skip already added tracks
    if (added_points.count(points.front().feature_id) > 0) continue;
    const double average_distance_from_mean = AverageDistanceFromMean(points);
    if (ValidPointSet(points.size(), average_distance_from_mean, params().min_avg_distance_from_mean,
                      params().min_num_points) &&
        !TooClose(added_points, points.front(), feature_track_min_separation)) {
      AddSmartFactor(points, smart_factors_to_add);
      // Use latest point
      added_points.emplace(points.front().feature_id, points.front());
    }
  }
}

std::vector<go::FactorsToAdd> SmartProjectionCumulativeFactorAdder::AddFactors() {
  // Add smart factor for each valid feature track
  go::FactorsToAdd smart_factors_to_add(go::GraphActionCompleterType::SmartFactor);
  if (params().use_allowed_timestamps) {
    for (const auto& feature_track : feature_tracker_->feature_tracks()) {
      const auto points = feature_track.second->AllowedPoints(feature_tracker_->smart_factor_timestamp_allow_list());
      const double average_distance_from_mean = AverageDistanceFromMean(points);
      if (ValidPointSet(points.size(), average_distance_from_mean, params().min_avg_distance_from_mean,
                        params().min_num_points) &&
          static_cast<int>(smart_factors_to_add.size()) < params().max_num_factors) {
        AddSmartFactor(points, smart_factors_to_add);
      }
    }
  } else {
    const auto& feature_tracks = feature_tracker_->feature_tracks_length_ordered();
    const auto& longest_feature_track = feature_tracker_->LongestFeatureTrack();
    if (!longest_feature_track) {
      LogDebug("AddFactors: Failed to get longest feature track.");
      return {};
    }
    const int spacing = longest_feature_track->MaxSpacing(params().max_num_points_per_factor);

    std::unordered_map<lm::FeatureId, lm::FeaturePoint> added_points;
    AddFactors(feature_tracks, spacing, params().feature_track_min_separation, smart_factors_to_add, added_points);
    if (static_cast<int>(smart_factors_to_add.size()) < params().max_num_factors) {
      // Zero min separation so any valid feature track is added as a fallback to try to add up to max_num_factors
      AddFactors(feature_tracks, spacing, 0, smart_factors_to_add, added_points);
    }
  }
  if (smart_factors_to_add.empty()) return {};
  const auto latest_timestamp = feature_tracker_->LatestTimestamp();
  if (!latest_timestamp) {
    LogError("AddFactors: Failed to get latest timestamp.");
    return {};
  }
  smart_factors_to_add.SetTimestamp(*latest_timestamp);
  LogDebug("AddFactors: Added " << smart_factors_to_add.size() << " smart factors.");
  return {smart_factors_to_add};
}

void SmartProjectionCumulativeFactorAdder::AddSmartFactor(const std::vector<lm::FeaturePoint>& feature_track_points,
                                                          go::FactorsToAdd& smart_factors_to_add) const {
  SharedRobustSmartFactor smart_factor;
  const int num_feature_track_points = feature_track_points.size();
  const double noise_scale =
    params().scale_noise_with_num_points ? params().noise_scale * num_feature_track_points : params().noise_scale;
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, noise_scale * params().cam_noise->sigma());
  smart_factor =
    boost::make_shared<RobustSmartFactor>(noise, params().cam_intrinsics, params().body_T_cam, smart_projection_params_,
                                          params().rotation_only_fallback, params().robust, params().huber_k);

  go::KeyInfos key_infos;
  key_infos.reserve(feature_track_points.size());
  // Gtsam requires unique key indices for each key, even though these will be replaced later
  int uninitialized_key_index = 0;
  for (int i = 0; i < static_cast<int>(feature_track_points.size()); ++i) {
    const auto& feature_point = feature_track_points[i];
    if (i >= params().max_num_points_per_factor) break;
    const go::KeyInfo key_info(&sym::P, go::NodeUpdaterType::CombinedNavState, feature_point.timestamp);
    key_infos.emplace_back(key_info);
    smart_factor->add(Camera::Measurement(feature_point.image_point), key_info.MakeKey(uninitialized_key_index++));
  }
  smart_factors_to_add.push_back({key_infos, smart_factor});
}

bool SmartProjectionCumulativeFactorAdder::TooClose(
  const std::unordered_map<lm::FeatureId, lm::FeaturePoint>& added_points, const lm::FeaturePoint& point,
  const double feature_track_min_separation) const {
  for (const auto& added_point_pair : added_points) {
    const auto& added_point = added_point_pair.second;
    if (((added_point.image_point - point.image_point).norm()) < feature_track_min_separation) {
      return true;
    }
  }
  return false;
}

const gtsam::SmartProjectionParams& SmartProjectionCumulativeFactorAdder::smart_projection_params() const {
  return smart_projection_params_;
}
}  // namespace graph_localizer
