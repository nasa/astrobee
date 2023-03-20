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

#ifndef FACTOR_ADDERS_VO_SMART_PROJECTION_FACTOR_ADDER_H_
#define FACTOR_ADDERS_VO_SMART_PROJECTION_FACTOR_ADDER_H_

#include <factor_adders/measurement_based_factor_adder.h>
#include <factor_adders/vo_smart_projection_factor_adder_params.h>
#include <localization_common/utilities.h>
#include <localization_measurements/feature_points_measurement.h.h>
#include <vision_common/feature_tracker.h>

#include <vector>

namespace factor_adders {
template <typename PoseVelocityNodeAdderType>
class VoSmartProjectionFactorAdder
    : public MeasurementBasedFactorAdder<localization_measurements::FeaturePointsMeasurement> {
  using Base = MeasurementBasedFactorAdder<localization_measurements::FeaturePointsMeasurement>;

 public:
  VoSmartProjectionFactorAdder(const VoSmartProjectionFactorAdderParams& params,
                               std::shared_ptr<PoseVelocityNodeAdderType> node_adder);

 private:
  // Add factors either using allowed timestamps or max spacing
  int AddMeasurementBasedFactors(const localization_common::Time oldest_allowed_time,
                                 const localization_common::Time newest_allowed_time,
                                 gtsam::NonlinearFactorGraph& factors) final;

  // Add factors using the max spacing between feature point measurements given the max number of
  // feature points to add.
  int AddFactorsUsingMaxSpacing(gtsam::NonlinearFactorGraph& factors) const;

  // Add factors using allowed timestamps which are simply a downsampled set of measurements to acheive the desired
  // spacing.
  int AddFactorsUsingAllowedTimestamps(gtsam::NonlinearFactorGraph& factors) const;

  // Helper function to add factors given a set spacing.
  int AddFactorsUsingSetSpacing(
    const vision_common::FeatureTrackLengthMap& feature_tracks, const int spacing,
    const double feature_track_min_separation,
    std::unordered_map<vision_common::FeatureId, vision_common::FeaturePoint>& added_points) const;

  // Helper function to add a smart factor given a set of feature track points.
  void AddSmartFactor(const std::vector<vision_common::FeaturePoint>& feature_track_points,
                      gtsam::NonlinearFactorGraph& factors) const;

  // Helper function to see if a point is too close in image space to other points belonging to other feature tracks.
  bool TooClose(const std::unordered_map<vision_common::FeatureId, vision_common::FeaturePoint>& added_points,
                const vision_common::FeaturePoint& point, const double feature_track_min_separation) const;

  std::shared_ptr<PoseVelocityNodeAdderType> node_adder_;
  VoSmartProjectionFactorAdderParams params_;
  vision_common::FeatureTracker feature_tracker_;
};

// Implementation
template <typename PoseVelocityNodeAdderType>
VoSmartProjectionFactorAdder<PoseVelocityNodeAdderType>::VoSmartProjectionFactorAdder(
  const SmartProjectionFactorAdderParams& params, std::shared_ptr<PoseVelocityNodeAdderType> node_adder,
  std::shared_ptr<vision_common::FeatureTracker> feature_tracker)
    : Base(params), node_adder_(node_adder), feature_tracker_(feature_tracker) {
  params_.verboseCheirality = params.verbose_cheirality;
  params_.setRankTolerance(1e-9);
  params_.setLandmarkDistanceThreshold(params.landmark_distance_threshold);
  params_.setDynamicOutlierRejectionThreshold(params.dynamic_outlier_rejection_threshold);
  params_.setRetriangulationThreshold(params.retriangulation_threshold);
  if (params.rotation_only_fallback) params_.setDegeneracyMode(gtsam::DegeneracyMode::HANDLE_INFINITY);
  params_.setEnableEPI(params.enable_EPI);
}

template <typename PoseVelocityNodeAdderType>
int VoSmartProjectionFactorAdder<PoseVelocityNodeAdderType>::AddMeasurementBasedFactors(
  const localization_common::Time oldest_allowed_time, const localization_common::Time newest_allowed_time, ,
  gtsam::NonlinearFactorGraph& factors) {
  // Update feature tracker with new measurements
  for (const auto& measurement : measurements_.set()) {
    feature_tracker_->UpdateFeatureTracks(measurement.feature_points);
  }

  // Remove old smart factors before adding new ones, since otherwise there would be repeat factors
  // for already existing feature tracks
  localization_common::DeleteFactors<RobustSmartFactor>(factors);

  // Create smart factors based on feature tracks
  int num_added_factors = 0;
  if (params_.use_allowed_timestamps) {
    num_added_factors = AddFactorsUsingAllowedTimestamps(factors);
  } else {
    num_added_factors = AddFactorsUsingMaxSpacing(factors);
  }
  feature_tracker_.RemoveOldFeaturePointsAndSlideWindow(oldest_allowed_time);
  return num_added_factors;
}

template <typename PoseVelocityNodeAdderType>
int VoSmartProjectionFactorAdder<PoseVelocityNodeAdderType>::AddFactorsUsingMaxSpacing(
  gtsam::NonlinearFactorGraph& factors) {
  const auto& feature_tracks = feature_tracker_->feature_tracks_length_ordered();
  const auto& longest_feature_track = feature_tracker_->LongestFeatureTrack();
  if (!longest_feature_track) {
    LogDebug("AddFactors: Failed to get longest feature track.");
    return {};
  }
  const int spacing = longest_feature_track->MaxSpacing(params_.max_num_points_per_factor);

  std::unordered_map<vc::FeatureId, vc::FeaturePoint> added_points;
  AddFactorsUsingSetSpacing(feature_tracks, spacing, params_.feature_track_min_separation, smart_factors_to_add,
                            added_points);
  if (static_cast<int>(smart_factors_to_add.size()) < params_.max_num_factors) {
    // Zero min separation so any valid feature track is added as a fallback to try to add up to max_num_factors
    AddFactorsUsingSetSpacing(feature_tracks, spacing, 0, smart_factors_to_add, added_points);
  }
}

template <typename PoseVelocityNodeAdderType>
int VoSmartProjectionFactorAdder<PoseVelocityNodeAdderType>::AddFactorsUsingAllowedTimestamps(
  gtsam::NonlinearFactorGraph& factors) {
  for (const auto& feature_track : feature_tracker_->feature_tracks()) {
    const auto points = feature_track.second->AllowedPoints(feature_tracker_->smart_factor_timestamp_allow_list());
    const double average_distance_from_mean = vc::AverageDistanceFromMean(points);
    if (vc::ValidPointSet(points.size(), average_distance_from_mean, params_.min_avg_distance_from_mean,
                          params_.min_num_points) &&
        static_cast<int>(smart_factors_to_add.size()) < params_.max_num_factors) {
      AddSmartFactor(points, smart_factors_to_add);
    }
  }
}

template <typename PoseVelocityNodeAdderType>
int VoSmartProjectionFactorAdder<PoseVelocityNodeAdderType>::AddFactorsUsingSetSpacing(
  const vc::FeatureTrackLengthMap& feature_tracks, const int spacing, const double feature_track_min_separation,
  std::unordered_map<vc::FeatureId, vc::FeaturePoint>& added_points) {
  // Iterate in reverse order so longer feature tracks are prioritized
  for (auto feature_track_it = feature_tracks.crbegin(); feature_track_it != feature_tracks.crend();
       ++feature_track_it) {
    if (static_cast<int>(smart_factors_to_add.size()) >= params_.max_num_factors) break;
    const auto& feature_track = *(feature_track_it->second);
    const auto points = feature_track.LatestPoints(spacing);
    // Skip already added tracks
    if (added_points.count(points.front().feature_id) > 0) continue;
    const double average_distance_from_mean = vc::AverageDistanceFromMean(points);
    if (vc::ValidPointSet(points.size(), average_distance_from_mean, params_.min_avg_distance_from_mean,
                          params_.min_num_points) &&
        !TooClose(added_points, points.front(), feature_track_min_separation)) {
      AddSmartFactor(points, factors);
      // Use latest point
      added_points.emplace(points.front().feature_id, points.front());
    }
  }
}

template <typename PoseVelocityNodeAdderType>
void VoSmartProjectionFactorAdder<PoseVelocityNodeAdderType>::AddSmartFactor(
  const std::vector<vc::FeaturePoint>& feature_track_points, gtsam::NonlinearFactorGraph& factors) const {
  SharedRobustSmartFactor smart_factor;
  const int num_feature_track_points = feature_track_points.size();
  // Optionally scale the noise with the number of points, as a longer track the relies on    a
  // longer history of poses tends to have higher error
  const double noise_scale =
    params_.scale_noise_with_num_points ? params_.noise_scale * num_feature_track_points : params_.noise_scale;
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, noise_scale * params_.cam_noise->sigma());
  smart_factor =
    boost::make_shared<RobustSmartFactor>(noise, params_.cam_intrinsics, params_.body_T_cam, smart_projection_params_,
                                          params_.rotation_only_fallback, params_.robust, params_.huber_k);

  for (int i = 0; i < static_cast<int>(feature_track_points.size()); ++i) {
    const auto& feature_point = feature_track_points[i];
    if (i >= params_.max_num_points_per_factor) break;
    // Add or get pose key
    if (!node_adder_->AddNode(feature_point.timestamp, factors)) {
      LogError("AddSmartFactor: Failed to add node for timestamp " << timestamp << ".");
      return boost::none;
    }
    const auto keys = node_adder_->Keys(timestamp);
    if (keys.empty()) {
      LogError("AddSmartFactor: Failed to get keys for timestamp " << timestamp << ".");
      return boost::none;
    }
    // Assumes first key is pose
    const auto& pose_key = keys[0];
    smart_factor->add(Camera::Measurement(feature_point.image_point), pose_key);
    factors.push_back(smart_factor);
  }
}

template <typename PoseVelocityNodeAdderType>
bool VoSmartProjectionFactorAdder<PoseVelocityNodeAdderType>::TooClose(
  const std::unordered_map<vc::FeatureId, vc::FeaturePoint>& added_points, const vc::FeaturePoint& point,
  const double feature_track_min_separation) const {
  for (const auto& added_point_pair : added_points) {
    const auto& added_point = added_point_pair.second;
    if (((added_point.image_point - point.image_point).norm()) < feature_track_min_separation) {
      return true;
    }
  }
  return false;
}
}  // namespace factor_adders
#endif  // FACTOR_ADDERS_VO_SMART_PROJECTION_FACTOR_ADDER_H_
