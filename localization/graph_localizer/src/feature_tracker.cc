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

#include <graph_localizer/feature_tracker.h>
#include <localization_common/logger.h>

namespace graph_localizer {
namespace lc = localization_common;
namespace lm = localization_measurements;
FeatureTracker::FeatureTracker(const FeatureTrackerParams& params) : params_(params) {}
void FeatureTracker::UpdateFeatureTracks(const lm::FeaturePoints& feature_points) {
  if (feature_points.empty()) {
    Clear();
    LogDebug("UpdateFeatureTracks: Removed all feature tracks.");
    return;
  }

  const int starting_num_feature_tracks = size();
  LogDebug("UpdateFeatureTracks: Starting num feature tracks: " << starting_num_feature_tracks);
  for (const auto& feature_point : feature_points) {
    AddOrUpdateTrack(feature_point);
  }
  const int post_add_num_feature_tracks = size();
  LogDebug("UpdateFeatureTracks: Added feature tracks: " << post_add_num_feature_tracks - starting_num_feature_tracks);

  // Remove features that weren't detected
  const auto feature_points_timestamp = feature_points.front().timestamp;
  RemoveUndetectedFeatures(feature_points_timestamp);
  UpdateLengthMap();
  const int removed_num_feature_tracks = post_add_num_feature_tracks - size();
  LogDebug("UpdateFeatureTracks: Removed feature tracks: " << removed_num_feature_tracks);
  LogDebug("UpdateFeatureTracks: Final total num feature tracks: " << size());
}

void FeatureTracker::RemoveOldFeaturePointsAndSlideWindow(boost::optional<lc::Time> oldest_allowed_time) {
  const auto latest_time = LatestTimestamp();
  if (!latest_time) return;
  const lc::Time window_start = *latest_time - params_.sliding_window_duration;
  if (window_start <= 0 && !oldest_allowed_time) return;
  oldest_allowed_time = oldest_allowed_time ? std::max(*oldest_allowed_time, window_start) : window_start;

  for (auto feature_track : feature_track_id_map_) {
    feature_track.second->RemoveOldMeasurements(*oldest_allowed_time);
  }

  UpdateLengthMap();
}

void FeatureTracker::RemoveUndetectedFeatures(const lc::Time& feature_point_timestamp) {
  for (auto feature_it = feature_track_id_map_.cbegin(); feature_it != feature_track_id_map_.cend();) {
    if (!feature_it->second->HasMeasurement(feature_point_timestamp)) {
      feature_it = feature_track_id_map_.erase(feature_it);
    } else {
      ++feature_it;
    }
  }
}

void FeatureTracker::AddOrUpdateTrack(const lm::FeaturePoint& feature_point) {
  if (feature_track_id_map_.count(feature_point.feature_id) == 0) {
    feature_track_id_map_[feature_point.feature_id] = std::make_shared<FeatureTrack>(feature_point.feature_id);
  }
  feature_track_id_map_[feature_point.feature_id]->AddMeasurement(feature_point.timestamp, feature_point);
}

void FeatureTracker::UpdateLengthMap() {
  feature_track_length_map_.clear();
  for (const auto& feature_track : feature_track_id_map_) {
    feature_track_length_map_.emplace(feature_track.second->size(), feature_track.second);
  }
}

const FeatureTrackIdMap& FeatureTracker::feature_tracks() const { return feature_track_id_map_; }

const FeatureTrackLengthMap& FeatureTracker::feature_tracks_length_ordered() const { return feature_track_length_map_; }

int FeatureTracker::NumTracksWithAtLeastNPoints(int n) const {
  const auto lower_bound_it = feature_track_length_map_.lower_bound(n);
  return std::distance(lower_bound_it, feature_track_length_map_.end());
}

size_t FeatureTracker::size() const { return feature_track_id_map_.size(); }

bool FeatureTracker::empty() const { return feature_track_id_map_.empty(); }

void FeatureTracker::Clear() {
  feature_track_id_map_.clear();
  feature_track_length_map_.clear();
}

boost::optional<lc::Time> FeatureTracker::LatestTimestamp() const {
  if (empty()) return boost::none;
  // Since Feature Tracks without latest timestamp are erased on updates, each track contains the latest timestamp
  return feature_track_id_map_.cbegin()->second->LatestTimestamp();
}

boost::optional<lc::Time> FeatureTracker::PreviousTimestamp() const {
  const auto& longest_feature_track = LongestFeatureTrack();
  if (!longest_feature_track) return boost::none;
  // TODO(rsoussan): Need to check this before returning? If boost::none, is this cast correctly when returned here?
  return longest_feature_track->PreviousTimestamp();
}

boost::optional<localization_common::Time> FeatureTracker::OldestTimestamp() const {
  const auto& longest_feature_track = LongestFeatureTrack();
  if (!longest_feature_track) return boost::none;
  return longest_feature_track->OldestTimestamp();
}

boost::optional<const FeatureTrack&> FeatureTracker::LongestFeatureTrack() const {
  if (empty()) return boost::none;
  return *(feature_track_length_map_.rbegin()->second.get());
}
}  // namespace graph_localizer
