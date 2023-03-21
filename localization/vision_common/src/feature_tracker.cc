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

#include <vision_common/feature_tracker.h>
#include <localization_common/logger.h>

namespace vision_common {
namespace lc = localization_common;
FeatureTracker::FeatureTracker(const FeatureTrackerParams& params) : params_(params) {}

void FeatureTracker::Update(const FeaturePoints& feature_points) {
  if (feature_points.empty() && params_.remove_undetected_feature_tracks) {
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

  if (params_.remove_undetected_feature_tracks) {
    const auto feature_points_timestamp = feature_points.front().timestamp;
    RemoveUndetectedFeatureTracks(feature_points_timestamp);
  }
  const int removed_num_feature_tracks = post_add_num_feature_tracks - size();
  LogDebug("UpdateFeatureTracks: Removed feature tracks: " << removed_num_feature_tracks);
  LogDebug("UpdateFeatureTracks: Final total num feature tracks: " << size());
}

void FeatureTracker::RemoveOldPoints(const lc::Time oldest_allowed_time) {
  for (auto& feature_track : id_feature_track_map_) {
    feature_track.second.RemoveOldValues(oldest_allowed_time);
  }
  RemoveEmptyFeatureTracks();
}

const IdFeatureTrackMap& FeatureTracker::feature_tracks() const { return id_feature_track_map_; }

std::vector<std::reference_wrapper<const FeatureTrack>> FeatureTracker::FeatureTracksLengthOrdered() const {
  std::map<int, int> length_id_map;
  for (const auto& feature_track : feature_tracks()) {
    length_id_map.insert({feature_track.second.size(), feature_track.first});
  }
  std::vector<std::reference_wrapper<const FeatureTrack>> feature_tracks_length_ordered;
  // Add tracks in order from longest to shortest
  for (auto it = length_id_map.crbegin(); it != length_id_map.crend(); ++it) {
    feature_tracks_length_ordered.emplace_back((this->feature_tracks().at(it->second)));
  }

  return feature_tracks_length_ordered;
}

size_t FeatureTracker::size() const { return id_feature_track_map_.size(); }

bool FeatureTracker::empty() const { return id_feature_track_map_.empty(); }

void FeatureTracker::Clear() { id_feature_track_map_.clear(); }

void FeatureTracker::AddOrUpdateTrack(const FeaturePoint& feature_point) {
  if (id_feature_track_map_.count(feature_point.feature_track_id) == 0) {
    id_feature_track_map_[feature_point.feature_track_id] = FeatureTrack(feature_point.feature_track_id);
  }
  id_feature_track_map_[feature_point.feature_track_id].Add(feature_point.timestamp, feature_point);
}

void FeatureTracker::RemoveUndetectedFeatureTracks(const lc::Time& feature_point_timestamp) {
  for (auto feature_it = id_feature_track_map_.cbegin(); feature_it != id_feature_track_map_.cend();) {
    if (!feature_it->second.Contains(feature_point_timestamp)) {
      feature_it = id_feature_track_map_.erase(feature_it);
    } else {
      ++feature_it;
    }
  }
}

void FeatureTracker::RemoveEmptyFeatureTracks() {
  for (auto feature_it = id_feature_track_map_.cbegin(); feature_it != id_feature_track_map_.cend();) {
    if (feature_it->second.empty()) {
      feature_it = id_feature_track_map_.erase(feature_it);
    } else {
      ++feature_it;
    }
  }
}
}  // namespace vision_common
