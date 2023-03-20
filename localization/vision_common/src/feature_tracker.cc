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

void FeatureTracker::UpdateFeatureTracks(const FeaturePoints& feature_points) {
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

  if (params_.remove_undetected_features) {
    const auto feature_points_timestamp = feature_points.front().timestamp;
    RemoveUndetectedFeatures(feature_points_timestamp);
  }
  const int removed_num_feature_tracks = post_add_num_feature_tracks - size();
  LogDebug("UpdateFeatureTracks: Removed feature tracks: " << removed_num_feature_tracks);
  LogDebug("UpdateFeatureTracks: Final total num feature tracks: " << size());
}

void FeatureTracker::RemoveOldPoints(const lc::Time oldest_allowed_time) {
  for (auto feature_track : feature_track_id_map_) {
    feature_track.second->RemoveOldValues(*oldest_allowed_time);
  }
}

void FeatureTracker::RemoveUndetectedFeatures(const lc::Time& feature_point_timestamp) {
  for (auto feature_it = feature_track_id_map_.cbegin(); feature_it != feature_track_id_map_.cend();) {
    if (!feature_it->second->Contains(feature_point_timestamp)) {
      feature_it = feature_track_id_map_.erase(feature_it);
    } else {
      ++feature_it;
    }
  }
}

void FeatureTracker::AddOrUpdateTrack(const FeaturePoint& feature_point) {
  if (feature_track_id_map_.count(feature_point.feature_id) == 0) {
    feature_track_id_map_[feature_point.feature_id] = std::make_shared<FeatureTrack>(feature_point.feature_id);
  }
  feature_track_id_map_[feature_point.feature_id]->Add(feature_point.timestamp, feature_point);
}

const FeatureTrackIdMap& FeatureTracker::feature_tracks() const { return feature_track_id_map_; }

size_t FeatureTracker::size() const { return feature_track_id_map_.size(); }

bool FeatureTracker::empty() const { return feature_track_id_map_.empty(); }

void FeatureTracker::Clear() { feature_track_id_map_.clear(); }
}  // namespace vision_common
