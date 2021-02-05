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
  // Update latest time if a new time is available.
  // Feature points don't necessarily arrive in time order.
  if (!feature_points.empty()) {
    latest_time_ =
      latest_time_ ? std::max(*latest_time_, feature_points.front().timestamp) : feature_points.front().timestamp;
  }

  const int starting_num_feature_tracks = feature_tracks_.size();

  LogDebug("UpdateFeatureTracks: Starting num feature tracks: " << starting_num_feature_tracks);
  // Update existing features or add new one
  for (const auto& feature_point : feature_points) {
    feature_tracks_[feature_point.feature_id].latest_image_id = feature_point.image_id;
    feature_tracks_[feature_point.feature_id].id = feature_point.feature_id;
    feature_tracks_[feature_point.feature_id].points.emplace_back(feature_point);
  }

  const int post_add_num_feature_tracks = feature_tracks_.size();
  LogDebug("UpdateFeatureTracks: Added feature tracks: " << post_add_num_feature_tracks - starting_num_feature_tracks);

  // Remove features that weren't detected
  const auto image_id = feature_points.empty() ? 0 : feature_points.front().image_id;
  const bool remove_all_features = feature_points.empty();
  for (auto feature_it = feature_tracks_.cbegin(); feature_it != feature_tracks_.cend();) {
    if (feature_it->second.latest_image_id != image_id || remove_all_features) {
      feature_it = feature_tracks_.erase(feature_it);
    } else {
      ++feature_it;
    }
  }

  const int removed_num_feature_tracks = post_add_num_feature_tracks - feature_tracks_.size();
  LogDebug("UpdateFeatureTracks: Removed feature tracks: " << removed_num_feature_tracks);
  LogDebug("UpdateFeatureTracks: Final total num feature tracks: " << feature_tracks_.size());
}

void FeatureTracker::RemovePointsOutsideWindow() {
  if (!latest_time_) return;
  const lc::Time oldest_allowed_time = *latest_time_ - params_.sliding_window_duration;
  if (oldest_allowed_time <= 0) return;
  RemoveOldFeaturePoints(oldest_allowed_time);
}

void FeatureTracker::RemoveOldFeaturePoints(lc::Time oldest_allowed_time) {
  // Remove any timestamp before oldest_allowed_time and before start of time window
  oldest_allowed_time =
    latest_time_ ? std::max(*latest_time_ - params_.sliding_window_duration, oldest_allowed_time) : oldest_allowed_time;

  // Handle any out of order tracks that are too old. Split into two for loops
  // so ordered points can be removed in sub linear time (most measurements are ordered).
  // TODO(rsoussan): Do this more efficiently
  for (auto& feature : feature_tracks_) {
    auto point_it = feature.second.points.cbegin();
    while (point_it != feature.second.points.cend() && point_it->timestamp < oldest_allowed_time) {
      ++point_it;
    }
    feature.second.points.erase(feature.second.points.begin(), point_it);
  }

  for (auto& feature : feature_tracks_) {
    auto point_it = feature.second.points.cbegin();
    while (point_it != feature.second.points.cend()) {
      if (point_it->timestamp < oldest_allowed_time)
        point_it = feature.second.points.erase(point_it);
      else
        ++point_it;
    }
  }
}

boost::optional<lc::Time> FeatureTracker::latest_timestamp() const { return latest_time_; }

boost::optional<lc::Time> FeatureTracker::PreviousTimestamp() const {
  for (const auto& feature_track_pair : feature_tracks_) {
    const auto& points = feature_track_pair.second.points;
    if (points.size() < 2) continue;
    return points[points.size() - 2].timestamp;
  }
  return boost::none;
}

// TODO(rsoussan): Store points in sorted order to make this and PreviousTimestamp more efficient
boost::optional<localization_common::Time> FeatureTracker::OldestTimestamp() const {
  boost::optional<localization_common::Time> oldest_timestamp;
  for (const auto& feature_track_pair : feature_tracks_) {
    for (const auto& point : feature_track_pair.second.points) {
      if (!oldest_timestamp || point.timestamp < *oldest_timestamp) oldest_timestamp = point.timestamp;
    }
  }
  return oldest_timestamp;
}

}  // namespace graph_localizer
