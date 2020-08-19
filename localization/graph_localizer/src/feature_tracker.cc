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

#include <glog/logging.h>

namespace graph_localizer {
namespace lm = localization_measurements;
void FeatureTracker::UpdateFeatureTracks(const lm::FeaturePoints &feature_points) {
  const int starting_num_feature_tracks = feature_tracks_.size();
  DLOG(INFO) << "UpdateFeatureTracks: Starting num feature tracks: " << starting_num_feature_tracks;
  // Update existing features or add new one
  for (const auto &feature_point : feature_points) {
    feature_tracks_[feature_point.feature_id].latest_image_id = feature_point.image_id;
    feature_tracks_[feature_point.feature_id].points.emplace_back(feature_point);
  }

  const int post_add_num_feature_tracks = feature_tracks_.size();
  DLOG(INFO) << "UpdateFeatureTracks: Added feature tracks: "
             << post_add_num_feature_tracks - starting_num_feature_tracks;

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
  DLOG(INFO) << "UpdateFeatureTracks: Removed feature tracks: " << removed_num_feature_tracks;
  DLOG(INFO) << "UpdateFeatureTracks: Final total num feature tracks: " << feature_tracks_.size();
}

void FeatureTracker::RemoveOldFeaturePoints(const lm::Time oldest_allowed_time) {
  for (auto &feature : feature_tracks_) {
    auto point_it = feature.second.points.cbegin();
    while (point_it != feature.second.points.cend() && point_it->timestamp < oldest_allowed_time) {
      ++point_it;
    }
    feature.second.points.erase(feature.second.points.begin(), point_it);
  }
}
}  // namespace graph_localizer
