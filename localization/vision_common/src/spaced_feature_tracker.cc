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

#include <vision_common/spaced_feature_tracker.h>
#include <localization_common/logger.h>

namespace vision_common {
namespace lc = localization_common;
SpacedFeatureTracker::SpacedFeatureTracker(const SpacedFeatureTrackerParams& params)
    : Base(params), params_(params), measurement_count_(0) {}

void SpacedFeatureTracker::Update(const FeaturePoints& feature_points) {
  Base::Update(feature_points);
  // Space out measurements by provided spacing
  if (!feature_points.empty() && ((measurement_count_++ % (params_.measurement_spacing + 1)) == 0))
    allowed_timestamps_.emplace(feature_points.front().timestamp);
}

void SpacedFeatureTracker::RemoveOldPoints(const localization_common::Time oldest_allowed_time) {
  Base::RemoveOldPoints(oldest_allowed_time);
  // Remove allowed measurements older than oldest allowed time
  allowed_timestamps_.erase(allowed_timestamps_.begin(), allowed_timestamps_.lower_bound(oldest_allowed_time));
}

void SpacedFeatureTracker::Clear() {
  Base::Clear();
  allowed_timestamps_.clear();
}

std::vector<FeaturePoints> SpacedFeatureTracker::SpacedFeatureTracks() const {
  std::vector<FeaturePoints> spaced_feature_tracks;
  for (const auto& feature_track : feature_tracks()) {
    const auto downsampled_track = feature_track.second.DownsampledValues(allowed_timestamps_);
    if (downsampled_track.empty()) continue;
    FeaturePoints points;
    for (const auto& point : downsampled_track) {
      points.emplace_back(point.value);
    }
    spaced_feature_tracks.emplace_back(points);
  }

  return spaced_feature_tracks;
}
}  // namespace vision_common
