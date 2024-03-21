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

#include <vision_common/spaced_feature_track.h>
#include <localization_common/logger.h>

namespace vision_common {
namespace lc = localization_common;
SpacedFeatureTrack::SpacedFeatureTrack(const FeatureId id) : FeatureTrack(id) {}

std::vector<FeaturePoint> SpacedFeatureTrack::LatestSpacedPoints(const int spacing) const {
  std::vector<FeaturePoint> latest_points;
  int i = 0;
  // Start with latest points
  for (auto point_it = set().rbegin(); point_it != set().rend(); ++point_it) {
    if (i++ % (spacing + 1) != 0) continue;
    latest_points.push_back(point_it->second);
  }
  // Flip so oldest points are first
  std::reverse(latest_points.begin(), latest_points.end());
  return latest_points;
}

bool SpacedFeatureTrack::SpacingFits(const int spacing, const int max_num_points) const {
  // Since we include the latest point, the points included for spacing and max_num_points
  // is 1 for the latest point plus a point at intervals of spacing + 1 for max_num_points - 1 (excludes latest point).
  // 1 0 0 1 0 0 1 0 0 -> here a 1 indicates a point used, the first 1 is the latest point, and the spacing is 2.
  // In this case if max_num_points <= 3 this suceeds and otherwise this fails as fewer than 3 points would be included
  // with the desired spacing.
  return ((spacing + 1) * (max_num_points - 1) + 1) <= static_cast<int>(size());
}

int SpacedFeatureTrack::MaxSpacing(const int max_num_points) const {
  // Avoid divide by zero and other corner cases
  if (max_num_points <= 1 || static_cast<int>(size()) <= 0) return 0;
  // Derived using equation from SpacingFits
  // (spacing + 1 ) * (max_num_points - 1) + 1 = size
  // -> spacing = (size - max_num_points)/(max_num_points - 1)
  return std::max(0, (static_cast<int>(size()) - max_num_points) / (max_num_points - 1));
}

int SpacedFeatureTrack::ClosestSpacing(const int ideal_spacing, const int ideal_max_num_points) const {
  // Check Ideal Case
  if (SpacingFits(ideal_spacing, ideal_max_num_points)) return ideal_spacing;
  // Check too few points case
  if (static_cast<int>(size()) <= ideal_max_num_points) return 0;
  // Derive new optimal spacing for ideal_max_num_points
  for (int spacing = ideal_spacing - 1; spacing >= 0; --spacing) {
    if (SpacingFits(spacing, ideal_max_num_points)) return spacing;
  }
  return 0;
}

boost::optional<lc::Time> SpacedFeatureTrack::SecondLatestTimestamp() const {
  if (size() < 2) return boost::none;
  return std::next(set().crbegin())->first;
}
}  // namespace vision_common
