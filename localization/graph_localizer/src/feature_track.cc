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

#include <graph_localizer/feature_track.h>
#include <localization_common/logger.h>

namespace graph_localizer {
namespace lc = localization_common;
namespace lm = localization_measurements;
FeatureTrack::FeatureTrack(const localization_measurements::FeatureId id) : id_(id) {}

void FeatureTrack::AddMeasurement(const lc::Time timestamp, const lm::FeaturePoint& feature_point) {
  points_.emplace(timestamp, feature_point);
}

void FeatureTrack::RemoveOldMeasurements(const lc::Time oldest_allowed_timestamp) {
  points_.erase(points_.begin(), points_.lower_bound(oldest_allowed_timestamp));
}

bool FeatureTrack::HasMeasurement(const lc::Time timestamp) { return (points_.count(timestamp) > 0); }

const FeatureTrack::Points& FeatureTrack::points() const { return points_; }

const localization_measurements::FeatureId& FeatureTrack::id() const { return id_; }

size_t FeatureTrack::size() const { return points_.size(); }

bool FeatureTrack::empty() const { return points_.empty(); }

std::vector<lm::FeaturePoint> FeatureTrack::LatestPoints(const int spacing) const {
  std::vector<lm::FeaturePoint> latest_points;
  int i = 0;
  for (auto point_it = points_.rbegin(); point_it != points_.rend(); ++point_it) {
    if (i++ % (spacing + 1) != 0) continue;
    latest_points.push_back(point_it->second);
  }
  return latest_points;
}

boost::optional<lc::Time> FeatureTrack::PreviousTimestamp() const {
  if (size() < 2) return boost::none;
  return std::next(points_.crbegin())->first;
}

boost::optional<lc::Time> FeatureTrack::LatestTimestamp() const {
  if (empty()) return boost::none;
  return points_.crbegin()->first;
}

boost::optional<lc::Time> FeatureTrack::OldestTimestamp() const {
  if (empty()) return boost::none;
  return points_.cbegin()->first;
}
}  // namespace graph_localizer
