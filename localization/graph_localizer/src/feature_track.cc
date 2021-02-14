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
void FeatureTrack::AddMeasurement(const lc::Time timestamp, const gtsam::Point2& measurement) {
  points.emplace(timestamp, measurement);
}
void FeatureTrack::RemoveOldMeasurements(const lc::Time oldest_allowed_timestamp) {
  points.erase(points.begin(), points.lower_bound(oldest_allowed_timestamp));
}
bool FeatureTrack::HasMeasurement(const lc::Time timestamp) { return (points.count(timestamp) > 0); }
}  // namespace graph_localizer
