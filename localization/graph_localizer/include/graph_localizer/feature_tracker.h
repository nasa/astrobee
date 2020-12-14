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

#ifndef GRAPH_LOCALIZER_FEATURE_TRACKER_H_
#define GRAPH_LOCALIZER_FEATURE_TRACKER_H_

#include <graph_localizer/feature_track.h>
#include <graph_localizer/feature_tracker_params.h>
#include <localization_common/time.h>
#include <localization_measurements/feature_point.h>

#include <gtsam/geometry/Point2.h>

#include <deque>
#include <map>

namespace graph_localizer {
using FeatureTrackMap = std::map<localization_measurements::FeatureId, FeatureTrack>;
class FeatureTracker {
 public:
  explicit FeatureTracker(const FeatureTrackerParams& params = FeatureTrackerParams());
  // Update existing tracks and add new tracks.  Remove tracks without
  // detections.
  void UpdateFeatureTracks(const localization_measurements::FeaturePoints& feature_points);
  const FeatureTrackMap& feature_tracks() const { return feature_tracks_; }
  void RemoveOldFeaturePoints(localization_common::Time oldest_allowed_time);
  boost::optional<localization_common::Time> latest_timestamp() const;
  boost::optional<localization_common::Time> PreviousTimestamp() const;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(feature_tracks_);
  }

  FeatureTrackMap feature_tracks_;
  FeatureTrackerParams params_;
  boost::optional<localization_common::Time> latest_time_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_FEATURE_TRACKER_H_
