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

#ifndef GRAPH_VIO_FEATURE_TRACKER_H_
#define GRAPH_VIO_FEATURE_TRACKER_H_

#include <graph_vio/feature_track.h>
#include <graph_vio/feature_tracker_params.h>
#include <localization_common/time.h>
#include <localization_measurements/feature_point.h>

#include <gtsam/geometry/Point2.h>

#include <deque>
#include <map>
#include <set>

namespace graph_vio {
using FeatureTrackIdMap = std::map<localization_measurements::FeatureId, std::shared_ptr<FeatureTrack>>;
using FeatureTrackLengthMap = std::multimap<int, std::shared_ptr<FeatureTrack>>;
class FeatureTracker {
 public:
  explicit FeatureTracker(const FeatureTrackerParams& params = FeatureTrackerParams());
  // Update existing tracks and add new tracks.  Remove tracks without
  // detections.
  void UpdateFeatureTracks(const localization_measurements::FeaturePoints& feature_points);
  const FeatureTrackIdMap& feature_tracks() const;
  const std::set<localization_common::Time>& smart_factor_timestamp_allow_list() const;
  const FeatureTrackLengthMap& feature_tracks_length_ordered() const;
  int NumTracksWithAtLeastNPoints(int n) const;
  void RemoveUndetectedFeatures(const localization_common::Time& feature_point);
  void RemoveOldFeaturePointsAndSlideWindow(
    boost::optional<localization_common::Time> oldest_allowed_time = boost::none);
  void AddOrUpdateTrack(const localization_measurements::FeaturePoint& feature_point);
  void UpdateLengthMap();
  void UpdateAllowList(const localization_common::Time& timestamp);
  void SlideAllowList(const localization_common::Time& oldest_allowed_time);
  boost::optional<const FeatureTrack&> LongestFeatureTrack() const;
  size_t size() const;
  bool empty() const;
  void Clear();
  boost::optional<localization_common::Time> OldestTimestamp() const;
  boost::optional<localization_common::Time> LatestTimestamp() const;
  boost::optional<localization_common::Time> PreviousTimestamp() const;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(feature_track_id_map_);
    ar& BOOST_SERIALIZATION_NVP(feature_track_length_map_);
  }

  FeatureTrackIdMap feature_track_id_map_;
  FeatureTrackLengthMap feature_track_length_map_;
  FeatureTrackerParams params_;
  // TODO(rsoussan): Move ths somewhere else?
  std::set<localization_common::Time> smart_factor_timestamp_allow_list_;
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_FEATURE_TRACKER_H_
