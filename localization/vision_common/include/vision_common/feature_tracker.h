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

#ifndef VISION_COMMON_FEATURE_TRACKER_H_
#define VISION_COMMON_FEATURE_TRACKER_H_

#include <localization_common/time.h>
#include <vision_common/feature_point.h>
#include <vision_common/feature_track.h>
#include <vision_common/feature_tracker_params.h>

#include <map>
#include <vector>

namespace vision_common {
template <typename FeatureTrackType = FeatureTrack>
class FeatureTracker {
 public:
  using IdFeatureTrackMap = std::map<FeatureId, FeatureTrackType>;
  explicit FeatureTracker(const FeatureTrackerParams& params);

  // Default constructor only for serialization
  FeatureTracker() = default;

  virtual ~FeatureTracker() = default;

  // Add new feature points to existing or new tracks.  Optionally removes
  // any existing tracks that weren't detected in passed feature_points.
  virtual void Update(const FeaturePoints& feature_points);

  // Remove any points older than oldest_allowed_time from each feature track.
  // Removes any feature tracks that subsequently have no more detections.
  virtual void RemoveOldPoints(const localization_common::Time oldest_allowed_time);

  // Returns a reference to the feature tracks.
  const IdFeatureTrackMap& feature_tracks() const;

  // Returns feature track references ordered from longest to shortest
  std::vector<std::reference_wrapper<const FeatureTrackType>> FeatureTracksLengthOrdered() const;

  // Returns the number of feature tracks.
  size_t size() const;

  // Returns if no feature tracks exist.
  bool empty() const;

  // Deletes all feature tracks.
  virtual void Clear();

 private:
  // Add new feature point to an existing track with the same track id
  // create a new track if this doesn't exist.
  void AddOrUpdateTrack(const FeaturePoint& feature_point);

  // Remove any feature tracks without detections at the provided timestamp
  void RemoveUndetectedFeatureTracks(const localization_common::Time& time);

  // Remove any feature tracks with no detections.
  void RemoveEmptyFeatureTracks();

  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(id_feature_track_map_);
    ar& BOOST_SERIALIZATION_NVP(params_);
  }

  IdFeatureTrackMap id_feature_track_map_;
  FeatureTrackerParams params_;
};

// Implementation
template <typename FeatureTrackType>
FeatureTracker<FeatureTrackType>::FeatureTracker(const FeatureTrackerParams& params) : params_(params) {}

template <typename FeatureTrackType>
void FeatureTracker<FeatureTrackType>::Update(const FeaturePoints& feature_points) {
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

template <typename FeatureTrackType>
void FeatureTracker<FeatureTrackType>::RemoveOldPoints(const localization_common::Time oldest_allowed_time) {
  for (auto& feature_track : id_feature_track_map_) {
    feature_track.second.RemoveOldValues(oldest_allowed_time);
  }
  RemoveEmptyFeatureTracks();
}

template <typename FeatureTrackType>
const typename FeatureTracker<FeatureTrackType>::IdFeatureTrackMap& FeatureTracker<FeatureTrackType>::feature_tracks()
  const {
  return id_feature_track_map_;
}

template <typename FeatureTrackType>
std::vector<std::reference_wrapper<const FeatureTrackType>>
FeatureTracker<FeatureTrackType>::FeatureTracksLengthOrdered() const {
  std::map<int, int> length_id_map;
  for (const auto& feature_track : feature_tracks()) {
    length_id_map.insert({feature_track.second.size(), feature_track.first});
  }
  std::vector<std::reference_wrapper<const FeatureTrackType>> feature_tracks_length_ordered;
  // Add tracks in order from longest to shortest
  for (auto it = length_id_map.crbegin(); it != length_id_map.crend(); ++it) {
    feature_tracks_length_ordered.emplace_back((this->feature_tracks().at(it->second)));
  }

  return feature_tracks_length_ordered;
}

template <typename FeatureTrackType>
size_t FeatureTracker<FeatureTrackType>::size() const {
  return id_feature_track_map_.size();
}

template <typename FeatureTrackType>
bool FeatureTracker<FeatureTrackType>::empty() const {
  return id_feature_track_map_.empty();
}

template <typename FeatureTrackType>
void FeatureTracker<FeatureTrackType>::Clear() {
  id_feature_track_map_.clear();
}

template <typename FeatureTrackType>
void FeatureTracker<FeatureTrackType>::AddOrUpdateTrack(const FeaturePoint& feature_point) {
  if (id_feature_track_map_.count(feature_point.feature_track_id) == 0) {
    id_feature_track_map_[feature_point.feature_track_id] = FeatureTrackType(feature_point.feature_track_id);
  }
  id_feature_track_map_[feature_point.feature_track_id].Add(feature_point.timestamp, feature_point);
}

template <typename FeatureTrackType>
void FeatureTracker<FeatureTrackType>::RemoveUndetectedFeatureTracks(
  const localization_common::Time& feature_point_timestamp) {
  for (auto feature_it = id_feature_track_map_.cbegin(); feature_it != id_feature_track_map_.cend();) {
    if (!feature_it->second.Contains(feature_point_timestamp)) {
      feature_it = id_feature_track_map_.erase(feature_it);
    } else {
      ++feature_it;
    }
  }
}

template <typename FeatureTrackType>
void FeatureTracker<FeatureTrackType>::RemoveEmptyFeatureTracks() {
  for (auto feature_it = id_feature_track_map_.cbegin(); feature_it != id_feature_track_map_.cend();) {
    if (feature_it->second.empty()) {
      feature_it = id_feature_track_map_.erase(feature_it);
    } else {
      ++feature_it;
    }
  }
}
}  // namespace vision_common

#endif  // VISION_COMMON_FEATURE_TRACKER_H_
