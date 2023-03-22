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

#ifndef VISION_COMMON_SPACED_FEATURE_TRACKER_H_
#define VISION_COMMON_SPACED_FEATURE_TRACKER_H_

#include <localization_common/time.h>
#include <localization_common/timestamped_set.h>
#include <vision_common/feature_tracker.h>
#include <vision_common/spaced_feature_track.h>
#include <vision_common/spaced_feature_tracker_params.h>

#include <set>
#include <vector>

namespace vision_common {
class SpacedFeatureTracker : public FeatureTracker<SpacedFeatureTrack> {
  using Base = FeatureTracker<SpacedFeatureTrack>;

 public:
  explicit SpacedFeatureTracker(const SpacedFeatureTrackerParams& params);

  // Default constructor only for serialization
  SpacedFeatureTracker() = default;

  // Updates FeatureTracker and updates allowed timestamps.
  void Update(const FeaturePoints& feature_points) final;

  // Removes old points from FeatureTracker and allowed timestamps.
  void RemoveOldPoints(const localization_common::Time oldest_allowed_time) final;

  // Clears FeatureTracker and allowed timestamps.
  void Clear() final;

  // Returns feature tracks spaced using allowed timestamps.
  // Skips feature tracks with no allowed timestamps.
  std::vector<FeaturePoints> SpacedFeatureTracks() const;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(allowed_timestamps_);
    ar& BOOST_SERIALIZATION_NVP(params_);
  }

  int measurement_count_;
  std::set<localization_common::Time> allowed_timestamps_;
  SpacedFeatureTrackerParams params_;
};
}  // namespace vision_common

#endif  // VISION_COMMON_SPACED_FEATURE_TRACKER_H_
