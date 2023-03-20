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

#ifndef VISION_COMMON_SPACED_FEATURE_TRACK_H_
#define VISION_COMMON_SPACED_FEATURE_TRACK_H_

#include <vision_common/feature_track.h>

#include <set>
#include <vector>

namespace vision_common {
// Feature track with additional methods to return downsampled or
// set duration feature tracks.
// Allows for maximally spaced tracks, downsampled tracks given a set spacing,
// and downsampled tracks using only allowabled timestamps.
class SpacedFeatureTrack : public FeatureTrack {
 public:
  explicit SpacedFeatureTrack(const FeatureId id);
  // Default constructor for serialization only.
  SpacedFeatureTrack() = default;

  virtual ~SpacedFeatureTrack() = default;

  // Returns the latest set of points spaced by the provided spacing.
  // Starts sampling with the latest point.
  // Return vector is ordered from oldest to latest points.
  std::vector<FeaturePoint> LatestSpacedPoints(const int spacing = 0) const;

  // Returns the max spacing usable for a feature track
  // such that the total number of points in the feature
  // track does not exceed max_num_points.
  int MaxSpacing(const int max_num_points) const;

  // Returns the second latest point's timestamp in the feature track.
  boost::optional<localization_common::Time> SecondLatestTimestamp() const;

 private:
  bool SpacingFits(const int spacing, const int max_num_points) const;
  int ClosestSpacing(const int ideal_spacing, const int ideal_max_num_points) const;

  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(FeatureTrack);
  }
};
}  // namespace vision_common

#endif  // VISION_COMMON_SPACED_FEATURE_TRACK_H_
