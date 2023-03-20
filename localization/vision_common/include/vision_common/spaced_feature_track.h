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
// Feature track with additional methods to return spaced feature tracks.
// Useful for appliciation that need to downsample the feature track in different ways.
// Allows for maximally spaced tracks or downsampled tracks given a set spacing.
class SpacedFeatureTrack : public FeatureTrack {
 public:
  // Default constructor for serialization only.
  SpacedFeatureTrack() = default;
  virtual ~SpacedFeatureTrack() = default;
  std::vector<FeaturePoint> AllowedPoints(const std::set<localization_common::Time>& allowed_timestamps) const;
  std::vector<FeaturePoint> LatestPointsInWindow(const double duration) const;
  std::vector<FeaturePoint> LatestPoints(const int spacing = 0) const;
  bool SpacingFits(const int spacing, const int max_num_points) const;
  int MaxSpacing(const int max_num_points) const;
  int ClosestSpacing(const int ideal_spacing, const int ideal_max_num_points) const;
  boost::optional<localization_common::Time> PreviousTimestamp() const;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(FeatureTrack);
  }
};
}  // namespace vision_common

#endif  // VISION_COMMON_SPACED_FEATURE_TRACK_H_
