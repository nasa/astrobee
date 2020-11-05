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

#ifndef GRAPH_LOCALIZER_FEATURE_TRACK_H_
#define GRAPH_LOCALIZER_FEATURE_TRACK_H_

#include <localization_measurements/feature_point.h>

#include <deque>
#include <vector>

namespace graph_localizer {

struct FeatureTrack {
  localization_measurements::FeatureId id;
  std::deque<localization_measurements::FeaturePoint> points;
  localization_measurements::ImageId latest_image_id;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(id);
    ar& BOOST_SERIALIZATION_NVP(latest_image_id);
    ar& BOOST_SERIALIZATION_NVP(points);
  }
};

using FeatureTracks = std::vector<FeatureTrack>;
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_FEATURE_TRACK_H_
