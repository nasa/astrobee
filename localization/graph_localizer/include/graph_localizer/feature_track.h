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

#include <map>
#include <vector>

namespace graph_localizer {
class FeatureTrack {
 public:
  using Points = std::map<localization_common::Time, localization_measurements::FeaturePoint>;
  explicit FeatureTrack(const localization_measurements::FeatureId id);
  void AddMeasurement(const localization_common::Time timestamp,
                      const localization_measurements::FeaturePoint& feature_point);
  void RemoveOldMeasurements(const localization_common::Time oldest_allowed_timestamp);
  bool HasMeasurement(const localization_common::Time timestamp);
  const Points& points() const;
  const localization_measurements::FeatureId& id() const;
  size_t size() const;
  bool empty() const;
  std::vector<localization_measurements::FeaturePoint> LatestPoints(const int spacing = 0) const;
  boost::optional<localization_common::Time> PreviousTimestamp() const;
  boost::optional<localization_common::Time> LatestTimestamp() const;
  boost::optional<localization_common::Time> OldestTimestamp() const;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(id_);
    ar& BOOST_SERIALIZATION_NVP(points_);
  }

  localization_measurements::FeatureId id_;
  Points points_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_FEATURE_TRACK_H_
