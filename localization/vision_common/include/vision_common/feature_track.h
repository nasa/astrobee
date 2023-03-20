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

#ifndef VISION_COMMON_FEATURE_TRACK_H_
#define VISION_COMMON_FEATURE_TRACK_H_

#include <localization_common/timestamped_set.h>
#include <vision_common/feature_point.h>

namespace vision_common {
// A Feature track consists of a set of timestamped feature point detections and
// an id.
class FeatureTrack : public localization_common::TimestampedSet<FeaturePoint> {
 public:
  explicit FeatureTrack(const FeatureId id) : id_(id) {}
  // Default constructor for serialization only.
  FeatureTrack() = default;
  virtual ~FeatureTrack() = default;
  const FeatureId& id() const { return id_; }

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(id_);
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(localization_common::TimestampedSet<FeaturePoint>);
  }

  FeatureId id_;
};
}  // namespace vision_common

#endif  // VISION_COMMON_FEATURE_TRACK_H_
