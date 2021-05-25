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

#ifndef LOCALIZATION_COMMON_FEATURE_POINT3D_H_
#define LOCALIZATION_COMMON_FEATURE_POINT3D_H_

#include <gtsam/geometry/Point3.h>

#include <vector>

namespace localization_common {
using FeatureId = int;

struct FeaturePoint3dNoise {
  gtsam::SharedNoiseModel noise;
};

struct FeaturePoint3d {
  FeaturePoint3d(const gtsam::Point3& global_t_point, const FeatureId feature_id)
      : global_t_point(global_t_point), feature_id(feature_id) {}
  FeaturePoint3d() {}
  gtsam::Point3 global_t_point;
  FeatureId feature_id;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(feature_id);
    ar& BOOST_SERIALIZATION_NVP(global_t_point);
  }
};

using FeaturePoint3ds = std::vector<FeaturePoint3d>;
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_FEATURE_POINT3D_H_
