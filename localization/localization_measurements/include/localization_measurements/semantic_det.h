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

#ifndef LOCALIZATION_MEASUREMENTS_SEMANTIC_DET_H_
#define LOCALIZATION_MEASUREMENTS_SEMANTIC_DET_H_

#include <localization_common/time.h>

#include <gtsam/geometry/Point2.h>

#include <vector>

namespace localization_measurements {
using ClassId = int;
using ImageId = int;

struct SemanticDet {
  SemanticDet(const double u, const double v, const ImageId image_id, const ClassId class_id,
              const localization_common::Time timestamp)
      : image_point(u, v), image_id(image_id), class_id(class_id), timestamp(timestamp) {}
  SemanticDet() {}
  gtsam::Point2 image_point;
  ImageId image_id;
  ClassId class_id;
  localization_common::Time timestamp;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(image_point);
    ar& BOOST_SERIALIZATION_NVP(image_id);
    ar& BOOST_SERIALIZATION_NVP(class_id);
    ar& BOOST_SERIALIZATION_NVP(timestamp);
  }
};

using SemanticDets = std::vector<SemanticDet>;
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_SEMANTIC_DET_H_
