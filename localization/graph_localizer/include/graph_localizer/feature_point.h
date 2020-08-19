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

#ifndef GRAPH_LOCALIZER_FEATURE_POINT_H_
#define GRAPH_LOCALIZER_FEATURE_POINT_H_

#include <graph_localizer/time.h>

#include <gtsam/geometry/Point2.h>

#include <vector>

namespace graph_localizer {
using FeatureId = int;
using ImageId = int;

struct FeaturePoint {
  FeaturePoint(const double u, const double v, const ImageId image_id, const FeatureId feature_id, const Time timestamp)
      : image_point(u, v), image_id(image_id), feature_id(feature_id), timestamp(timestamp) {}
  gtsam::Point2 image_point;
  ImageId image_id;
  FeatureId feature_id;
  Time timestamp;
};

using FeaturePoints = std::vector<FeaturePoint>;
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_FEATURE_POINT_H_
