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

#ifndef LOCALIZATION_MEASUREMENTS_MATCHED_PROJECTION_H_
#define LOCALIZATION_MEASUREMENTS_MATCHED_PROJECTION_H_

#include <localization_common/time.h>
#include <localization_measurements/image_point.h>
#include <localization_measurements/map_point.h>

#include <vector>

namespace localization_measurements {
struct MatchedProjection {
  MatchedProjection(const ImagePoint& image_point, const MapPoint& map_point, const localization_common::Time timestamp)
      : image_point(image_point), map_point(map_point), timestamp(timestamp) {}

  MatchedProjection(const ImagePoint& image_point, const gtsam::Point2& bounding_box, const MapPoint& map_point, 
      const localization_common::Time timestamp)
      : image_point(image_point), bounding_box(bounding_box), map_point(map_point), timestamp(timestamp) {}
  ImagePoint image_point;
  gtsam::Point2 bounding_box;
  MapPoint map_point;
  localization_common::Time timestamp;
};

using MatchedProjections = std::vector<MatchedProjection>;
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_MATCHED_PROJECTION_H_
