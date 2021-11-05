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
#ifndef LOCALIZATION_COMMON_IMAGE_CORRESPONDENCES_H_
#define LOCALIZATION_COMMON_IMAGE_CORRESPONDENCES_H_

#include <ff_common/eigen_vectors.h>

#include <vector>

namespace localization_common {
struct ImageCorrespondences {
  ImageCorrespondences(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& points_3d)
      : image_points(image_points), points_3d(points_3d) {}
  ImageCorrespondences() {}

  void AddCorrespondence(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d) {
    image_points.emplace_back(image_point);
    points_3d.emplace_back(point_3d);
  }

  std::vector<Eigen::Vector2d> image_points;
  std::vector<Eigen::Vector3d> points_3d;
};
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_IMAGE_CORRESPONDENCES_H_
