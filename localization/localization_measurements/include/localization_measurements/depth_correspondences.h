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
#ifndef LOCALIZATION_MEASUREMENTS_DEPTH_CORRESPONDENCES_H_
#define LOCALIZATION_MEASUREMENTS_DEPTH_CORRESPONDENCES_H_

#include <ff_common/eigen_vectors.h>

#include <vector>

namespace localization_measurements {
struct DepthCorrespondences {
  DepthCorrespondences(const std::vector<Eigen::Vector2d>& source_image_points,
                       const std::vector<Eigen::Vector2d>& target_image_points,
                       const std::vector<Eigen::Vector3d>& source_3d_points,
                       const std::vector<Eigen::Vector3d>& target_3d_points)
      : source_image_points(source_image_points),
        target_image_points(target_image_points),
        source_3d_points(source_3d_points),
        target_3d_points(target_3d_points),
        valid_image_points(true),
        valid_3d_points(true) {}

  DepthCorrespondences(const std::vector<Eigen::Vector3d>& source_3d_points,
                       const std::vector<Eigen::Vector3d>& target_3d_points)
      : source_3d_points(source_3d_points),
        target_3d_points(target_3d_points),
        valid_image_points(false),
        valid_3d_points(true) {}

  bool valid_image_points;
  bool valid_3d_points;
  std::vector<Eigen::Vector2d> source_image_points;
  std::vector<Eigen::Vector2d> target_image_points;
  std::vector<Eigen::Vector3d> source_3d_points;
  std::vector<Eigen::Vector3d> target_3d_points;
};
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_DEPTH_CORRESPONDENCES_H_
