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
#ifndef DEPTH_ODOMETRY_DEPTH_MATCHES_H_
#define DEPTH_ODOMETRY_DEPTH_MATCHES_H_

#include <depth_odometry/feature_match.h>
#include <ff_common/eigen_vectors.h>
#include <localization_common/time.h>

namespace depth_odometry {
struct DepthMatches {
  DepthMatches(const std::vector<Eigen::Vector2d>& source_image_points,
               const std::vector<Eigen::Vector2d>& target_image_points,
               const std::vector<Eigen::Vector3d>& source_3d_points,
               const std::vector<Eigen::Vector3d>& target_3d_points, const localization_common::Time source_time,
               const localization_common::Time target_time)
      : source_image_points(source_image_points),
        target_image_points(target_image_points),
        source_3d_points(source_3d_points),
        target_3d_points(target_3d_points),
        source_time(source_time),
        target_time(target_time) {}
  std::vector<Eigen::Vector2d> source_image_points;
  std::vector<Eigen::Vector2d> target_image_points;
  std::vector<Eigen::Vector3d> source_3d_points;
  std::vector<Eigen::Vector3d> target_3d_points;
  localization_common::Time source_time;
  localization_common::Time target_time;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_MATCHES_H_
