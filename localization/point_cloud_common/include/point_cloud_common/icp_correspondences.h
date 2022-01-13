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
#ifndef POINT_CLOUD_COMMON_ICP_CORRESPONDENCES_H_
#define POINT_CLOUD_COMMON_ICP_CORRESPONDENCES_H_

#include <ff_common/eigen_vectors.h>

#include <vector>

namespace point_cloud_common {
struct ICPCorrespondences {
  ICPCorrespondences(const std::vector<Eigen::Vector3d>& source_points,
                     const std::vector<Eigen::Vector3d>& target_points,
                     const std::vector<Eigen::Vector3d>& target_normals)
      : source_points(source_points), target_points(target_points), target_normals(target_normals) {}
  size_t size() const { return source_points.size(); }

  std::vector<Eigen::Vector3d> source_points;
  std::vector<Eigen::Vector3d> target_points;
  std::vector<Eigen::Vector3d> target_normals;
};
}  // namespace point_cloud_common

#endif  // POINT_CLOUD_COMMON_ICP_CORRESPONDENCES_H_
