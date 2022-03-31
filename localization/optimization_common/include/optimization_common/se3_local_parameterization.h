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
#ifndef OPTIMIZATION_COMMON_SE3_LOCAL_PARAMETERIZATION_H_
#define OPTIMIZATION_COMMON_SE3_LOCAL_PARAMETERIZATION_H_

#include <optimization_common/utilities.h>

#include <ceres/autodiff_local_parameterization.h>

#include <Eigen/Geometry>

namespace optimization_common {
struct SE3Plus {
  template <typename T>
  bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
    const Eigen::Transform<T, 3, Eigen::Isometry> pose = Isometry3(x);
    const Eigen::Transform<T, 3, Eigen::Isometry> pose_delta = Isometry3(delta);
    const Eigen::Transform<T, 3, Eigen::Isometry> updated_pose = pose * pose_delta;
    const Eigen::Matrix<T, 6, 1> updated_pose_vector = VectorFromIsometry3(updated_pose);
    for (int i = 0; i < 6; ++i) {
      x_plus_delta[i] = updated_pose_vector[i];
    }
    return true;
  }
};

using SE3LocalParameterization = ceres::AutoDiffLocalParameterization<SE3Plus, 6, 6>;
}  // namespace optimization_common

#endif  // OPTIMIZATION_COMMON_SE3_LOCAL_PARAMETERIZATION_H_
