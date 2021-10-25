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
#ifndef OPTIMIZATION_COMMON_FOV_DISTORTION_H_
#define OPTIMIZATION_COMMON_FOV_DISTORTION_H_

#include <optimization_common/utilities.h>

#include <Eigen/Core>

#include <ceres/ceres.h>

namespace optimization_common {
class FovDistortion {
 public:
  template <typename T>
  Eigen::Matrix<T, 2, 1> Distort(const T* distortion, const T* intrinsics,
                                 const Eigen::Matrix<T, 2, 1>& undistorted_point) const {
    // Distortion model expects image coordinates to be in relative coordinates
    const Eigen::Matrix<T, 2, 1> relative_coordinates = RelativeCoordinates(undistorted_point, intrinsics);
    const T& relative_x = relative_coordinates[0];
    const T& relative_y = relative_coordinates[1];

    // Squared norm
    const T r2 = relative_x * relative_x + relative_y * relative_y;
    const T& w = distortion[0];
    T warping;
    if (r2 > 1e-5) {
      const T a = 2.0 * ceres::tan(w / 2.0);
      const T b = ceres::atan(r2 * a) / w;
      warping = b / r2;
    } else {
      warping = T(1.0);
    }
    const T distorted_relative_x = warping * relative_x;
    const T distorted_relative_y = warping * relative_y;
    return AbsoluteCoordinates(Eigen::Matrix<T, 2, 1>(distorted_relative_x, distorted_relative_y), intrinsics);
  }

  static constexpr int NUM_PARAMS = 1;
};
}  // namespace optimization_common

#endif  // OPTIMIZATION_COMMON_FOV_DISTORTION_H_
