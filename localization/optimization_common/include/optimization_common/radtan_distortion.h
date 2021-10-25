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
#ifndef OPTIMIZATION_COMMON_RADTAN_DISTORTION_H_
#define OPTIMIZATION_COMMON_RADTAN_DISTORTION_H_

#include <optimization_common/utilities.h>

#include <Eigen/Core>

#include <ceres/ceres.h>

namespace optimization_common {
class RadTanDistortion {
 public:
  template <typename T>
  Eigen::Matrix<T, 2, 1> Distort(const T* distortion, const T* intrinsics,
                                 const Eigen::Matrix<T, 2, 1>& undistorted_point) const {
    const T& k1 = distortion[0];
    const T& k2 = distortion[1];
    const T& p1 = distortion[2];
    const T& p2 = distortion[3];
    // TODO(rsoussan): Support 5 distortion params
    const T k3(0.0);

    const Eigen::Matrix<T, 2, 1> relative_coordinates = RelativeCoordinates(undistorted_point, intrinsics);
    const T& relative_x = relative_coordinates[0];
    const T& relative_y = relative_coordinates[1];
    // Squared norm
    const T r2 = relative_x * relative_x + relative_y * relative_y;

    // Apply radial distortion
    const T radial_distortion_coeff = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    T distorted_relative_x = relative_x * radial_distortion_coeff;
    T distorted_relative_y = relative_y * radial_distortion_coeff;

    // Apply tangential distortion
    distorted_relative_x =
      distorted_relative_x + (2.0 * p1 * relative_x * relative_y + p2 * (r2 + 2.0 * relative_x * relative_x));
    distorted_relative_y =
      distorted_relative_y + (p1 * (r2 + 2.0 * relative_y * relative_y) + 2.0 * p2 * relative_x * relative_y);

    return AbsoluteCoordinates(Eigen::Matrix<T, 2, 1>(distorted_relative_x, distorted_relative_y), intrinsics);
  }

  static constexpr int NUM_PARAMS = 1;
};
}  // namespace optimization_common

#endif  // OPTIMIZATION_COMMON_RADTAN_DISTORTION_H_
