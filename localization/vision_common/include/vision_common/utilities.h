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
#ifndef VISION_COMMON_UTILITIES_H_
#define VISION_COMMON_UTILITIES_H_

#include <Eigen/Core>

namespace vision_common {
template <typename T>
Eigen::Matrix<T, 2, 1> RelativeCoordinates(const Eigen::Matrix<T, 2, 1>& absolute_point,
                                           const Eigen::Matrix<T, 3, 3>& intrinsics);
template <typename T>
Eigen::Matrix<T, 2, 1> AbsoluteCoordinates(const Eigen::Matrix<T, 2, 1>& relative_point,
                                           const Eigen::Matrix<T, 3, 3>& intrinsics);

// Implementation
template <typename T>
Eigen::Matrix<T, 2, 1> RelativeCoordinates(const Eigen::Matrix<T, 2, 1>& absolute_point,
                                           const Eigen::Matrix<T, 3, 3>& intrinsics) {
  const T& f_x = intrinsics(0, 0);
  const T& f_y = intrinsics(1, 1);
  const T& p_x = intrinsics(0, 2);
  const T& p_y = intrinsics(1, 2);

  const T& x = absolute_point[0];
  const T& y = absolute_point[1];
  const T relative_x = (x - p_x) / f_x;
  const T relative_y = (y - p_y) / f_y;
  return Eigen::Matrix<T, 2, 1>(relative_x, relative_y);
}

template <typename T>
Eigen::Matrix<T, 2, 1> AbsoluteCoordinates(const Eigen::Matrix<T, 2, 1>& relative_point,
                                           const Eigen::Matrix<T, 3, 3>& intrinsics) {
  const T& f_x = intrinsics(0, 0);
  const T& f_y = intrinsics(1, 1);
  const T& p_x = intrinsics(0, 2);
  const T& p_y = intrinsics(1, 2);
  return Eigen::Matrix<T, 2, 1>(relative_point[0] * f_x + p_x, relative_point[1] * f_y + p_y);
}
}  // namespace vision_common

#endif  // VISION_COMMON_UTILITIES_H_
