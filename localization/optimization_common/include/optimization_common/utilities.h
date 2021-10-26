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
#ifndef OPTIMIZATION_COMMON_UTILITIES_H_
#define OPTIMIZATION_COMMON_UTILITIES_H_

#include <Eigen/Core>

#include <ceres/ceres.h>

#include <vector>

namespace optimization_common {
Eigen::Matrix<double, 6, 1> VectorFromIsometry3d(const Eigen::Isometry3d& isometry_3d);
Eigen::Matrix<double, 7, 1> VectorFromAffine3d(const Eigen::Affine3d& affine_3d);
Eigen::Matrix3d Intrinsics(const Eigen::Vector2d& focal_lengths, const Eigen::Vector2d& principal_points);

// Assumes compact quaternion parameterization for rotations
// TODO(rsoussan): Use exponential map with local parameterization and compact axis angle parameterization
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> Isometry3(const T* isometry_data) {
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> compact_quaternion(isometry_data);
  Eigen::Matrix<T, 3, 3> rotation;
  // For a quaternion, sqrt(x^2+y^2+z^2+w^2) = 1
  // Since a compact quaternion provides the x,y,z compenents, to recover w use:
  // w^2 = 1 - (x^2 + y^2 + z^2)
  const T w_squared = 1.0 - compact_quaternion.squaredNorm();
  // Catch invalid quaternion
  if (w_squared <= 0.0) {
    rotation = Eigen::Matrix<T, 3, 3>::Identity();
  } else {
    const Eigen::Quaternion<T> quaternion(ceres::sqrt(w_squared), compact_quaternion[0], compact_quaternion[1],
                                          compact_quaternion[2]);
    rotation = Eigen::Matrix<T, 3, 3>(quaternion);
  }
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(&isometry_data[3]);
  Eigen::Transform<T, 3, Eigen::Isometry> isometry_3;
  isometry_3.linear() = rotation;
  isometry_3.translation() = translation;
  return isometry_3;
}

// Assumes compact quaternion parameterization for rotations
// TODO(rsoussan): Use exponential map with local parameterization and compact axis angle parameterization
template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> Affine3(const T* affine_data) {
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> compact_quaternion(affine_data);
  Eigen::Matrix<T, 3, 3> rotation;
  // For a quaternion, sqrt(x^2+y^2+z^2+w^2) = 1
  // Since a compact quaternion provides the x,y,z compenents, to recover w use:
  // w^2 = 1 - (x^2 + y^2 + z^2)
  const T w_squared = 1.0 - compact_quaternion.squaredNorm();
  // Catch invalid quaternion
  if (w_squared < 0.0) {
    rotation = Eigen::Matrix<T, 3, 3>::Identity();
  } else {
    const Eigen::Quaternion<T> quaternion(ceres::sqrt(w_squared), compact_quaternion[0], compact_quaternion[1],
                                          compact_quaternion[2]);
    rotation = Eigen::Matrix<T, 3, 3>(quaternion);
  }
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(&affine_data[3]);
  const T scale = affine_data[6];
  const Eigen::Matrix<T, 3, 3> scale_matrix(Eigen::Matrix<T, 3, 3>::Identity() * scale);
  Eigen::Transform<T, 3, Eigen::Affine> affine_3;
  affine_3.linear() = scale_matrix * rotation;
  affine_3.translation() = translation;
  // TODO(rsoussan): why doesnt't this work?
  // affine_3.fromPositionOrientationScale(translation, rotation, scale_matrix);
  return affine_3;
}

// Assumes storage as focal lengths followed by principal points
template <typename T>
Eigen::Matrix<T, 3, 3> Intrinsics(const T* intrinsics_data) {
  Eigen::Matrix<T, 3, 3> intrinsics(Eigen::Matrix<T, 3, 3>::Identity());
  intrinsics(0, 0) = intrinsics_data[0];
  intrinsics(1, 1) = intrinsics_data[1];
  intrinsics(0, 2) = intrinsics_data[2];
  intrinsics(1, 2) = intrinsics_data[3];
  return intrinsics;
}

template <typename T>
Eigen::Matrix<T, 3, 3> Intrinsics(const T* focal_lengths, const T* principal_points) {
  Eigen::Matrix<T, 3, 3> intrinsics(Eigen::Matrix<T, 3, 3>::Identity());
  intrinsics(0, 0) = focal_lengths[0];
  intrinsics(1, 1) = focal_lengths[1];
  intrinsics(0, 2) = principal_points[0];
  intrinsics(1, 2) = principal_points[1];
  return intrinsics;
}

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

double ResidualNorm(const std::vector<double>& residual, const int index, const int residual_size);

// Assumes each residual is the same size
void CheckResiduals(const int residual_size, ceres::Problem& problem, const double outlier_threshold = 0.99);
}  // namespace optimization_common

#endif  // OPTIMIZATION_COMMON_UTILITIES_H_
