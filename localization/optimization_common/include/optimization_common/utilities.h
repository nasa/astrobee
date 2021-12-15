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
#include <ceres/rotation.h>

#include <vector>

namespace optimization_common {
// Assumes compact angle axis (3d vector where norm gives the angle) parameterization for rotations
// First 3 values of isometry_data are the compact angle axis, next 3 are the translation
template <typename T>
Eigen::Matrix<T, 6, 1> VectorFromIsometry3(const Eigen::Transform<T, 3, Eigen::Isometry>& isometry_3);

Eigen::Matrix<double, 6, 1> VectorFromIsometry3d(const Eigen::Isometry3d& isometry_3d);
// Assumes compact angle axis (3d vector where norm gives the angle) parameterization for rotations
// First 3 values of isometry_data are the compact angle axis, next 3 are the translation, last is scale
Eigen::Matrix<double, 7, 1> VectorFromAffine3d(const Eigen::Affine3d& affine_3d);
// Assumes compact angle axis (3d vector where norm gives the angle) parameterization for rotations
// First 3 values of isometry_data are the compact angle axis, next 3 are the translation
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> Isometry3(const T* isometry_data);
Eigen::Matrix3d Intrinsics(const Eigen::Vector2d& focal_lengths, const Eigen::Vector2d& principal_points);
// Assumes compact angle axis (3d vector where norm gives the angle) parameterization for rotations
// First 3 values of isometry_data are the compact angle axis, next 3 are the translation, last is scale
template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> Affine3(const T* affine_data);

Eigen::Isometry3d Isometry3d(const Eigen::Matrix<double, 6, 1>& isometry_vector);

Eigen::Affine3d Affine3d(const Eigen::Matrix<double, 7, 1>& affine_vector);

// Assumes storage as focal lengths followed by principal points
template <typename T>
Eigen::Matrix<T, 3, 3> Intrinsics(const T* intrinsics_data);

template <typename T>
Eigen::Matrix<T, 3, 3> Intrinsics(const T* focal_lengths, const T* principal_points);

template <typename T>
Eigen::Matrix<T, 2, 1> RelativeCoordinates(const Eigen::Matrix<T, 2, 1>& absolute_point,
                                           const Eigen::Matrix<T, 3, 3>& intrinsics);
template <typename T>
Eigen::Matrix<T, 2, 1> AbsoluteCoordinates(const Eigen::Matrix<T, 2, 1>& relative_point,
                                           const Eigen::Matrix<T, 3, 3>& intrinsics);

void AddParameterBlock(const int num_parameters, double* const parameters, ceres::Problem& problem,
                       const bool set_constant = false);

void AddConstantParameterBlock(const int num_parameters, double* const parameters, ceres::Problem& problem);

void AddConstantParameterBlock(const int num_parameters, double const* const parameters, ceres::Problem& problem);

double ResidualNorm(const std::vector<double>& residual, const int index, const int residual_size);

// Assumes each residual is the same size
void CheckResiduals(const int residual_size, ceres::Problem& problem, const double outlier_threshold = 0.99);

template <typename T>
Eigen::Matrix<T, 6, 1> VectorFromIsometry3(const Eigen::Transform<T, 3, Eigen::Isometry>& isometry_3) {
  // Isometry3d linear().data() returns the data pointer to the full Isometry3d matrix rather than just the rotation
  const Eigen::Matrix<T, 3, 3> rotation = isometry_3.linear();
  Eigen::Matrix<T, 6, 1> isometry_3_vector;
  ceres::RotationMatrixToAngleAxis(rotation.data(), &(isometry_3_vector.data()[0]));
  isometry_3_vector[3] = isometry_3.translation().x();
  isometry_3_vector[4] = isometry_3.translation().y();
  isometry_3_vector[5] = isometry_3.translation().z();
  return isometry_3_vector;
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> Isometry3(const T* isometry_data) {
  Eigen::Matrix<T, 3, 3> rotation;
  ceres::AngleAxisToRotationMatrix(isometry_data, rotation.data());
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(&isometry_data[3]);
  Eigen::Transform<T, 3, Eigen::Isometry> isometry_3(Eigen::Transform<T, 3, Eigen::Isometry>::Identity());
  isometry_3.linear() = rotation;
  isometry_3.translation() = translation;
  return isometry_3;
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> Affine3(const T* affine_data) {
  const Eigen::Transform<T, 3, Eigen::Isometry> isometry_3 = Isometry3(affine_data);
  const T scale = affine_data[6];
  Eigen::Transform<T, 3, Eigen::Affine> affine_3;
  affine_3.linear() = scale * isometry_3.linear();
  affine_3.translation() = isometry_3.translation();
  return affine_3;
}

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
}  // namespace optimization_common

#endif  // OPTIMIZATION_COMMON_UTILITIES_H_
