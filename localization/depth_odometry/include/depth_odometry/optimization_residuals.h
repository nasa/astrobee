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
#ifndef DEPTH_ODOMETRY_OPTIMIZATION_RESIDUALS_H_
#define DEPTH_ODOMETRY_OPTIMIZATION_RESIDUALS_H_

#include <depth_odometry/optimization_utilities.h>

#include <Eigen/Core>

#include <ceres/problem.h>
#include <ceres/jet.h>
#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/autodiff_cost_function.h>

namespace depth_odometry {
class PointToPointError {
 public:
  PointToPointError(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point)
      : source_point_(source_point), target_point_(target_point) {}

  template <typename T>
  bool operator()(const T* relative_transform_data, T* point_to_point_error) const {
    const auto relative_transform = Isometry3<T>(relative_transform_data);
    // Compute error
    const Eigen::Matrix<T, 3, 1> transformed_source_point = relative_transform * source_point_.cast<T>();
    point_to_point_error[0] = transformed_source_point[0] - target_point_[0];
    point_to_point_error[1] = transformed_source_point[1] - target_point_[1];
    point_to_point_error[2] = transformed_source_point[2] - target_point_[2];
    return true;
  }

 private:
  Eigen::Vector3d source_point_;
  Eigen::Vector3d target_point_;
};

class PointToPlaneError {
 public:
  PointToPlaneError(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                    const Eigen::Vector3d& target_normal)
      : source_point_(source_point), target_point_(target_point), target_normal_(target_normal) {}

  template <typename T>
  bool operator()(const T* relative_transform_data, T* point_to_plane_error) const {
    const auto relative_transform = Isometry3<T>(relative_transform_data);
    // Compute error
    const Eigen::Matrix<T, 3, 1> transformed_source_point = relative_transform * source_point_.cast<T>();
    const Eigen::Matrix<T, 3, 1> transformed_source_point_to_target_point =
      transformed_source_point - target_point_.cast<T>();
    point_to_plane_error[0] = transformed_source_point_to_target_point.dot(target_normal_.cast<T>());
    return true;
  }

 private:
  Eigen::Vector3d source_point_;
  Eigen::Vector3d target_point_;
  Eigen::Vector3d target_normal_;
};

class SymmetricPointToPlaneError {
 public:
  SymmetricPointToPlaneError(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                             const Eigen::Vector3d& source_normal, const Eigen::Vector3d& target_normal)
      : source_point_(source_point),
        target_point_(target_point),
        source_normal_(source_normal),
        target_normal_(target_normal) {}

  template <typename T>
  bool operator()(const T* relative_transform_data, T* symmetric_point_to_plane_error) const {
    const auto relative_transform = Isometry3<T>(relative_transform_data);
    // Compute error
    const Eigen::Matrix<T, 3, 1> transformed_source_point = relative_transform * source_point_.cast<T>();
    const Eigen::Matrix<T, 3, 1> transformed_source_point_to_target_point =
      transformed_source_point - target_point_.cast<T>();
    symmetric_point_to_plane_error[0] = transformed_source_point_to_target_point.dot(source_normal_.cast<T>());
    symmetric_point_to_plane_error[1] = transformed_source_point_to_target_point.dot(target_normal_.cast<T>());
    return true;
  }

 private:
  Eigen::Vector3d source_point_;
  Eigen::Vector3d target_point_;
  Eigen::Vector3d source_normal_;
  Eigen::Vector3d target_normal_;
};

void AddPointToPointCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                 Eigen::Matrix<double, 6, 1>& relative_transform, ceres::Problem& problem);

void AddPointToPlaneCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                 const Eigen::Vector3d& target_normal, Eigen::Matrix<double, 6, 1>& relative_transform,
                                 ceres::Problem& problem);

void AddSymmetricPointToPlaneCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                          const Eigen::Vector3d& source_normal, const Eigen::Vector3d& target_normal,
                                          Eigen::Matrix<double, 6, 1>& relative_transform, ceres::Problem& problem);
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_OPTIMIZATION_RESIDUALS_H_
