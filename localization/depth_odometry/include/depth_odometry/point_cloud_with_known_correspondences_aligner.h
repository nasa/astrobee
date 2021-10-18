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
#ifndef DEPTH_ODOMETRY_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_H_
#define DEPTH_ODOMETRY_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_H_

#include <depth_odometry/point_cloud_with_known_correspondences_aligner_params.h>
#include <localization_common/pose_with_covariance.h>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ceres/problem.h>
#include <ceres/jet.h>
#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/autodiff_cost_function.h>

#include <boost/optional.hpp>

namespace depth_odometry {
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

class PointCloudWithKnownCorrespondencesAligner {
 public:
  PointCloudWithKnownCorrespondencesAligner(const PointCloudWithKnownCorrespondencesAlignerParams& params);

  Eigen::Isometry3d Align(const std::vector<Eigen::Vector3d>& source_points,
                          const std::vector<Eigen::Vector3d>& target_points,
                          const Eigen::Isometry3d& initial_guess) const;

  localization_common::PoseWithCovariance ComputeRelativeTransform(
    const std::vector<Eigen::Vector3d>& source_points, const std::vector<Eigen::Vector3d>& target_points) const;

  Eigen::Isometry3d ComputeRelativeTransformUmeyama(const std::vector<Eigen::Vector3d>& source_points,
                                                    const std::vector<Eigen::Vector3d>& target_points) const;

  void SetSourceNormals(const std::vector<Eigen::Vector3d>& source_normals);

  void SetTargetNormals(const std::vector<Eigen::Vector3d>& target_normals);

 private:
  PointCloudWithKnownCorrespondencesAlignerParams params_;
  boost::optional<std::vector<Eigen::Vector3d>> source_normals_;
  boost::optional<std::vector<Eigen::Vector3d>> target_normals_;

  // TODO(rsoussan): Add these functions to common place to share with intrinsics calibrator

  void AddPointToPointCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                   Eigen::Matrix<double, 6, 1>& relative_transform, ceres::Problem& problem) const {
    // TODO: pass this? delete at end?
    ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
    ceres::CostFunction* point_to_point_cost_function =
      new ceres::AutoDiffCostFunction<PointToPointError, 3, 6>(new PointToPointError(source_point, target_point));
    problem.AddResidualBlock(point_to_point_cost_function, huber_loss, relative_transform.data());
  }

  void AddPointToPlaneCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                   const Eigen::Vector3d& target_normal,
                                   Eigen::Matrix<double, 6, 1>& relative_transform, ceres::Problem& problem) const {
    // TODO: pass this? delete at end?
    ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
    ceres::CostFunction* point_to_plane_cost_function = new ceres::AutoDiffCostFunction<PointToPlaneError, 1, 6>(
      new PointToPlaneError(source_point, target_point, target_normal));
    problem.AddResidualBlock(point_to_plane_cost_function, huber_loss, relative_transform.data());
  }

  void AddSymmetricPointToPlaneCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                            const Eigen::Vector3d& source_normal, const Eigen::Vector3d& target_normal,
                                            Eigen::Matrix<double, 6, 1>& relative_transform,
                                            ceres::Problem& problem) const {
    // TODO: pass this? delete at end?
    ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
    ceres::CostFunction* symmetric_point_to_plane_cost_function =
      new ceres::AutoDiffCostFunction<SymmetricPointToPlaneError, 2, 6>(
        new SymmetricPointToPlaneError(source_point, target_point, source_normal, target_normal));
    problem.AddResidualBlock(symmetric_point_to_plane_cost_function, huber_loss, relative_transform.data());
  }

  static Eigen::Matrix<double, 6, 1> VectorFromIsometry3d(const Eigen::Isometry3d& isometry_3d) {
    Eigen::Quaterniond quaternion(isometry_3d.linear());
    // Use normalized x,y,z components for compact quaternion
    // TODO(rsoussan): Is this normalize call necessary?
    quaternion.normalize();
    Eigen::Vector3d compact_quaternion = quaternion.vec();
    Eigen::Matrix<double, 6, 1> isometry_3d_vector;
    isometry_3d_vector.head<3>() = compact_quaternion;
    isometry_3d_vector.block<3, 1>(3, 0) = isometry_3d.translation();
    return isometry_3d_vector;
  }
};

}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_H_
