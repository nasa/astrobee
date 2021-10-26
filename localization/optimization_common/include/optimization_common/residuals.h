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
#ifndef OPTIMIZATION_COMMON_RESIDUALS_H_
#define OPTIMIZATION_COMMON_RESIDUALS_H_

#include <optimization_common/utilities.h>

#include <Eigen/Core>

#include <ceres/problem.h>
#include <ceres/jet.h>
#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/autodiff_cost_function.h>

namespace optimization_common {
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

template <typename DISTORTION>
class AffineReprojectionError {
 public:
  AffineReprojectionError(const Eigen::Vector2d& image_point, const Eigen::Vector3d& depth_cloud_F_point_3d)
      : image_point_(image_point), depth_cloud_F_point_3d_(depth_cloud_F_point_3d) {}

  template <typename T>
  bool operator()(const T* depth_image_A_depth_cloud_data, const T* intrinsics_data, const T* distortion_data,
                  T* reprojection_error) const {
    // Handle type conversions
    const auto intrinsics = Intrinsics<T>(intrinsics_data);
    const auto depth_image_A_depth_cloud = Affine3<T>(depth_image_A_depth_cloud_data);

    // Compute error
    const Eigen::Matrix<T, 3, 1> depth_image_F_point_3d = depth_image_A_depth_cloud * depth_cloud_F_point_3d_.cast<T>();
    const Eigen::Matrix<T, 2, 1> undistorted_reprojected_point_3d = (intrinsics * depth_image_F_point_3d).hnormalized();
    const Eigen::Matrix<T, 2, 1> distorted_reprojected_point_3d =
      distortion_.Distort(distortion_data, intrinsics, undistorted_reprojected_point_3d);

    reprojection_error[0] = image_point_[0] - distorted_reprojected_point_3d[0];
    reprojection_error[1] = image_point_[1] - distorted_reprojected_point_3d[1];
    return true;
  }

 private:
  Eigen::Vector2d image_point_;
  Eigen::Vector3d depth_cloud_F_point_3d_;
  DISTORTION distortion_;
};

template <typename DISTORTION>
class ReprojectionError {
 public:
  ReprojectionError(const Eigen::Vector2d& image_point, const Eigen::Vector3d& target_t_point_3d)
      : image_point_(image_point), target_t_point_3d_(target_t_point_3d) {}

  template <typename T>
  bool operator()(const T* camera_T_target_data, const T* focal_lengths_data, const T* principal_points_data,
                  const T* distortion_data, T* reprojection_error) const {
    // Handle type conversions
    const auto intrinsics = Intrinsics<T>(focal_lengths_data, principal_points_data);
    const auto camera_T_target = Isometry3<T>(camera_T_target_data);
    // Compute error
    const Eigen::Matrix<T, 3, 1> camera_t_point_3d = camera_T_target * target_t_point_3d_.cast<T>();
    const Eigen::Matrix<T, 2, 1> undistorted_reprojected_point_3d = (intrinsics * camera_t_point_3d).hnormalized();
    const Eigen::Matrix<T, 2, 1> distorted_reprojected_point_3d =
      distortion_.Distort(distortion_data, intrinsics, undistorted_reprojected_point_3d);

    reprojection_error[0] = image_point_[0] - distorted_reprojected_point_3d[0];
    reprojection_error[1] = image_point_[1] - distorted_reprojected_point_3d[1];
    return true;
  }

 private:
  Eigen::Vector2d image_point_;
  Eigen::Vector3d target_t_point_3d_;
  DISTORTION distortion_;
};

void AddPointToPointCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                 Eigen::Matrix<double, 6, 1>& relative_transform, ceres::Problem& problem);

void AddPointToPlaneCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                 const Eigen::Vector3d& target_normal, Eigen::Matrix<double, 6, 1>& relative_transform,
                                 ceres::Problem& problem);

void AddSymmetricPointToPlaneCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                          const Eigen::Vector3d& source_normal, const Eigen::Vector3d& target_normal,
                                          Eigen::Matrix<double, 6, 1>& relative_transform, ceres::Problem& problem);

template <typename DISTORTION>
void AddAffineReprojectionCostFunction(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d,
                                       Eigen::Matrix<double, 7, 1>& depth_image_A_depth_cloud_vector,
                                       Eigen::Matrix<double, 4, 1>& intrinsics_vector, Eigen::VectorXd& distortion,
                                       ceres::Problem& problem) {
  // change intrinsics to be a parameter! set to constant initially!
  // toggle const vs non const to switch between intrinsics vs affine vs both calibration!!!
  // TODO(rsoussan): pass this? delete at end?
  ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
  ceres::CostFunction* reprojection_cost_function =
    new ceres::AutoDiffCostFunction<AffineReprojectionError<DISTORTION>, 2, 7, 4, DISTORTION::NUM_PARAMS>(
      new AffineReprojectionError<DISTORTION>(image_point, point_3d));
  problem.AddResidualBlock(reprojection_cost_function, huber_loss, depth_image_A_depth_cloud_vector.data(),
                           intrinsics_vector.data(), distortion.data());
}

template <typename DISTORTION>
void AddReprojectionCostFunction(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d,
                                 Eigen::Matrix<double, 6, 1>& camera_T_target, Eigen::Vector2d& focal_lengths,
                                 Eigen::Vector2d& principal_points, Eigen::VectorXd& distortion,
                                 ceres::Problem& problem) {
  // TODO(rsoussan): pass this? delete at end?
  ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
  ceres::CostFunction* reprojection_cost_function =
    new ceres::AutoDiffCostFunction<ReprojectionError<DISTORTION>, 2, 6, 2, 2, DISTORTION::NUM_PARAMS>(
      new ReprojectionError<DISTORTION>(image_point, point_3d));
  problem.AddResidualBlock(reprojection_cost_function, huber_loss, camera_T_target.data(), focal_lengths.data(),
                           principal_points.data(), distortion.data());
}
}  // namespace optimization_common

#endif  // OPTIMIZATION_COMMON_RESIDUALS_H_
