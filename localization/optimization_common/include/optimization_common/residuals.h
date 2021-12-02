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
  PointToPointError(const Eigen::Vector3d& source_t_point, const Eigen::Vector3d& target_t_point)
      : source_t_point_(source_t_point), target_t_point_(target_t_point) {}

  template <typename T>
  bool operator()(const T* target_T_source_data, T* point_to_point_error) const {
    const auto target_T_source = Isometry3<T>(target_T_source_data);
    // Compute error
    const Eigen::Matrix<T, 3, 1> estimated_target_t_point = target_T_source * source_t_point_.cast<T>();
    point_to_point_error[0] = estimated_target_t_point[0] - target_t_point_[0];
    point_to_point_error[1] = estimated_target_t_point[1] - target_t_point_[1];
    point_to_point_error[2] = estimated_target_t_point[2] - target_t_point_[2];
    return true;
  }

  static void AddCostFunction(const Eigen::Vector3d& source_t_point, const Eigen::Vector3d& target_t_point,
                              Eigen::Matrix<double, 6, 1>& target_T_source, ceres::Problem& problem) {
    ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
    ceres::CostFunction* point_to_point_cost_function =
      new ceres::AutoDiffCostFunction<PointToPointError, 3, 6>(new PointToPointError(source_t_point, target_t_point));
    problem.AddResidualBlock(point_to_point_cost_function, huber_loss, target_T_source.data());
  }

 private:
  Eigen::Vector3d source_t_point_;
  Eigen::Vector3d target_t_point_;
};

class PointToPlaneError {
 public:
  PointToPlaneError(const Eigen::Vector3d& source_t_point, const Eigen::Vector3d& target_t_point,
                    const Eigen::Vector3d& target_normal)
      : source_t_point_(source_t_point), target_t_point_(target_t_point), target_normal_(target_normal) {}

  template <typename T>
  bool operator()(const T* target_T_source_data, T* point_to_plane_error) const {
    const auto target_T_source = Isometry3<T>(target_T_source_data);
    // Compute error
    const Eigen::Matrix<T, 3, 1> estimated_target_t_point = target_T_source * source_t_point_.cast<T>();
    const Eigen::Matrix<T, 3, 1> target_F_point_t_estimated_point =
      estimated_target_t_point - target_t_point_.cast<T>();
    point_to_plane_error[0] = target_F_point_t_estimated_point.dot(target_normal_.cast<T>());
    return true;
  }

  static void AddCostFunction(const Eigen::Vector3d& source_t_point, const Eigen::Vector3d& target_t_point,
                              const Eigen::Vector3d& target_normal, Eigen::Matrix<double, 6, 1>& target_T_source,
                              ceres::Problem& problem) {
    ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
    ceres::CostFunction* point_to_plane_cost_function = new ceres::AutoDiffCostFunction<PointToPlaneError, 1, 6>(
      new PointToPlaneError(source_t_point, target_t_point, target_normal));
    problem.AddResidualBlock(point_to_plane_cost_function, huber_loss, target_T_source.data());
  }

 private:
  Eigen::Vector3d source_t_point_;
  Eigen::Vector3d target_t_point_;
  Eigen::Vector3d target_normal_;
};

class SymmetricPointToPlaneError {
 public:
  SymmetricPointToPlaneError(const Eigen::Vector3d& source_t_point, const Eigen::Vector3d& target_t_point,
                             const Eigen::Vector3d& source_normal, const Eigen::Vector3d& target_normal)
      : source_t_point_(source_t_point),
        target_t_point_(target_t_point),
        source_normal_(source_normal),
        target_normal_(target_normal) {}

  template <typename T>
  bool operator()(const T* target_T_source_data, T* symmetric_point_to_plane_error) const {
    const auto target_T_source = Isometry3<T>(target_T_source_data);
    // Compute error
    const Eigen::Matrix<T, 3, 1> estimated_target_t_point = target_T_source * source_t_point_.cast<T>();
    const Eigen::Matrix<T, 3, 1> target_F_point_t_estimated_point =
      estimated_target_t_point - target_t_point_.cast<T>();
    const Eigen::Matrix<T, 3, 1> estimated_source_t_point = target_T_source.inverse() * target_t_point_.cast<T>();
    const Eigen::Matrix<T, 3, 1> source_F_point_t_estimated_point =
      estimated_source_t_point - source_t_point_.cast<T>();
    symmetric_point_to_plane_error[0] = source_F_point_t_estimated_point.dot(source_normal_.cast<T>());
    symmetric_point_to_plane_error[1] = target_F_point_t_estimated_point.dot(target_normal_.cast<T>());
    return true;
  }

  static void AddCostFunction(const Eigen::Vector3d& source_t_point, const Eigen::Vector3d& target_t_point,
                              const Eigen::Vector3d& source_normal, const Eigen::Vector3d& target_normal,
                              Eigen::Matrix<double, 6, 1>& target_T_source, ceres::Problem& problem) {
    ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
    ceres::CostFunction* symmetric_point_to_plane_cost_function =
      new ceres::AutoDiffCostFunction<SymmetricPointToPlaneError, 2, 6>(
        new SymmetricPointToPlaneError(source_t_point, target_t_point, source_normal, target_normal));
    problem.AddResidualBlock(symmetric_point_to_plane_cost_function, huber_loss, target_T_source.data());
  }

 private:
  Eigen::Vector3d source_t_point_;
  Eigen::Vector3d target_t_point_;
  Eigen::Vector3d source_normal_;
  Eigen::Vector3d target_normal_;
};

template <typename DISTORTER>
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
      distorter_.Distort(distortion_data, intrinsics, undistorted_reprojected_point_3d);

    reprojection_error[0] = image_point_[0] - distorted_reprojected_point_3d[0];
    reprojection_error[1] = image_point_[1] - distorted_reprojected_point_3d[1];
    return true;
  }

  static void AddCostFunction(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d,
                              Eigen::Matrix<double, 7, 1>& depth_image_A_depth_cloud_vector,
                              Eigen::Matrix<double, 4, 1>& intrinsics_vector, Eigen::VectorXd& distortion,
                              ceres::Problem& problem) {
    ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
    ceres::CostFunction* reprojection_cost_function =
      new ceres::AutoDiffCostFunction<AffineReprojectionError<DISTORTER>, 2, 7, 4, DISTORTER::kNumParams>(
        new AffineReprojectionError<DISTORTER>(image_point, point_3d));
    problem.AddResidualBlock(reprojection_cost_function, huber_loss, depth_image_A_depth_cloud_vector.data(),
                             intrinsics_vector.data(), distortion.data());
  }

 private:
  Eigen::Vector2d image_point_;
  Eigen::Vector3d depth_cloud_F_point_3d_;
  DISTORTER distorter_;
};

template <typename DISTORTER>
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
      distorter_.Distort(distortion_data, intrinsics, undistorted_reprojected_point_3d);

    reprojection_error[0] = image_point_[0] - distorted_reprojected_point_3d[0];
    reprojection_error[1] = image_point_[1] - distorted_reprojected_point_3d[1];
    return true;
  }

  static void AddCostFunction(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d,
                              Eigen::Matrix<double, 6, 1>& camera_T_target, Eigen::Vector2d& focal_lengths,
                              Eigen::Vector2d& principal_points, Eigen::VectorXd& distortion, ceres::Problem& problem,
                              const double scale_factor = 1, const double huber_threshold = 1.345) {
    ceres::LossFunction* huber_loss = new ceres::HuberLoss(huber_threshold);
    ceres::LossFunction* scaled_huber_loss = new ceres::ScaledLoss(huber_loss, scale_factor, ceres::TAKE_OWNERSHIP);
    ceres::CostFunction* reprojection_cost_function =
      new ceres::AutoDiffCostFunction<ReprojectionError<DISTORTER>, 2, 6, 2, 2, DISTORTER::kNumParams>(
        new ReprojectionError<DISTORTER>(image_point, point_3d));
    problem.AddResidualBlock(reprojection_cost_function, scaled_huber_loss, camera_T_target.data(),
                             focal_lengths.data(), principal_points.data(), distortion.data());
  }

 private:
  Eigen::Vector2d image_point_;
  Eigen::Vector3d target_t_point_3d_;
  DISTORTER distorter_;
};
}  // namespace optimization_common

#endif  // OPTIMIZATION_COMMON_RESIDUALS_H_
