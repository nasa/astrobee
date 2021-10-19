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
#include <depth_odometry/optimization_residuals.h>

namespace depth_odometry {
void AddPointToPointCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                 Eigen::Matrix<double, 6, 1>& relative_transform, ceres::Problem& problem) {
  // TODO: pass this? delete at end?
  ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
  ceres::CostFunction* point_to_point_cost_function =
    new ceres::AutoDiffCostFunction<PointToPointError, 3, 6>(new PointToPointError(source_point, target_point));
  problem.AddResidualBlock(point_to_point_cost_function, huber_loss, relative_transform.data());
}

void AddPointToPlaneCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                 const Eigen::Vector3d& target_normal, Eigen::Matrix<double, 6, 1>& relative_transform,
                                 ceres::Problem& problem) {
  // TODO: pass this? delete at end?
  ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
  ceres::CostFunction* point_to_plane_cost_function = new ceres::AutoDiffCostFunction<PointToPlaneError, 1, 6>(
    new PointToPlaneError(source_point, target_point, target_normal));
  problem.AddResidualBlock(point_to_plane_cost_function, huber_loss, relative_transform.data());
}

void AddSymmetricPointToPlaneCostFunction(const Eigen::Vector3d& source_point, const Eigen::Vector3d& target_point,
                                          const Eigen::Vector3d& source_normal, const Eigen::Vector3d& target_normal,
                                          Eigen::Matrix<double, 6, 1>& relative_transform, ceres::Problem& problem) {
  // TODO: pass this? delete at end?
  ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
  ceres::CostFunction* symmetric_point_to_plane_cost_function =
    new ceres::AutoDiffCostFunction<SymmetricPointToPlaneError, 2, 6>(
      new SymmetricPointToPlaneError(source_point, target_point, source_normal, target_normal));
  problem.AddResidualBlock(symmetric_point_to_plane_cost_function, huber_loss, relative_transform.data());
}

void AddAffineReprojectionCostFunction(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d,
                                       Eigen::Matrix<double, 7, 1>& depth_image_A_depth_cloud_vector,
                                       Eigen::Matrix<double, 4, 1>& intrinsics_vector,
                                       Eigen::Matrix<double, 4, 1>& distortion, ceres::Problem& problem) {
  // change intrinsics to be a parameter! set to constant initially!
  // toggle const vs non const to switch between intrinsics vs affine vs both calibration!!!
  // TODO: pass this? delete at end?
  ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
  ceres::CostFunction* reprojection_cost_function =
    new ceres::AutoDiffCostFunction<AffineReprojectionError, 2, 7, 4, 4>(
      new AffineReprojectionError(image_point, point_3d));
  problem.AddResidualBlock(reprojection_cost_function, huber_loss, depth_image_A_depth_cloud_vector.data(),
                           intrinsics_vector.data(), distortion.data());
}

void AddReprojectionCostFunction(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d,
                                 Eigen::Matrix<double, 6, 1>& camera_T_target,
                                 Eigen::Matrix<double, 4, 1>& intrinsics_vector,
                                 Eigen::Matrix<double, 4, 1>& distortion, ceres::Problem& problem) {
  // TODO: pass this? delete at end?
  ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
  ceres::CostFunction* reprojection_cost_function =
    new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 4, 4>(new ReprojectionError(image_point, point_3d));
  problem.AddResidualBlock(reprojection_cost_function, huber_loss, camera_T_target.data(), intrinsics_vector.data(),
                           distortion.data());
}
}  // namespace depth_odometry
