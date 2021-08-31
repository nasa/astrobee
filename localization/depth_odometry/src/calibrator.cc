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

#include <depth_odometry/calibrator.h>

#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/autodiff_cost_function.h>

namespace depth_odometry {
void Calibrator::AddCostFunction(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d,
                                 const Eigen::Matrix3d& intrinsics_matrix,
                                 Eigen::Matrix<double, 7, 1>& depth_image_A_depth_cloud_vector,
                                 ceres::Problem& problem) {
  // change intrinsics to be a parameter! set to constant initially!
  // toggle const vs non const to switch between intrinsics vs affine vs both calibration!!!
  // TODO: pass this? delete at end?
  LogError("image pt: " << image_point.matrix());
  LogError("pt3d: " << point_3d.matrix());
  LogError("intrinsics: " << intrinsics_matrix.matrix());
  LogError("depthAdepth: " << depth_image_A_depth_cloud_vector.matrix());
  ceres::LossFunction* huber_loss = new ceres::HuberLoss(1.345);
  ceres::CostFunction* reprojection_cost_function = new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7>(
    new ReprojectionError(intrinsics_matrix, image_point, point_3d));
  problem.AddResidualBlock(reprojection_cost_function, huber_loss, depth_image_A_depth_cloud_vector.data());
}

// Organize as compact quaternion, translation, scale
Eigen::Matrix<double, 7, 1> Calibrator::VectorFromAffine3d(const Eigen::Affine3d& affine_3d) {
  Eigen::Matrix3d rotation;
  Eigen::Matrix3d scale_matrix;
  affine_3d.computeRotationScaling(&rotation, &scale_matrix);
  LogError("rot: " << rotation.matrix());
  LogError("scale: " << scale_matrix.matrix());
  // Assumes uniform scaling, which is (i*the case for Affine3d
  const double scale = scale_matrix(0, 0);
  Eigen::Quaterniond quaternion(rotation);
  LogError("quat: " << quaternion.matrix());
  // Use normalized x,y,z components for compact quaternion
  // TODO(rsoussan): Is this normalize call necessary?
  quaternion.normalize();
  Eigen::Vector3d compact_quaternion = quaternion.vec();
  LogError("cq: " << compact_quaternion.matrix());
  Eigen::Matrix<double, 7, 1> affine_3d_vector;
  affine_3d_vector.head<3>() = compact_quaternion;
  affine_3d_vector.block<3, 1>(3, 0) = affine_3d.translation();
  affine_3d_vector(6, 0) = scale;
  LogError("vec affine: " << affine_3d_vector.matrix());
  return affine_3d_vector;
}

Eigen::Affine3d Calibrator::Calibrate(const std::vector<DepthMatches>& match_sets,
                                      const Eigen::Affine3d& initial_depth_image_A_depth_cloud) {
  LogError("cal dAd: " << initial_depth_image_A_depth_cloud.matrix());
  Eigen::Matrix<double, 7, 1> depth_image_A_depth_cloud = VectorFromAffine3d(initial_depth_image_A_depth_cloud);
  LogError("cal dAdv: " << depth_image_A_depth_cloud.matrix());
  Eigen::Matrix3d intrinsics_matrix = params_.camera_params->GetIntrinsicMatrix<camera::DISTORTED>();
  ceres::Problem problem;
  for (const auto& match_set : match_sets) {
    for (int i = 0; i < static_cast<int>(match_set.source_image_points.size()) && i < params_.max_num_match_sets; ++i) {
      AddCostFunction(match_set.source_image_points[i], match_set.source_3d_points[i], intrinsics_matrix,
                      depth_image_A_depth_cloud, problem);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.max_num_iterations = params_.max_num_iterations;
  options.function_tolerance = params_.function_tolerance;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  return Affine3<double>(depth_image_A_depth_cloud.data());
}
}  // namespace depth_odometry
