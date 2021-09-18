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

#include <camera/camera_model.h>
#include <depth_odometry/intrinsics_calibrator.h>
#include <localization_common/logger.h>
#include <sparse_mapping/reprojection.h>

#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/autodiff_cost_function.h>

namespace depth_odometry {
void IntrinsicsCalibrator::AddCostFunction(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d,
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

// Organize as compact quaternion, translation, scale
Eigen::Matrix<double, 6, 1> IntrinsicsCalibrator::VectorFromIsometry3d(const Eigen::Isometry3d& isometry_3d) {
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

// Stored as focal points then principal points
Eigen::Matrix<double, 4, 1> IntrinsicsCalibrator::VectorFromIntrinsicsMatrix(const Eigen::Matrix3d& intrinsics) {
  Eigen::Matrix<double, 4, 1> intrinsics_vector;
  intrinsics_vector(0, 0) = intrinsics(0, 0);
  intrinsics_vector(1, 0) = intrinsics(1, 1);
  intrinsics_vector(2, 0) = intrinsics(0, 2);
  intrinsics_vector(3, 0) = intrinsics(1, 2);
  return intrinsics_vector;
}

boost::optional<Eigen::Isometry3d> CameraTTarget(const camera::CameraParameters& camera,
                                                 const depth_odometry::ImageCorrespondences& matches) {
  Eigen::Isometry3d camera_T_target(Eigen::Isometry3d::Identity());
  constexpr int num_ransac_iterations = 100;
  constexpr int ransac_inlier_tolerance = 3;
  camera::CameraModel cam_model(camera_T_target, camera);
  if (!sparse_mapping::RansacEstimateCamera(matches.points_3d, matches.image_points, num_ransac_iterations,
                                            ransac_inlier_tolerance, &cam_model))
    return boost::none;
  return Eigen::Isometry3d(cam_model.GetTransform().matrix());
}

void IntrinsicsCalibrator::Calibrate(const std::vector<ImageCorrespondences>& match_sets,
                                     const camera::CameraParameters& camera_params,
                                     const Eigen::Matrix3d& initial_intrinsics,
                                     const Eigen::Matrix<double, 4, 1>& initial_distortion,
                                     Eigen::Matrix3d& calibrated_intrinsics,
                                     Eigen::Matrix<double, 4, 1>& calibrated_distortion) {
  Eigen::Matrix<double, 4, 1> intrinsics = VectorFromIntrinsicsMatrix(initial_intrinsics);
  Eigen::Matrix<double, 4, 1> distortion = initial_distortion;
  ceres::Problem problem;
  problem.AddParameterBlock(intrinsics.data(), 4);
  problem.AddParameterBlock(distortion.data(), 4);

  if (params_.calibrate_intrinsics)
    LogError("Calibrating intrinsics.");
  else
    problem.SetParameterBlockConstant(intrinsics.data());
  if (params_.calibrate_distortion)
    LogError("Calibrating distortion.");
  else
    problem.SetParameterBlockConstant(distortion.data());

  std::vector<Eigen::Matrix<double, 6, 1>> camera_T_targets;
  camera_T_targets.reserve(match_sets.size());
  for (const auto& match_set : match_sets) {
    const auto& camera_T_target = CameraTTarget(camera_params, match_set);
    if (!camera_T_target) {
      LogError("Failed to get camera_T_target.");
      continue;
    }
    camera_T_targets.emplace_back(VectorFromIsometry3d(*camera_T_target));
    problem.AddParameterBlock(camera_T_targets.back().data(), 6);
    for (int i = 0; i < static_cast<int>(match_set.image_points.size()) && i < params_.max_num_match_sets; ++i) {
      AddCostFunction(match_set.image_points[i], match_set.points_3d[i], camera_T_targets.back(), intrinsics,
                      distortion, problem);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.max_num_iterations = params_.max_num_iterations;
  options.function_tolerance = params_.function_tolerance;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  calibrated_intrinsics = Intrinsics<double>(intrinsics.data());
  calibrated_distortion = distortion;
}
}  // namespace depth_odometry
