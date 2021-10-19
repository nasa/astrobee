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
#include <depth_odometry/optimization_residuals.h>
#include <localization_common/logger.h>
#include <sparse_mapping/reprojection.h>

#include <ceres/ceres.h>
#include <ceres/solver.h>

namespace depth_odometry {
boost::optional<Eigen::Isometry3d> CameraTTarget(const camera::CameraParameters& camera,
                                                 const depth_odometry::ImageCorrespondences& matches) {
  Eigen::Isometry3d camera_T_target(Eigen::Isometry3d::Identity());
  constexpr int num_ransac_iterations = 100;
  constexpr int ransac_inlier_tolerance = 3;
  camera::CameraModel cam_model(camera_T_target, camera);
  if (!sparse_mapping::RansacEstimateCameraWithDistortion(matches.points_3d, matches.image_points,
                                                          num_ransac_iterations, ransac_inlier_tolerance, &cam_model))
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
    } else {
      LogError("Got camera_T_target, adding parameter block and cost functions.");
    }
    camera_T_targets.emplace_back(VectorFromIsometry3d(*camera_T_target));
    problem.AddParameterBlock(camera_T_targets.back().data(), 6);
    for (int i = 0; i < static_cast<int>(match_set.image_points.size()) && i < params_.max_num_match_sets; ++i) {
      AddReprojectionCostFunction(match_set.image_points[i], match_set.points_3d[i], camera_T_targets.back(),
                                  intrinsics, distortion, problem);
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
