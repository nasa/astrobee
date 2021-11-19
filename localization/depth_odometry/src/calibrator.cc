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
#include <localization_common/logger.h>
#include <optimization_common/residuals.h>

#include <ceres/ceres.h>
#include <ceres/solver.h>

namespace depth_odometry {
// TODO(rsoussan): use structs with state parameteres, opt parameters!!
void Calibrator::Calibrate(const std::vector<DepthMatches>& match_sets,
                           const Eigen::Affine3d& initial_depth_image_A_depth_cloud,
                           const Eigen::Matrix3d& initial_intrinsics,
                           const Eigen::Matrix<double, 4, 1>& initial_distortion,
                           Eigen::Affine3d& calibrated_depth_image_A_depth_cloud,
                           Eigen::Matrix3d& calibrated_intrinsics, Eigen::Matrix<double, 4, 1>& calibrated_distortion) {
  Eigen::Matrix<double, 7, 1> depth_image_A_depth_cloud = VectorFromAffine3d(initial_depth_image_A_depth_cloud);
  Eigen::Matrix<double, 4, 1> intrinsics = VectorFromIntrinsicsMatrix(initial_intrinsics);
  Eigen::Matrix<double, 4, 1> distortion = initial_distortion;
  ceres::Problem problem;
  problem.AddParameterBlock(depth_image_A_depth_cloud.data(), 7);
  problem.AddParameterBlock(intrinsics.data(), 4);
  problem.AddParameterBlock(distortion.data(), 4);

  // TODO(rsoussan): replace this with opt common fcn
  if (params_.calibrate_depth_image_A_depth_haz)
    LogError("Calibrating depth_image_A_depth_haz.");
  else
    problem.SetParameterBlockConstant(depth_image_A_depth_cloud.data());
  if (params_.calibrate_intrinsics)
    LogError("Calibrating intrinsics.");
  else
    problem.SetParameterBlockConstant(intrinsics.data());
  if (params_.calibrate_distortion)
    LogError("Calibrating distortion.");
  else
    problem.SetParameterBlockConstant(distortion.data());

  for (const auto& match_set : match_sets) {
    for (int i = 0; i < static_cast<int>(match_set.source_image_points.size()) && i < params_.max_num_match_sets; ++i) {
      oc::AffineReprojectionError::AddCostFunction(match_set.source_image_points[i], match_set.source_3d_points[i],
                                                   depth_image_A_depth_cloud, intrinsics, distortion, problem);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.max_num_iterations = params_.max_num_iterations;
  options.function_tolerance = params_.function_tolerance;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  calibrated_depth_image_A_depth_cloud = Affine3<double>(depth_image_A_depth_cloud.data());
  calibrated_intrinsics = Intrinsics<double>(intrinsics.data());
  calibrated_distortion = distortion;
}
}  // namespace depth_odometry
