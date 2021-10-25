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
#include <calibration/camera_target_based_intrinsics_calibrator.h>
#include <calibration/camera_utilities.h>
#include <localization_common/logger.h>
#include <optimization_common/fov_distortion.h>
#include <optimization_common/radtan_distortion.h>
#include <optimization_common/residuals.h>
#include <optimization_common/utilities.h>

#include <ceres/ceres.h>
#include <ceres/solver.h>

namespace calibration {
namespace lc = localization_common;
namespace oc = optimization_common;

boost::optional<Eigen::Isometry3d> CameraTTarget(const camera::CameraParameters& camera,
                                                 const lc::ImageCorrespondences& matches) {
  Eigen::Isometry3d camera_T_target(Eigen::Isometry3d::Identity());
  constexpr int num_ransac_iterations = 100;
  constexpr int ransac_inlier_tolerance = 3;
  camera::CameraModel cam_model(camera_T_target, camera);
  // TODO(rsoussan): add num inliers as param
  constexpr int min_num_inliers = 4;
  if (!RansacEstimateCameraWithDistortion(matches.points_3d, matches.image_points, num_ransac_iterations,
                                          ransac_inlier_tolerance, min_num_inliers, &cam_model))
    return boost::none;
  return Eigen::Isometry3d(cam_model.GetTransform().matrix());
}

void CameraTargetBasedIntrinsicsCalibrator::Calibrate(const std::vector<lc::ImageCorrespondences>& match_sets,
                                                      const camera::CameraParameters& camera_params,
                                                      const Eigen::Matrix3d& initial_intrinsics,
                                                      const Eigen::VectorXd& initial_distortion,
                                                      Eigen::Matrix3d& calibrated_intrinsics,
                                                      Eigen::Matrix<double, 4, 1>& calibrated_distortion) {
  Eigen::Matrix<double, 4, 1> intrinsics = oc::VectorFromIntrinsicsMatrix(initial_intrinsics);
  Eigen::VectorXd distortion = initial_distortion;
  ceres::Problem problem;
  problem.AddParameterBlock(intrinsics.data(), 4);
  problem.AddParameterBlock(distortion.data(), distortion.size());

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
      LogError("Failed to get camera_T_target with " << match_set.points_3d.size() << " matches.");
      continue;
    }
    camera_T_targets.emplace_back(oc::VectorFromIsometry3d(*camera_T_target));
    problem.AddParameterBlock(camera_T_targets.back().data(), 6);
    for (int i = 0; i < static_cast<int>(match_set.image_points.size()) && i < params_.max_num_match_sets; ++i) {
      if (params_.distortion_type == "fov") {
        oc::AddReprojectionCostFunction<oc::FovDistortion>(match_set.image_points[i], match_set.points_3d[i],
                                                           camera_T_targets.back(), intrinsics, distortion, problem);
      } else if (params_.distortion_type == "radtan") {
        oc::AddReprojectionCostFunction<oc::RadTanDistortion>(match_set.image_points[i], match_set.points_3d[i],
                                                              camera_T_targets.back(), intrinsics, distortion, problem);
      } else {
        LogFatal("Invalid distortion type provided.");
      }
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.max_num_iterations = params_.max_num_iterations;
  options.function_tolerance = params_.function_tolerance;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  calibrated_intrinsics = oc::Intrinsics<double>(intrinsics.data());
  calibrated_distortion = distortion;
}
}  // namespace calibration
