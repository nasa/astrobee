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

#include <calibration/camera_target_based_intrinsics_calibrator.h>
#include <calibration/utilities.h>
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

void CameraTargetBasedIntrinsicsCalibrator::Calibrate(const std::vector<lc::ImageCorrespondences>& match_sets,
                                                      const camera::CameraParameters& camera_params,
                                                      const Eigen::Matrix3d& initial_intrinsics,
                                                      const Eigen::VectorXd& initial_distortion,
                                                      Eigen::Matrix3d& calibrated_intrinsics,
                                                      Eigen::VectorXd& calibrated_distortion) {
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
  // TODO(rsoussan): More efficient way to do this
  std::vector<lc::ImageCorrespondences> valid_match_sets;
  camera_T_targets.reserve(match_sets.size());
  for (const auto& match_set : match_sets) {
    const auto& camera_T_target = CameraTTarget(camera_params, match_set);
    if (!camera_T_target) {
      LogError("Failed to get camera_T_target with " << match_set.points_3d.size() << " matches.");
      continue;
    }
    camera_T_targets.emplace_back(oc::VectorFromIsometry3d(*camera_T_target));
    valid_match_sets.emplace_back(match_set);
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
  if (params_.linear_solver == "dense_qr") {
    options.linear_solver_type = ceres::DENSE_QR;
  } else if (params_.linear_solver == "dense_schur") {
    options.linear_solver_type = ceres::DENSE_SCHUR;
  } else if (params_.linear_solver == "sparse_normal_cholesky") {
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  } else if (params_.linear_solver == "sparse_schur") {
    options.linear_solver_type = ceres::SPARSE_SCHUR;
  } else if (params_.linear_solver == "iterative_schur") {
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  } else {
    LogFatal("Invalid linear solver provided.");
  }
  options.use_explicit_schur_complement = params_.use_explicit_schur_complement;
  options.max_num_iterations = params_.max_num_iterations;
  options.function_tolerance = params_.function_tolerance;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  calibrated_intrinsics = oc::Intrinsics<double>(intrinsics.data());
  calibrated_distortion = distortion;
  if (params_.distortion_type == "fov") {
    SaveReprojectionErrors<oc::FovDistortion>(camera_T_targets, valid_match_sets, calibrated_intrinsics,
                                              calibrated_distortion);
  } else if (params_.distortion_type == "radtan") {
    SaveReprojectionErrors<oc::RadTanDistortion>(camera_T_targets, valid_match_sets, calibrated_intrinsics,
                                                 calibrated_distortion);
  } else {
    LogFatal("Invalid distortion type provided.");
  }
}
}  // namespace calibration
