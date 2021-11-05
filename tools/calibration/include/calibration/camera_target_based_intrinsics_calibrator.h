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
#ifndef CALIBRATION_CAMERA_TARGET_BASED_INTRINSICS_CALIBRATOR_H_
#define CALIBRATION_CAMERA_TARGET_BASED_INTRINSICS_CALIBRATOR_H_

#include <calibration/camera_target_based_intrinsics_calibrator_params.h>
#include <calibration/utilities.h>
#include <ff_common/eigen_vectors.h>
#include <localization_common/image_correspondences.h>
#include <localization_common/logger.h>
#include <optimization_common/residuals.h>
#include <optimization_common/utilities.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/ceres.h>
#include <ceres/solver.h>

#include <utility>
#include <vector>

namespace calibration {
template <typename DISTORTER>
class CameraTargetBasedIntrinsicsCalibrator {
 public:
  explicit CameraTargetBasedIntrinsicsCalibrator(const CameraTargetBasedIntrinsicsCalibratorParams& params)
      : params_(params) {}
  void Calibrate(const std::vector<localization_common::ImageCorrespondences>& match_sets,
                 const Eigen::Vector2d& initial_focal_lengths, const Eigen::Vector2d& initial_principal_points,
                 const Eigen::VectorXd& initial_distortion, Eigen::Vector2d& calibrated_focal_lengths,
                 Eigen::Vector2d& calibrated_principal_points, Eigen::VectorXd& calibrated_distortion);

  const CameraTargetBasedIntrinsicsCalibratorParams& params() { return params_; }

 private:
  double RadialScaleFactor(const Eigen::Vector2d& image_point, const Eigen::Vector2i& image_size) const;
  localization_common::ImageCorrespondences InlierMatches(const localization_common::ImageCorrespondences& match_set,
                                                          const std::vector<int>& inliers) const;

  CameraTargetBasedIntrinsicsCalibratorParams params_;
};

template <typename DISTORTER>
void CameraTargetBasedIntrinsicsCalibrator<DISTORTER>::Calibrate(
  const std::vector<localization_common::ImageCorrespondences>& match_sets,
  const Eigen::Vector2d& initial_focal_lengths, const Eigen::Vector2d& initial_principal_points,
  const Eigen::VectorXd& initial_distortion, Eigen::Vector2d& calibrated_focal_lengths,
  Eigen::Vector2d& calibrated_principal_points, Eigen::VectorXd& calibrated_distortion) {
  Eigen::Vector2d focal_lengths = initial_focal_lengths;
  Eigen::Vector2d principal_points = initial_principal_points;
  Eigen::VectorXd distortion = initial_distortion;

  ceres::Problem problem;
  optimization_common::AddParameterBlock(2, focal_lengths.data(), problem, !params_.calibrate_focal_lengths);
  optimization_common::AddParameterBlock(2, principal_points.data(), problem, !params_.calibrate_principal_points);
  optimization_common::AddParameterBlock(DISTORTER::kNumParams, distortion.data(), problem,
                                         !params_.calibrate_distortion);
  if (params_.calibrate_focal_lengths) LogInfo("Calibrating focal lengths.");
  if (params_.calibrate_principal_points) LogInfo("Calibrating principal points.");
  if (params_.calibrate_distortion) LogInfo("Calibrating distortion.");
  if (params_.calibrate_target_poses) LogInfo("Calibrating target poses.");

  std::vector<Eigen::Matrix<double, 6, 1>> camera_T_targets;
  std::vector<Eigen::Isometry3d> initial_camera_T_targets;

  const Eigen::Matrix3d initial_intrinsics = optimization_common::Intrinsics(focal_lengths, principal_points);
  // TODO(rsoussan): More efficient way to do this
  std::vector<localization_common::ImageCorrespondences> valid_match_sets;
  camera_T_targets.reserve(match_sets.size());
  for (const auto& match_set : match_sets) {
    const auto camera_T_target =
      ReprojectionPoseEstimate<DISTORTER>(match_set.image_points, match_set.points_3d, focal_lengths, principal_points,
                                          distortion, params_.reprojection_pose_estimate);

    if (!camera_T_target) {
      LogError("Failed to get camera_T_target with " << match_set.points_3d.size() << " matches.");
      continue;
    }

    camera_T_targets.emplace_back(optimization_common::VectorFromIsometry3d(camera_T_target->first));
    initial_camera_T_targets.emplace_back(camera_T_target->first);
    if (params_.save_individual_initial_reprojection_images) {
      static int image_count = 0;
      SaveReprojectionImage<DISTORTER>(match_set.image_points, match_set.points_3d, camera_T_target->second,
                                       initial_intrinsics, distortion, camera_T_target->first,
                                       params_.individual_max_visualization_error_norm,
                                       "reprojection_image_" + std::to_string(image_count++) + ".png");
    }

    problem.AddParameterBlock(camera_T_targets.back().data(), 6);
    if (!params_.calibrate_target_poses) problem.SetParameterBlockConstant(camera_T_targets.back().data());
    localization_common::ImageCorrespondences valid_match_set;
    if (params_.only_use_inliers) {
      valid_match_set = InlierMatches(match_set, camera_T_target->second);
    } else {
      valid_match_set = match_set;
    }
    valid_match_sets.emplace_back(valid_match_set);
    for (int i = 0; i < static_cast<int>(match_set.image_points.size()) && i < params_.max_num_match_sets; ++i) {
      const double radial_scale_factor = RadialScaleFactor(match_set.image_points[i], params_.image_size);
      optimization_common::ReprojectionError<DISTORTER>::AddCostFunction(
        match_set.image_points[i], match_set.points_3d[i], camera_T_targets.back(), focal_lengths, principal_points,
        distortion, problem, radial_scale_factor, params_.optimization.huber_loss);
    }
  }

  ceres::Solver::Summary summary;
  ceres::Solve(params_.optimization.solver_options, &problem, &summary);
  if (params_.optimization.verbose) std::cout << summary.FullReport() << std::endl;

  calibrated_focal_lengths = focal_lengths;
  calibrated_principal_points = principal_points;
  calibrated_distortion = distortion;

  if (params_.calibrate_target_poses) PrintCameraTTargetsStats(initial_camera_T_targets, camera_T_targets);

  const Eigen::Matrix3d calibrated_intrinsics =
    optimization_common::Intrinsics(calibrated_focal_lengths, calibrated_principal_points);
  SaveReprojectionFromAllTargetsImage<DISTORTER>(camera_T_targets, valid_match_sets, calibrated_intrinsics, distortion,
                                                 params_.image_size, params_.max_visualization_error_norm);
}

template <typename DISTORTER>
double CameraTargetBasedIntrinsicsCalibrator<DISTORTER>::RadialScaleFactor(const Eigen::Vector2d& image_point,
                                                                           const Eigen::Vector2i& image_size) const {
  if (!params_.scale_loss_radially) return 1.0;
  const Eigen::Vector2d centered_image_point = image_point - image_size.cast<double>() / 2.0;
  const double radius = centered_image_point.norm();
  return std::pow(radius, params_.radial_scale_power);
}

template <typename DISTORTER>
localization_common::ImageCorrespondences CameraTargetBasedIntrinsicsCalibrator<DISTORTER>::InlierMatches(
  const localization_common::ImageCorrespondences& match_set, const std::vector<int>& inliers) const {
  std::vector<Eigen::Vector2d> inlier_image_points;
  std::vector<Eigen::Vector3d> inlier_points_3d;
  for (const int inlier_index : inliers) {
    inlier_image_points.emplace_back(match_set.image_points[inlier_index]);
    inlier_points_3d.emplace_back(match_set.points_3d[inlier_index]);
  }
  return localization_common::ImageCorrespondences(inlier_image_points, inlier_points_3d);
}
}  // namespace calibration

#endif  // CALIBRATION_CAMERA_TARGET_BASED_INTRINSICS_CALIBRATOR_H_
