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

#include <camera/camera_params.h>
#include <calibration/camera_target_based_intrinsics_calibrator_params.h>
#include <calibration/utilities.h>
#include <ff_common/eigen_vectors.h>
#include <localization_common/image_correspondences.h>
#include <localization_common/logger.h>
#include <optimization_common/fov_distorter.h>
#include <optimization_common/rad_distorter.h>
#include <optimization_common/radtan_distorter.h>
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
                 const camera::CameraParameters& camera_params, const Eigen::Vector2d& initial_focal_lengths,
                 const Eigen::Vector2d& initial_principal_points, const Eigen::VectorXd& initial_distortion,
                 Eigen::Vector2d& calibrated_focal_lengths, Eigen::Vector2d& calibrated_principal_points,
                 Eigen::VectorXd& calibrated_distortion);

  const CameraTargetBasedIntrinsicsCalibratorParams& params() { return params_; }

 private:
  double RadialScaleFactor(const Eigen::Vector2d& image_point, const Eigen::Vector2i& image_size) const;

  CameraTargetBasedIntrinsicsCalibratorParams params_;
};

template <typename DISTORTER>
void CameraTargetBasedIntrinsicsCalibrator<DISTORTER>::Calibrate(
  const std::vector<localization_common::ImageCorrespondences>& match_sets,
  const camera::CameraParameters& camera_params, const Eigen::Vector2d& initial_focal_lengths,
  const Eigen::Vector2d& initial_principal_points, const Eigen::VectorXd& initial_distortion,
  Eigen::Vector2d& calibrated_focal_lengths, Eigen::Vector2d& calibrated_principal_points,
  Eigen::VectorXd& calibrated_distortion) {
  Eigen::Vector2d focal_lengths = initial_focal_lengths;
  Eigen::Vector2d principal_points = initial_principal_points;
  Eigen::VectorXd distortion = initial_distortion;

  ceres::Problem problem;
  problem.AddParameterBlock(focal_lengths.data(), 2);
  problem.AddParameterBlock(principal_points.data(), 2);
  // Rad distortion is stored internally as RadTan with 4 parameters but only 2 are optimized
  const int num_distortion_params = params_.distortion_type == "rad" ? 2 : distortion.size();
  // TODO(rsoussan): make function for this! (add param block, set const if necessary, print output)
  problem.AddParameterBlock(distortion.data(), num_distortion_params);
  if (params_.calibrate_focal_lengths)
    LogError("Calibrating focal lengths.");
  else
    problem.SetParameterBlockConstant(focal_lengths.data());
  if (params_.calibrate_principal_points)
    LogError("Calibrating principal points.");
  else
    problem.SetParameterBlockConstant(principal_points.data());
  if (params_.calibrate_distortion)
    LogError("Calibrating distortion.");
  else
    problem.SetParameterBlockConstant(distortion.data());
  if (params_.calibrate_target_poses) LogError("Calibrating target poses.");

  std::vector<Eigen::Matrix<double, 6, 1>> camera_T_targets;
  std::vector<Eigen::Isometry3d> initial_camera_T_targets;

  const Eigen::Matrix3d initial_intrinsics = optimization_common::Intrinsics(focal_lengths, principal_points);
  // TODO(rsoussan): More efficient way to do this
  std::vector<localization_common::ImageCorrespondences> valid_match_sets;
  camera_T_targets.reserve(match_sets.size());
  for (const auto& match_set : match_sets) {
    // TODO(rsoussan): Remove once class is templated
    boost::optional<std::pair<Eigen::Isometry3d, std::vector<int>>> camera_T_target;
    if (params_.distortion_type == "fov") {
      camera_T_target = ReprojectionPoseEstimate<optimization_common::FovDistorter>(
        match_set.image_points, match_set.points_3d, focal_lengths, principal_points, distortion,
        params_.reprojection_pose_estimate);
    } else if (params_.distortion_type == "rad") {
      camera_T_target = ReprojectionPoseEstimate<optimization_common::RadDistorter>(
        match_set.image_points, match_set.points_3d, focal_lengths, principal_points, distortion,
        params_.reprojection_pose_estimate);
    } else if (params_.distortion_type == "radtan") {
      camera_T_target = ReprojectionPoseEstimate<optimization_common::RadTanDistorter>(
        match_set.image_points, match_set.points_3d, focal_lengths, principal_points, distortion,
        params_.reprojection_pose_estimate);
    } else {
      LogFatal("Invalid distortion type provided.");
    }

    if (!camera_T_target) {
      LogError("Failed to get camera_T_target with " << match_set.points_3d.size() << " matches.");
      continue;
    }
    camera_T_targets.emplace_back(optimization_common::VectorFromIsometry3d(camera_T_target->first));
    initial_camera_T_targets.emplace_back(camera_T_target->first);
    if (params_.save_individual_initial_reprojection_images) {
      static int image_count = 0;
      // TODO(rsoussan): use same template arg here!!
      SaveReprojectionImage<optimization_common::RadDistorter>(
        match_set.image_points, match_set.points_3d, camera_T_target->second, initial_intrinsics, distortion,
        camera_T_target->first, params_.individual_max_visualization_error_norm,
        "reprojection_image_" + std::to_string(image_count++) + ".png");
    }

    valid_match_sets.emplace_back(match_set);
    problem.AddParameterBlock(camera_T_targets.back().data(), 6);
    if (!params_.calibrate_target_poses) problem.SetParameterBlockConstant(camera_T_targets.back().data());
    for (int i = 0; i < static_cast<int>(match_set.image_points.size()) && i < params_.max_num_match_sets; ++i) {
      const double radial_scale_factor = RadialScaleFactor(match_set.image_points[i], params_.image_size);
      if (params_.distortion_type == "fov") {
        optimization_common::AddReprojectionCostFunction<optimization_common::FovDistorter>(
          match_set.image_points[i], match_set.points_3d[i], camera_T_targets.back(), focal_lengths, principal_points,
          distortion, problem, radial_scale_factor, params_.optimization.huber_loss);
      } else if (params_.distortion_type == "rad") {
        optimization_common::AddReprojectionCostFunction<optimization_common::RadDistorter>(
          match_set.image_points[i], match_set.points_3d[i], camera_T_targets.back(), focal_lengths, principal_points,
          distortion, problem, radial_scale_factor, params_.optimization.huber_loss);
      } else if (params_.distortion_type == "radtan") {
        optimization_common::AddReprojectionCostFunction<optimization_common::RadTanDistorter>(
          match_set.image_points[i], match_set.points_3d[i], camera_T_targets.back(), focal_lengths, principal_points,
          distortion, problem, radial_scale_factor, params_.optimization.huber_loss);
      } else {
        LogFatal("Invalid distortion type provided.");
      }
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
  if (params_.distortion_type == "fov") {
    SaveReprojectionFromAllTargetsImage<optimization_common::FovDistorter>(
      camera_T_targets, valid_match_sets, calibrated_intrinsics, distortion, params_.image_size,
      params_.max_visualization_error_norm);
  } else if (params_.distortion_type == "rad") {
    SaveReprojectionFromAllTargetsImage<optimization_common::RadDistorter>(
      camera_T_targets, valid_match_sets, calibrated_intrinsics, distortion, params_.image_size,
      params_.max_visualization_error_norm);
  } else if (params_.distortion_type == "radtan") {
    SaveReprojectionFromAllTargetsImage<optimization_common::RadTanDistorter>(
      camera_T_targets, valid_match_sets, calibrated_intrinsics, distortion, params_.image_size,
      params_.max_visualization_error_norm);
  } else {
    LogFatal("Invalid distortion type provided.");
  }
}

template <typename DISTORTER>
double CameraTargetBasedIntrinsicsCalibrator<DISTORTER>::RadialScaleFactor(const Eigen::Vector2d& image_point,
                                                                           const Eigen::Vector2i& image_size) const {
  if (!params_.scale_loss_radially) return 1.0;
  const Eigen::Vector2d centered_image_point = image_point - image_size.cast<double>() / 2.0;
  const double radius = centered_image_point.norm();
  return std::pow(radius, params_.radial_scale_power);
}
}  // namespace calibration

#endif  // CALIBRATION_CAMERA_TARGET_BASED_INTRINSICS_CALIBRATOR_H_
