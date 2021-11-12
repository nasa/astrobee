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
#include <calibration/match_set.h>
#include <calibration/state_parameters.h>
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

  bool EstimateInitialTargetPosesAndCalibrate(
    const std::vector<localization_common::ImageCorrespondences>& correspondences_set,
    const StateParameters& initial_state_parameters, StateParameters& calibrated_state_parameters,
    StateParametersCovariances& covariances);

  bool Calibrate(const std::vector<MatchSet>& match_sets, const StateParameters& initial_state_parameters,
                 StateParameters& calibrated_state_parameters, StateParametersCovariances& covariances);

  const CameraTargetBasedIntrinsicsCalibratorParams& params() { return params_; }

 private:
  void Initialize(const StateParameters& initial_state_parameters);
  boost::optional<StateParametersCovariances> Covariances();
  void AddCameraTTargetParameter(const Eigen::Isometry3d& camera_T_target);
  double RadialScaleFactor(const Eigen::Vector2d& image_point, const Eigen::Vector2i& image_size) const;
  void SaveResults(const StateParameters& calibrated_state_parameters, const std::vector<MatchSet>& match_sets) const;
  localization_common::ImageCorrespondences InlierMatches(const localization_common::ImageCorrespondences& match_set,
                                                          const std::vector<int>& inliers) const;

  CameraTargetBasedIntrinsicsCalibratorParams params_;
  OptimizationStateParameters state_parameters_;
  ceres::Problem problem_;
  // Keep track of correspondences used to create final reprojection image
  std::vector<localization_common::ImageCorrespondences> valid_correspondences_set_;
};

template <typename DISTORTER>
bool CameraTargetBasedIntrinsicsCalibrator<DISTORTER>::EstimateInitialTargetPosesAndCalibrate(
  const std::vector<localization_common::ImageCorrespondences>& correspondences_set,
  const StateParameters& initial_state_parameters, StateParameters& calibrated_state_parameters,
  StateParametersCovariances& covariances) {
  std::vector<MatchSet> match_sets;
  const Eigen::Matrix3d initial_intrinsics =
    optimization_common::Intrinsics(initial_state_parameters.focal_lengths, initial_state_parameters.principal_points);
  for (const auto& correspondences : correspondences_set) {
    const auto camera_T_target = ReprojectionPoseEstimate<DISTORTER>(
      correspondences.image_points, correspondences.points_3d, initial_state_parameters.focal_lengths,
      initial_state_parameters.principal_points, initial_state_parameters.distortion,
      params_.reprojection_pose_estimate);

    if (!camera_T_target) {
      LogError("Failed to get camera_T_target with " << correspondences.points_3d.size() << " matches.");
      continue;
    }

    match_sets.emplace_back(correspondences, camera_T_target->first, camera_T_target->second);
    if (params_.save_individual_initial_reprojection_images) {
      static int image_count = 0;
      const auto& match_set = match_sets.back();
      SaveReprojectionImage<DISTORTER>(match_set.correspondences.image_points, match_set.correspondences.points_3d,
                                       match_set.inliers, initial_intrinsics, initial_state_parameters.distortion,
                                       match_set.pose_estimate, params_.individual_max_visualization_error_norm,
                                       "reprojection_image_" + std::to_string(image_count++) + ".png");
    }
  }

  return Calibrate(match_sets, initial_state_parameters, calibrated_state_parameters, covariances);
}

template <typename DISTORTER>
bool CameraTargetBasedIntrinsicsCalibrator<DISTORTER>::Calibrate(const std::vector<MatchSet>& match_sets,
                                                                 const StateParameters& initial_state_parameters,
                                                                 StateParameters& calibrated_state_parameters,
                                                                 StateParametersCovariances& covariances) {
  Initialize(initial_state_parameters);
  // Reserve so that pointers aren't modified while new target poses are added.
  // This can mess up ceres::Problem since it relies on pointers of add parameters.
  state_parameters_.camera_T_targets.reserve(match_sets.size());
  // Add residuals to problem
  for (const auto& match_set : match_sets) {
    AddCameraTTargetParameter(match_set.pose_estimate);
    localization_common::ImageCorrespondences valid_correspondences =
      params_.only_use_inliers ? InlierMatches(match_set.correspondences, match_set.inliers)
                               : match_set.correspondences;
    valid_correspondences_set_.emplace_back(valid_correspondences);
    for (int i = 0; i < static_cast<int>(valid_correspondences.image_points.size()) && i < params_.max_num_match_sets;
         ++i) {
      const double radial_scale_factor = RadialScaleFactor(valid_correspondences.image_points[i], params_.image_size);
      optimization_common::ReprojectionError<DISTORTER>::AddCostFunction(
        valid_correspondences.image_points[i], valid_correspondences.points_3d[i],
        state_parameters_.camera_T_targets.back(), state_parameters_.focal_lengths, state_parameters_.principal_points,
        state_parameters_.distortion, problem_, radial_scale_factor, params_.optimization.huber_loss);
    }
  }

  ceres::Solver::Summary summary;
  ceres::Solve(params_.optimization.solver_options, &problem_, &summary);
  if (params_.optimization.verbose) std::cout << summary.FullReport() << std::endl;
  if (!summary.IsSolutionUsable()) {
    LogError("Calibrate: Calibration failed.");
    return false;
  }

  calibrated_state_parameters = state_parameters_.OptimizedStateParameters();
  SaveResults(calibrated_state_parameters, match_sets);

  const auto state_covariances = Covariances();
  if (!state_covariances) {
    LogError("Calibrate: Failed to get covariances");
    return false;
  }
  covariances = *state_covariances;

  return true;
}

template <typename DISTORTER>
void CameraTargetBasedIntrinsicsCalibrator<DISTORTER>::Initialize(const StateParameters& initial_state_parameters) {
  state_parameters_.SetInitialStateParameters(initial_state_parameters);
  optimization_common::AddParameterBlock(2, state_parameters_.focal_lengths.data(), problem_,
                                         !params_.calibrate_focal_lengths);
  optimization_common::AddParameterBlock(2, state_parameters_.principal_points.data(), problem_,
                                         !params_.calibrate_principal_points);
  optimization_common::AddParameterBlock(DISTORTER::kNumParams, state_parameters_.distortion.data(), problem_,
                                         !params_.calibrate_distortion);
  if (params_.optimization.verbose) {
    if (params_.calibrate_focal_lengths) LogInfo("Calibrating focal lengths.");
    if (params_.calibrate_principal_points) LogInfo("Calibrating principal points.");
    if (params_.calibrate_distortion) LogInfo("Calibrating distortion.");
    if (params_.calibrate_target_poses) LogInfo("Calibrating target poses.");
  }
}

template <typename DISTORTER>
boost::optional<StateParametersCovariances> CameraTargetBasedIntrinsicsCalibrator<DISTORTER>::Covariances() {
  ceres::Covariance::Options options;
  ceres::Covariance covariance(options);

  std::vector<std::pair<const double*, const double*> > covariance_blocks;
  covariance_blocks.push_back(
    std::make_pair(state_parameters_.focal_lengths.data(), state_parameters_.focal_lengths.data()));
  covariance_blocks.push_back(
    std::make_pair(state_parameters_.principal_points.data(), state_parameters_.principal_points.data()));
  covariance_blocks.push_back(std::make_pair(state_parameters_.distortion.data(), state_parameters_.distortion.data()));

  if (!covariance.Compute(covariance_blocks, &problem_)) {
    LogError("Covariances: Failed to compute covariances.");
    return boost::none;
  }

  StateParametersCovariances covariances;
  covariance.GetCovarianceBlock(state_parameters_.focal_lengths.data(), state_parameters_.focal_lengths.data(),
                                covariances.focal_lengths.data());
  covariance.GetCovarianceBlock(state_parameters_.principal_points.data(), state_parameters_.principal_points.data(),
                                covariances.principal_points.data());
  covariances.distortion = Eigen::MatrixXd(state_parameters_.distortion.size(), state_parameters_.distortion.size());
  covariance.GetCovarianceBlock(state_parameters_.distortion.data(), state_parameters_.distortion.data(),
                                covariances.distortion.data());
  return covariances;
}

template <typename DISTORTER>
void CameraTargetBasedIntrinsicsCalibrator<DISTORTER>::AddCameraTTargetParameter(
  const Eigen::Isometry3d& camera_T_target) {
  state_parameters_.AddCameraTTarget(camera_T_target);
  optimization_common::AddParameterBlock(6, state_parameters_.camera_T_targets.back().data(), problem_,
                                         !params_.calibrate_target_poses);
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

template <typename DISTORTER>
void CameraTargetBasedIntrinsicsCalibrator<DISTORTER>::SaveResults(const StateParameters& calibrated_state_parameters,
                                                                   const std::vector<MatchSet>& match_sets) const {
  if (!(params_.calibrate_target_poses && params_.optimization.verbose) && !params_.save_final_reprojection_image)
    return;

  const auto calibrated_camera_T_targets = state_parameters_.OptimizedCameraTTargets();
  if (params_.calibrate_target_poses && params_.optimization.verbose)
    PrintCameraTTargetsStats(match_sets, calibrated_camera_T_targets);

  if (params_.save_final_reprojection_image) {
    const Eigen::Matrix3d calibrated_intrinsics = optimization_common::Intrinsics(
      calibrated_state_parameters.focal_lengths, calibrated_state_parameters.principal_points);
    SaveReprojectionFromAllTargetsImage<DISTORTER>(calibrated_camera_T_targets, valid_correspondences_set_,
                                                   calibrated_intrinsics, calibrated_state_parameters.distortion,
                                                   params_.image_size, params_.max_visualization_error_norm);
  }
}
}  // namespace calibration

#endif  // CALIBRATION_CAMERA_TARGET_BASED_INTRINSICS_CALIBRATOR_H_
