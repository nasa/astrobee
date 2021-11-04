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

#include <calibration/parameter_reader.h>
#include <localization_common/logger.h>
#include <msg_conversions/msg_conversions.h>

#include <opencv2/calib3d/calib3d.hpp>

namespace calibration {
namespace mc = msg_conversions;

void LoadCalibratorParams(config_reader::ConfigReader& config, CameraTargetBasedIntrinsicsCalibratorParams& params) {
  LoadOptimizationParams(config, params.optimization, "calibrator_");
  LoadReprojectionPoseEstimateParams(config, params.reprojection_pose_estimate);
  params.calibrate_focal_lengths = mc::LoadBool(config, "calibrate_focal_lengths");
  params.calibrate_principal_points = mc::LoadBool(config, "calibrate_principal_points");
  params.calibrate_distortion = mc::LoadBool(config, "calibrate_distortion");
  params.calibrate_target_poses = mc::LoadBool(config, "calibrate_target_poses");
  params.scale_loss_radially = mc::LoadBool(config, "scale_loss_radially");
  params.radial_scale_power = mc::LoadDouble(config, "radial_scale_power");
  params.max_num_match_sets = mc::LoadInt(config, "max_num_match_sets");
  params.min_num_target_inliers = mc::LoadInt(config, "min_num_target_inliers");
  params.max_visualization_error_norm = mc::LoadDouble(config, "max_visualization_error_norm");
  const int image_width = mc::LoadInt(config, "image_width");
  const int image_height = mc::LoadInt(config, "image_height");
  params.image_size = Eigen::Vector2i(image_width, image_height);
  params.camera_name = mc::LoadString(config, "camera");
  params.camera_params.reset(new camera::CameraParameters(&config, params.camera_name.c_str()));
  params.distortion_type = mc::LoadString(config, "distortion_type");
}

void LoadReprojectionPoseEstimateParams(config_reader::ConfigReader& config, ReprojectionPoseEstimateParams& params) {
  LoadOptimizationParams(config, params.optimization, "reprojection_");
  LoadRansacPnPParams(config, params.ransac_pnp);
  params.optimize_estimate = mc::LoadBool(config, "reprojection_optimize_estimate");
}

void LoadSolverOptions(config_reader::ConfigReader& config, ceres::Solver::Options& solver_options,
                       const std::string& prefix) {
  const std::string linear_solver = mc::LoadString(config, prefix + "linear_solver");
  if (linear_solver == "dense_qr") {
    solver_options.linear_solver_type = ceres::DENSE_QR;
  } else if (linear_solver == "dense_schur") {
    solver_options.linear_solver_type = ceres::DENSE_SCHUR;
  } else if (linear_solver == "sparse_normal_cholesky") {
    solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  } else if (linear_solver == "sparse_schur") {
    solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  } else if (linear_solver == "iterative_schur") {
    solver_options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  } else {
    LogFatal("LoadSolverOptions: Invalid linear solver provided.");
  }
  solver_options.use_explicit_schur_complement = mc::LoadBool(config, prefix + "use_explicit_schur_complement");
  solver_options.max_num_iterations = mc::LoadInt(config, prefix + "max_num_iterations");
  solver_options.function_tolerance = mc::LoadDouble(config, prefix + "function_tolerance");
  solver_options.parameter_tolerance = mc::LoadDouble(config, prefix + "parameter_tolerance");
}

void LoadOptimizationParams(config_reader::ConfigReader& config, OptimizationParams& params,
                            const std::string& prefix) {
  LoadSolverOptions(config, params.solver_options, prefix);
  params.verbose = mc::LoadBool(config, prefix + "verbose_optimization");
  params.huber_loss = mc::LoadDouble(config, prefix + "huber_loss");
}

void LoadRansacPnPParams(config_reader::ConfigReader& config, RansacPnPParams& params) {
  params.max_inlier_threshold = mc::LoadDouble(config, "ransac_max_inlier_threshold");
  params.num_iterations = mc::LoadInt(config, "ransac_num_iterations");
  params.min_num_inliers = mc::LoadInt(config, "ransac_min_num_inliers");
  const std::string pnp_method = mc::LoadString(config, "ransac_pnp_method");
  // TODO(rsoussan): Add ippe once opencv version is updated
  if (pnp_method == "p3p") {
    params.pnp_method = cv::SOLVEPNP_P3P;
  } else if (pnp_method == "epnp") {
    params.pnp_method = cv::SOLVEPNP_EPNP;
  } else if (pnp_method == "ap3p") {
    params.pnp_method = cv::SOLVEPNP_AP3P;
  } else if (pnp_method == "it") {
    params.pnp_method = cv::SOLVEPNP_ITERATIVE;
  } else {
    LogFatal("LoadRansacPnPParams: Invalid pnp type provided.");
  }
}
}  // namespace calibration
