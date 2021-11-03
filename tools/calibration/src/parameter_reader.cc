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
#include <msg_conversions/msg_conversions.h>

namespace calibration {
namespace mc = msg_conversions;

void LoadCalibratorParams(config_reader::ConfigReader& config, CameraTargetBasedIntrinsicsCalibratorParams& params) {
  LoadOptimizationParams(config, params.optimization);
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
  LoadOptimizationParams(config, params.optimization);
  LoadRansacPnPParams(config, params.ransac_pnp);
}

void LoadOptimizationParams(config_reader::ConfigReader& config, OptimizationParams& params) {
  params.max_num_iterations = mc::LoadInt(config, "max_num_iterations");
  params.function_tolerance = mc::LoadDouble(config, "function_tolerance");
  params.huber_loss = mc::LoadDouble(config, "huber_loss");
  params.linear_solver = mc::LoadString(config, "linear_solver");
  params.use_explicit_schur_complement = mc::LoadBool(config, "use_explicit_schur_complement");
}

void LoadRansacPnPParams(config_reader::ConfigReader& config, RansacPnPParams& params) {
  params.max_inlier_threshold = mc::LoadDouble(config, "ransac_max_inlier_threshold");
  params.num_iterations = mc::LoadInt(config, "ransac_num_iterations");
  params.min_num_inliers = mc::LoadInt(config, "ransac_min_num_inliers");
}
}  // namespace calibration
