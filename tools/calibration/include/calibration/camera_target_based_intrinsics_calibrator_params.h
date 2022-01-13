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
#ifndef CALIBRATION_CAMERA_TARGET_BASED_INTRINSICS_CALIBRATOR_PARAMS_H_
#define CALIBRATION_CAMERA_TARGET_BASED_INTRINSICS_CALIBRATOR_PARAMS_H_

#include <camera/camera_params.h>
#include <optimization_common/optimization_params.h>
#include <vision_common/reprojection_pose_estimate_params.h>

#include <Eigen/Core>

#include <memory>
#include <string>

namespace calibration {
struct CameraTargetBasedIntrinsicsCalibratorParams {
  optimization_common::OptimizationParams optimization;
  vision_common::ReprojectionPoseEstimateParams reprojection_pose_estimate;
  bool calibrate_focal_lengths;
  bool calibrate_principal_points;
  bool calibrate_distortion;
  bool calibrate_target_poses;
  // Optimization Options
  bool scale_loss_radially;
  double radial_scale_power;
  // Other
  bool only_use_inliers;
  int max_num_match_sets;
  int min_num_target_inliers;
  double max_visualization_error_norm;
  bool save_individual_initial_reprojection_images;
  bool save_final_reprojection_image;
  double individual_max_visualization_error_norm;
  Eigen::Vector2i image_size;
};
}  // namespace calibration

#endif  // CALIBRATION_CAMERA_TARGET_BASED_INTRINSICS_CALIBRATOR_PARAMS_H_
