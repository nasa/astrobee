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

#include "test_utilities.h"  // NOLINT

#include <localization_common/test_utilities.h>

#include <algorithm>

namespace calibration {
namespace lc = localization_common;

CameraTargetBasedIntrinsicsCalibratorParams DefaultCameraTargetBasedIntrinsicsCalibratorParams() {
  CameraTargetBasedIntrinsicsCalibratorParams params;
  params.optimization = DefaultOptimizationParams();
  params.reprojection_pose_estimate = DefaultReprojectionPoseEstimateParams();
  params.calibrate_focal_lengths = true;
  params.calibrate_principal_points = true;
  params.calibrate_distortion = true;
  params.calibrate_target_poses = true;
  params.scale_loss_radially = false;
  params.radial_scale_power = 1.0;
  params.only_use_inliers = true;
  params.max_num_match_sets = 10000000;
  params.min_num_target_inliers = 4;
  params.save_individual_initial_reprojection_images = false;
  params.save_final_reprojection_image = false;
  params.max_visualization_error_norm = 50;
  params.individual_max_visualization_error_norm = 50;
  params.image_size = Eigen::Vector2i(1280, 960);
  return params;
}

StateParameters AddNoiseToStateParameters(const StateParameters& state_parameters, const double focal_lengths_stddev,
                                          const double principal_points_stddev, const double distortion_stddev,
                                          const bool fov) {
  StateParameters noisy_state_parameters;
  noisy_state_parameters.focal_lengths = lc::AddNoiseToVector(state_parameters.focal_lengths, focal_lengths_stddev);
  noisy_state_parameters.principal_points =
    lc::AddNoiseToVector(state_parameters.principal_points, principal_points_stddev);
  noisy_state_parameters.distortion = lc::AddNoiseToVector(state_parameters.distortion, distortion_stddev);
  if (fov) {
    // Close to zero fov values cause issues for solver, neg values are same as positive values
    noisy_state_parameters.distortion[0] = std::max(0.1, noisy_state_parameters.distortion[0]);
  }
  return noisy_state_parameters;
}
}  // namespace calibration
