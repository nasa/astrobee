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
#include <localization_common/image_correspondences.h>
#include <localization_common/logger.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace calibration {
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
}  // namespace calibration

#endif  // CALIBRATION_CAMERA_TARGET_BASED_INTRINSICS_CALIBRATOR_H_
