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
#ifndef CALIBRATION_PARAMETER_READER_H_
#define CALIBRATION_PARAMETER_READER_H_

#include <calibration/camera_target_based_intrinsics_calibrator_params.h>
#include <calibration/optimization_params.h>
#include <calibration/reprojection_pose_estimate_params.h>
#include <calibration/ransac_pnp_params.h>

namespace calibration {
void LoadCalibratorParams(config_reader::ConfigReader& config, CameraTargetBasedIntrinsicsCalibratorParams& params);

void LoadReprojectionPoseEstimateParams(config_reader::ConfigReader& config, ReprojectionPoseEstimateParams& params);

void LoadOptimizationParams(config_reader::ConfigReader& config, OptimizationParams& params);

void LoadRansacPnPParams(config_reader::ConfigReader& config, RansacPnPParams& params);
}  // namespace calibration

#endif  // CALIBRATION_PARAMETER_READER_H_
