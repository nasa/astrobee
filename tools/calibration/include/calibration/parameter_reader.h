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

#include <calibration/run_calibrator_params.h>
#include <calibration/camera_target_based_intrinsics_calibrator_params.h>
#include <optimization_common/optimization_params.h>
#include <vision_common/reprojection_pose_estimate_params.h>
#include <vision_common/ransac_pnp_params.h>

#include <ceres/solver.h>

#include <string>

namespace calibration {
void LoadRunCalibratorParams(config_reader::ConfigReader& config, RunCalibratorParams& params);

void LoadCameraTargetBasedIntrinsicsCalibratorParams(config_reader::ConfigReader& config,
                                                     CameraTargetBasedIntrinsicsCalibratorParams& params);

void LoadReprojectionPoseEstimateParams(config_reader::ConfigReader& config,
                                        vision_common::ReprojectionPoseEstimateParams& params);

void LoadOptimizationParams(config_reader::ConfigReader& config, optimization_common::OptimizationParams& params,
                            const std::string& prefix = "");

void LoadSolverOptions(config_reader::ConfigReader& config, ceres::Solver::Options& solver_options,
                       const std::string& prefix = "");

void LoadRansacPnPParams(config_reader::ConfigReader& config, vision_common::RansacPnPParams& params);
}  // namespace calibration

#endif  // CALIBRATION_PARAMETER_READER_H_
