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

#include <calibration/camera_target_based_intrinsics_calibrator.h>
#include <calibration/test_utilities.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <optimization_common/identity_distorter.h>
#include <optimization_common/fov_distorter.h>
#include <optimization_common/utilities.h>

#include <gtest/gtest.h>

namespace ca = calibration;
namespace lc = localization_common;
namespace oc = optimization_common;

TEST(CameraTargetBasedIntrinsicsCalibratorTester, RandomFrontFacingPosesRandomPointsIdentityDistortionNoNoise) {
  const auto params = ca::DefaultCameraTargetBasedIntrinsicsCalibratorParams();
  const int num_points_per_set = 20;
  const int num_match_sets = 20;
  for (int i = 0; i < 500; ++i) {
    const auto intrinsics = lc::RandomIntrinsics();
    ca::StateParameters true_state_parameters;
    true_state_parameters.focal_lengths = lc::FocalLengths(intrinsics);
    true_state_parameters.principal_points = lc::PrincipalPoints(intrinsics);
    true_state_parameters.distortion = Eigen::VectorXd(1);
    ca::StateParameters calibrated_state_parameters;
    const auto match_sets = ca::RandomMatchSets<oc::IdentityDistorter>(num_match_sets, num_points_per_set, intrinsics);
    ca::CameraTargetBasedIntrinsicsCalibrator<oc::IdentityDistorter> calibrator(params);
    calibrator.Calibrate(match_sets, true_state_parameters, calibrated_state_parameters);
    ASSERT_TRUE(calibrated_state_parameters == true_state_parameters);
  }
}

TEST(CameraTargetBasedIntrinsicsCalibratorTester, RandomFrontFacingPosesRandomPointsFovDistortionNoNoise) {
  const auto params = ca::DefaultCameraTargetBasedIntrinsicsCalibratorParams();
  const int num_points_per_set = 20;
  const int num_match_sets = 20;
  for (int i = 0; i < 500; ++i) {
    const auto intrinsics = lc::RandomIntrinsics();
    ca::StateParameters true_state_parameters;
    true_state_parameters.focal_lengths = lc::FocalLengths(intrinsics);
    true_state_parameters.principal_points = lc::PrincipalPoints(intrinsics);
    true_state_parameters.distortion = Eigen::VectorXd(1);
    // TODO(rsoussan): What should the range be for this?
    // TODO(rsoussan): Add fcn in test utils to generate random fov eigen vectorxd!
    true_state_parameters.distortion[0] = lc::RandomDouble(0, 3.0);
    ca::StateParameters calibrated_state_parameters;
    const auto match_sets = ca::RandomMatchSets<oc::FovDistorter>(num_match_sets, num_points_per_set, intrinsics);
    ca::CameraTargetBasedIntrinsicsCalibrator<oc::FovDistorter> calibrator(params);
    calibrator.Calibrate(match_sets, true_state_parameters, calibrated_state_parameters);
    ASSERT_TRUE(calibrated_state_parameters == true_state_parameters);
  }
}

// TODO(rsoussan): Add test with EstimateTargetPoseAndCalibrateIntrinsics once pnp issues are resolved
