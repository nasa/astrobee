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
#include <calibration/state_parameters.h>
#include <calibration/test_utilities.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <optimization_common/identity_distorter.h>
#include <optimization_common/fov_distorter.h>
#include <optimization_common/rad_distorter.h>
#include <optimization_common/radtan_distorter.h>
#include <optimization_common/utilities.h>

#include <gtest/gtest.h>

namespace ca = calibration;
namespace lc = localization_common;
namespace oc = optimization_common;

// No Noise Tests
TEST(CameraTargetBasedIntrinsicsCalibratorTester, RandomFrontFacingPosesRandomPointsIdentityDistortionNoNoise) {
  auto params = ca::DefaultCameraTargetBasedIntrinsicsCalibratorParams();
  // Don't calibrate distortion for identity distorter
  params.calibrate_distortion = false;
  const int num_points_per_set = 20;
  const int num_match_sets = 20;
  for (int i = 0; i < 50; ++i) {
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
  for (int i = 0; i < 50; ++i) {
    const auto intrinsics = lc::RandomIntrinsics();
    ca::StateParameters true_state_parameters;
    true_state_parameters.focal_lengths = lc::FocalLengths(intrinsics);
    true_state_parameters.principal_points = lc::PrincipalPoints(intrinsics);
    true_state_parameters.distortion = ca::RandomFovDistortion();
    ca::StateParameters calibrated_state_parameters;
    const auto match_sets = ca::RandomMatchSets<oc::FovDistorter>(num_match_sets, num_points_per_set, intrinsics,
                                                                  true_state_parameters.distortion);
    ca::CameraTargetBasedIntrinsicsCalibrator<oc::FovDistorter> calibrator(params);
    calibrator.Calibrate(match_sets, true_state_parameters, calibrated_state_parameters);
    ASSERT_TRUE(calibrated_state_parameters == true_state_parameters);
  }
}

TEST(CameraTargetBasedIntrinsicsCalibratorTester, RandomFrontFacingPosesRandomPointsRadDistortionNoNoise) {
  const auto params = ca::DefaultCameraTargetBasedIntrinsicsCalibratorParams();
  const int num_points_per_set = 20;
  const int num_match_sets = 20;
  for (int i = 0; i < 50; ++i) {
    const auto intrinsics = lc::RandomIntrinsics();
    ca::StateParameters true_state_parameters;
    true_state_parameters.focal_lengths = lc::FocalLengths(intrinsics);
    true_state_parameters.principal_points = lc::PrincipalPoints(intrinsics);
    true_state_parameters.distortion = ca::RandomRadDistortion();
    ca::StateParameters calibrated_state_parameters;
    const auto match_sets = ca::RandomMatchSets<oc::RadDistorter>(num_match_sets, num_points_per_set, intrinsics,
                                                                  true_state_parameters.distortion);
    ca::CameraTargetBasedIntrinsicsCalibrator<oc::RadDistorter> calibrator(params);
    calibrator.Calibrate(match_sets, true_state_parameters, calibrated_state_parameters);
    ASSERT_TRUE(calibrated_state_parameters == true_state_parameters);
  }
}

TEST(CameraTargetBasedIntrinsicsCalibratorTester, RandomFrontFacingPosesRandomPointsRadTanDistortionNoNoise) {
  const auto params = ca::DefaultCameraTargetBasedIntrinsicsCalibratorParams();
  const int num_points_per_set = 20;
  const int num_match_sets = 20;
  for (int i = 0; i < 50; ++i) {
    const auto intrinsics = lc::RandomIntrinsics();
    ca::StateParameters true_state_parameters;
    true_state_parameters.focal_lengths = lc::FocalLengths(intrinsics);
    true_state_parameters.principal_points = lc::PrincipalPoints(intrinsics);
    true_state_parameters.distortion = ca::RandomRadTanDistortion();
    ca::StateParameters calibrated_state_parameters;
    const auto match_sets = ca::RandomMatchSets<oc::RadTanDistorter>(num_match_sets, num_points_per_set, intrinsics,
                                                                     true_state_parameters.distortion);
    ca::CameraTargetBasedIntrinsicsCalibrator<oc::RadTanDistorter> calibrator(params);
    calibrator.Calibrate(match_sets, true_state_parameters, calibrated_state_parameters);
    ASSERT_TRUE(calibrated_state_parameters == true_state_parameters);
  }
}

// Noise Tests
TEST(CameraTargetBasedIntrinsicsCalibratorTester, RandomFrontFacingPosesRandomPointsIdentityDistortionWithNoise) {
  auto params = ca::DefaultCameraTargetBasedIntrinsicsCalibratorParams();
  // Don't calibrate distortion for identity distorter
  params.calibrate_distortion = false;
  const int num_points_per_set = 20;
  const int num_match_sets = 20;
  const double focal_lengths_stddev = 1.0;
  const double principal_points_stddev = 1.0;
  const double distortion_stddev = 0;
  for (int i = 0; i < 50; ++i) {
    const auto intrinsics = lc::RandomIntrinsics();
    ca::StateParameters true_state_parameters;
    true_state_parameters.focal_lengths = lc::FocalLengths(intrinsics);
    true_state_parameters.principal_points = lc::PrincipalPoints(intrinsics);
    true_state_parameters.distortion = Eigen::VectorXd(1);
    true_state_parameters.distortion[0] = 0;
    const auto noisy_state_parameters = ca::AddNoiseToStateParameters(true_state_parameters, focal_lengths_stddev,
                                                                      principal_points_stddev, distortion_stddev);
    ca::StateParameters calibrated_state_parameters;
    const auto match_sets = ca::RandomMatchSets<oc::IdentityDistorter>(num_match_sets, num_points_per_set, intrinsics);
    ca::CameraTargetBasedIntrinsicsCalibrator<oc::IdentityDistorter> calibrator(params);
    calibrator.Calibrate(match_sets, noisy_state_parameters, calibrated_state_parameters);
    ASSERT_TRUE(calibrated_state_parameters == true_state_parameters);
  }
}

TEST(CameraTargetBasedIntrinsicsCalibratorTester, RandomFrontFacingPosesRandomPointsFovDistortionWithNoise) {
  auto params = ca::DefaultCameraTargetBasedIntrinsicsCalibratorParams();
  const int num_points_per_set = 20;
  const int num_match_sets = 20;
  const double focal_lengths_stddev = 1.0;
  const double principal_points_stddev = 1.0;
  const double distortion_stddev = 0.1;
  for (int i = 0; i < 50; ++i) {
    const auto intrinsics = lc::RandomIntrinsics();
    ca::StateParameters true_state_parameters;
    true_state_parameters.focal_lengths = lc::FocalLengths(intrinsics);
    true_state_parameters.principal_points = lc::PrincipalPoints(intrinsics);
    true_state_parameters.distortion = ca::RandomFovDistortion();
    const auto noisy_state_parameters = ca::AddNoiseToStateParameters(true_state_parameters, focal_lengths_stddev,
                                                                      principal_points_stddev, distortion_stddev, true);
    ca::StateParameters calibrated_state_parameters;
    const auto match_sets = ca::RandomMatchSets<oc::FovDistorter>(num_match_sets, num_points_per_set, intrinsics,
                                                                  true_state_parameters.distortion);
    ca::CameraTargetBasedIntrinsicsCalibrator<oc::FovDistorter> calibrator(params);
    calibrator.Calibrate(match_sets, noisy_state_parameters, calibrated_state_parameters);
    ASSERT_TRUE(calibrated_state_parameters == true_state_parameters);
  }
}

// TODO(rsoussan): Add test with EstimateTargetPoseAndCalibrateIntrinsics once pnp issues are resolved
