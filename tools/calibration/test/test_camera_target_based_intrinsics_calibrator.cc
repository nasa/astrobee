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

#include <calibration/camera_target_based_intrinsics_calibrator.h>
#include <calibration/state_parameters.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <vision_common/identity_distorter.h>
#include <vision_common/fov_distorter.h>
#include <vision_common/rad_distorter.h>
#include <vision_common/radtan_distorter.h>
#include <vision_common/utilities.h>

#include <gtest/gtest.h>

namespace ca = calibration;
namespace lc = localization_common;
namespace vc = vision_common;

TEST(CameraTargetBasedIntrinsicsCalibratorTester, EvenlySpacedTargetsIdentityDistortionWithNoise) {
  auto params = ca::DefaultCameraTargetBasedIntrinsicsCalibratorParams();
  params.calibrate_distortion = false;
  params.calibrate_target_poses = false;
  const int num_target_points_per_row_and_col = 10;
  const int num_pose_rows = 3;
  const int num_pose_cols = 5;
  const int num_pose_y_levels = 2;
  const double focal_lengths_stddev = 1.0;
  const double principal_points_stddev = 1.0;
  const double distortion_stddev = 0;
  for (int i = 0; i < 10; ++i) {
    const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
    ca::StateParameters true_state_parameters;
    true_state_parameters.focal_lengths = vc::FocalLengths(intrinsics);
    true_state_parameters.principal_points = vc::PrincipalPoints(intrinsics);
    true_state_parameters.distortion = Eigen::VectorXd(1);
    true_state_parameters.distortion[0] = 0.0;
    const auto noisy_state_parameters = ca::AddNoiseToStateParameters(true_state_parameters, focal_lengths_stddev,
                                                                      principal_points_stddev, distortion_stddev);
    ca::StateParameters calibrated_state_parameters;
    const auto match_sets = ca::EvenlySpacedTargetMatchSets<vc::IdentityDistorter>(
      num_pose_rows, num_pose_cols, num_pose_y_levels, num_target_points_per_row_and_col, intrinsics);
    ca::CameraTargetBasedIntrinsicsCalibrator<vc::IdentityDistorter> calibrator(params);
    ca::StateParametersCovariances covariances;
    ASSERT_TRUE(calibrator.Calibrate(match_sets, noisy_state_parameters, calibrated_state_parameters, covariances));
    EXPECT_MATRIX_NEAR(true_state_parameters.focal_lengths, calibrated_state_parameters.focal_lengths, 1e-2);
    EXPECT_MATRIX_NEAR(true_state_parameters.principal_points, calibrated_state_parameters.principal_points, 1e-2);
  }
}

TEST(CameraTargetBasedIntrinsicsCalibratorTester, EvenlySpacedTargetsFovDistortionWithNoise) {
  auto params = ca::DefaultCameraTargetBasedIntrinsicsCalibratorParams();
  params.calibrate_target_poses = false;
  const int num_target_points_per_row_and_col = 10;
  const int num_pose_rows = 3;
  const int num_pose_cols = 5;
  const int num_pose_y_levels = 2;
  const double focal_lengths_stddev = 1.0;
  const double principal_points_stddev = 1.0;
  const double distortion_stddev = 0.05;
  for (int i = 0; i < 10; ++i) {
    const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
    ca::StateParameters true_state_parameters;
    true_state_parameters.focal_lengths = vc::FocalLengths(intrinsics);
    true_state_parameters.principal_points = vc::PrincipalPoints(intrinsics);
    true_state_parameters.distortion = vc::RandomFovDistortion();
    const auto noisy_state_parameters = ca::AddNoiseToStateParameters(true_state_parameters, focal_lengths_stddev,
                                                                      principal_points_stddev, distortion_stddev);
    ca::StateParameters calibrated_state_parameters;
    const auto match_sets = ca::EvenlySpacedTargetMatchSets<vc::FovDistorter>(
      num_pose_rows, num_pose_cols, num_pose_y_levels, num_target_points_per_row_and_col, intrinsics,
      true_state_parameters.distortion);
    ca::CameraTargetBasedIntrinsicsCalibrator<vc::FovDistorter> calibrator(params);
    ca::StateParametersCovariances covariances;
    ASSERT_TRUE(calibrator.Calibrate(match_sets, noisy_state_parameters, calibrated_state_parameters, covariances));
    EXPECT_MATRIX_NEAR(true_state_parameters.focal_lengths, calibrated_state_parameters.focal_lengths, 1e-2);
    EXPECT_MATRIX_NEAR(true_state_parameters.principal_points, calibrated_state_parameters.principal_points, 1e-2);
    // Use absolute value for Fov distortion comparison since positive and negative values have same meaning
    EXPECT_MATRIX_NEAR(true_state_parameters.distortion.cwiseAbs(), calibrated_state_parameters.distortion.cwiseAbs(),
                       1e-2);
  }
}

TEST(CameraTargetBasedIntrinsicsCalibratorTester, EvenlySpacedTargetsRadDistortionWithNoise) {
  auto params = ca::DefaultCameraTargetBasedIntrinsicsCalibratorParams();
  params.calibrate_target_poses = false;
  const int num_target_points_per_row_and_col = 10;
  const int num_pose_rows = 3;
  const int num_pose_cols = 5;
  const int num_pose_y_levels = 2;
  const double focal_lengths_stddev = 1.0;
  const double principal_points_stddev = 1.0;
  const double distortion_stddev = 0.05;
  for (int i = 0; i < 10; ++i) {
    const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
    ca::StateParameters true_state_parameters;
    true_state_parameters.focal_lengths = vc::FocalLengths(intrinsics);
    true_state_parameters.principal_points = vc::PrincipalPoints(intrinsics);
    true_state_parameters.distortion = vc::RandomRadDistortion();
    const auto noisy_state_parameters = ca::AddNoiseToStateParameters(true_state_parameters, focal_lengths_stddev,
                                                                      principal_points_stddev, distortion_stddev);
    ca::StateParameters calibrated_state_parameters;
    const auto match_sets = ca::EvenlySpacedTargetMatchSets<vc::RadDistorter>(
      num_pose_rows, num_pose_cols, num_pose_y_levels, num_target_points_per_row_and_col, intrinsics,
      true_state_parameters.distortion);
    ca::CameraTargetBasedIntrinsicsCalibrator<vc::RadDistorter> calibrator(params);
    ca::StateParametersCovariances covariances;
    ASSERT_TRUE(calibrator.Calibrate(match_sets, noisy_state_parameters, calibrated_state_parameters, covariances));
    EXPECT_MATRIX_NEAR(true_state_parameters.focal_lengths, calibrated_state_parameters.focal_lengths, 1e-2);
    EXPECT_MATRIX_NEAR(true_state_parameters.principal_points, calibrated_state_parameters.principal_points, 1e-2);
    EXPECT_MATRIX_NEAR(true_state_parameters.distortion, calibrated_state_parameters.distortion, 1e-2);
  }
}

TEST(CameraTargetBasedIntrinsicsCalibratorTester, EvenlySpacedTargetsRadTanDistortionWithNoise) {
  auto params = ca::DefaultCameraTargetBasedIntrinsicsCalibratorParams();
  params.calibrate_target_poses = false;
  const int num_target_points_per_row_and_col = 10;
  const int num_pose_rows = 3;
  const int num_pose_cols = 5;
  const int num_pose_y_levels = 2;
  const double focal_lengths_stddev = 1.0;
  const double principal_points_stddev = 1.0;
  const double distortion_stddev = 0.05;
  for (int i = 0; i < 10; ++i) {
    const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
    ca::StateParameters true_state_parameters;
    true_state_parameters.focal_lengths = vc::FocalLengths(intrinsics);
    true_state_parameters.principal_points = vc::PrincipalPoints(intrinsics);
    true_state_parameters.distortion = vc::RandomRadTanDistortion();
    const auto noisy_state_parameters = ca::AddNoiseToStateParameters(true_state_parameters, focal_lengths_stddev,
                                                                      principal_points_stddev, distortion_stddev);
    ca::StateParameters calibrated_state_parameters;
    const auto match_sets = ca::EvenlySpacedTargetMatchSets<vc::RadTanDistorter>(
      num_pose_rows, num_pose_cols, num_pose_y_levels, num_target_points_per_row_and_col, intrinsics,
      true_state_parameters.distortion);
    ca::CameraTargetBasedIntrinsicsCalibrator<vc::RadTanDistorter> calibrator(params);
    ca::StateParametersCovariances covariances;
    ASSERT_TRUE(calibrator.Calibrate(match_sets, noisy_state_parameters, calibrated_state_parameters, covariances));
    EXPECT_MATRIX_NEAR(true_state_parameters.focal_lengths, calibrated_state_parameters.focal_lengths, 1e-2);
    EXPECT_MATRIX_NEAR(true_state_parameters.principal_points, calibrated_state_parameters.principal_points, 1e-2);
    EXPECT_MATRIX_NEAR(true_state_parameters.distortion, calibrated_state_parameters.distortion, 1e-2);
  }
}
// TODO(rsoussan): Add test with EstimateTargetPoseAndCalibrateIntrinsics once pnp issues are resolved

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
