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
#include <optimization_common/utilities.h>

#include <gtest/gtest.h>

namespace ca = calibration;
namespace lc = localization_common;
namespace oc = optimization_common;
TEST(CameraTargetBasedIntrinsicsCalibratorTester, RandomFrontFacingPosesRandomIntrinsicsAndRandomPoints) {
  const auto params = ca::DefaultReprojectionPoseEstimateParams();
  const double initial_estimate_translation_noise = 0.1;
  const double initial_estimate_rotation_noise = 0.1;
  const int num_points = 20;
  for (int i = 0; i < 500; ++i) {
    const auto correspondences = ca::RegistrationCorrespondences(ca::RandomFrontFacingPose(), lc::RandomIntrinsics(),
                                                                 ca::RandomFrontFacingPoints(num_points));
    void EstimateInitialTargetPosesAndCalibrate(
      const std::vector<localization_common::ImageCorrespondences>& correspondences_set,
      const Eigen::Vector2d& initial_focal_lengths, const Eigen::Vector2d& initial_principal_points,
      const Eigen::VectorXd& initial_distortion, Eigen::Vector2d& calibrated_focal_lengths,
      Eigen::Vector2d& calibrated_principal_points, Eigen::VectorXd& calibrated_distortion);

    const Eigen::Isometry3d noisy_initial_estimate = lc::AddNoiseToIsometry3d(
      correspondences.camera_T_target(), initial_estimate_translation_noise, initial_estimate_rotation_noise);
    const auto pose_estimate = ca::ReprojectionPoseEstimateWithInitialEstimate<oc::IdentityDistorter>(
      correspondences.correspondences().image_points, correspondences.correspondences().points_3d,
      correspondences.intrinsics(), Eigen::VectorXd(1), params, noisy_initial_estimate, initial_inliers);
    ASSERT_TRUE(pose_estimate != boost::none);
    ASSERT_TRUE(pose_estimate->first.matrix().isApprox(correspondences.camera_T_target().matrix(), 1e-6));
    ASSERT_TRUE(pose_estimate->second.size() == num_points);
  }
}

TEST(CameraTargetBasedIntrinsicsCalibratorTester, RandomFrontFacingPosesRandomIntrinsicsTargetPoints) {
  const auto params = ca::DefaultReprojectionPoseEstimateParams();
  const double initial_estimate_translation_noise = 0.1;
  const double initial_estimate_rotation_noise = 0.1;
  const int num_points = 100;
  std::vector<int> initial_inliers(num_points);
  // Fill inliers with all indices
  std::iota(initial_inliers.begin(), initial_inliers.end(), 0);
  for (int i = 0; i < 500; ++i) {
    const auto correspondences =
      ca::RegistrationCorrespondences(ca::RandomFrontFacingPose(), lc::RandomIntrinsics(), ca::TargetPoints(10, 10));
    const Eigen::Isometry3d noisy_initial_estimate = lc::AddNoiseToIsometry3d(
      correspondences.camera_T_target(), initial_estimate_translation_noise, initial_estimate_rotation_noise);
    const auto pose_estimate = ca::ReprojectionPoseEstimateWithInitialEstimate<oc::IdentityDistorter>(
      correspondences.correspondences().image_points, correspondences.correspondences().points_3d,
      correspondences.intrinsics(), Eigen::VectorXd(1), params, noisy_initial_estimate, initial_inliers);
    ASSERT_TRUE(pose_estimate != boost::none);
    ASSERT_TRUE(pose_estimate->first.matrix().isApprox(correspondences.camera_T_target().matrix(), 1e-6));
    ASSERT_TRUE(pose_estimate->second.size() == num_points);
  }
}

// TODO(rsoussan): Add test with ReprojectionPoseEstimate without initial estimate once pnp issues are resolved
