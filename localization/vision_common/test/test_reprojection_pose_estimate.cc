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

#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <vision_common/fov_distorter.h>
#include <vision_common/identity_distorter.h>
#include <vision_common/pose_estimation.h>
#include <vision_common/rad_distorter.h>
#include <vision_common/radtan_distorter.h>
#include <vision_common/test_utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace vc = vision_common;
TEST(ReprojectionPoseEstimateTester, EvenlySpacedTargetsIdentityDistortionWithNoise) {
  const auto params = vc::DefaultReprojectionPoseEstimateParams();
  const int num_rows = 5;
  const int num_cols = 5;
  const int num_y_levels = 5;
  const auto target_poses = vc::EvenlySpacedTargetPoses(num_rows, num_cols, num_y_levels);
  const auto target_points = vc::TargetPoints(10, 10);
  std::vector<int> initial_inliers(target_points.size());
  // Fill inliers with all indices
  std::iota(initial_inliers.begin(), initial_inliers.end(), 0);
  const double initial_estimate_translation_noise = 0.1;
  const double initial_estimate_rotation_noise = 0.1;
  for (int i = 0; i < static_cast<int>(target_poses.size()); ++i) {
    const auto correspondences =
      vc::RegistrationCorrespondences<vc::IdentityDistorter>(target_poses[i], lc::RandomIntrinsics(), target_points);
    const Eigen::Isometry3d noisy_initial_estimate = lc::AddNoiseToIsometry3d(
      correspondences.camera_T_target(), initial_estimate_translation_noise, initial_estimate_rotation_noise);
    const auto pose_estimate = vc::ReprojectionPoseEstimateWithInitialEstimate<vc::IdentityDistorter>(
      correspondences.correspondences().image_points, correspondences.correspondences().points_3d,
      correspondences.intrinsics(), Eigen::VectorXd(1), params, noisy_initial_estimate, initial_inliers);
    ASSERT_TRUE(pose_estimate != boost::none);
    EXPECT_MATRIX_NEAR(pose_estimate->pose, correspondences.camera_T_target(), 1e-4);
    ASSERT_TRUE(pose_estimate->inliers.size() == target_points.size());
  }
}

TEST(ReprojectionPoseEstimateTester, EvenlySpacedTargetsFovDistortionWithNoise) {
  const auto params = vc::DefaultReprojectionPoseEstimateParams();
  const int num_rows = 5;
  const int num_cols = 5;
  const int num_y_levels = 5;
  const auto target_poses = vc::EvenlySpacedTargetPoses(num_rows, num_cols, num_y_levels);
  const auto target_points = vc::TargetPoints(10, 10);
  std::vector<int> initial_inliers(target_points.size());
  // Fill inliers with all indices
  std::iota(initial_inliers.begin(), initial_inliers.end(), 0);
  const double initial_estimate_translation_noise = 0.1;
  const double initial_estimate_rotation_noise = 0.1;
  for (int i = 0; i < static_cast<int>(target_poses.size()); ++i) {
    const auto distortion = vc::RandomFovDistortion();
    const auto correspondences = vc::RegistrationCorrespondences<vc::FovDistorter>(
      target_poses[i], lc::RandomIntrinsics(), target_points, distortion);
    const Eigen::Isometry3d noisy_initial_estimate = lc::AddNoiseToIsometry3d(
      correspondences.camera_T_target(), initial_estimate_translation_noise, initial_estimate_rotation_noise);
    const auto pose_estimate = vc::ReprojectionPoseEstimateWithInitialEstimate<vc::FovDistorter>(
      correspondences.correspondences().image_points, correspondences.correspondences().points_3d,
      correspondences.intrinsics(), distortion, params, noisy_initial_estimate, initial_inliers);
    ASSERT_TRUE(pose_estimate != boost::none);
    EXPECT_MATRIX_NEAR(pose_estimate->pose, correspondences.camera_T_target(), 1e-4);
    ASSERT_TRUE(pose_estimate->inliers.size() == target_points.size());
  }
}

TEST(ReprojectionPoseEstimateTester, EvenlySpacedTargetsRadDistortionWithNoise) {
  const auto params = vc::DefaultReprojectionPoseEstimateParams();
  const int num_rows = 5;
  const int num_cols = 5;
  const int num_y_levels = 5;
  const auto target_poses = vc::EvenlySpacedTargetPoses(num_rows, num_cols, num_y_levels);
  const auto target_points = vc::TargetPoints(10, 10);
  std::vector<int> initial_inliers(target_points.size());
  // Fill inliers with all indices
  std::iota(initial_inliers.begin(), initial_inliers.end(), 0);
  const double initial_estimate_translation_noise = 0.1;
  const double initial_estimate_rotation_noise = 0.1;
  for (int i = 0; i < static_cast<int>(target_poses.size()); ++i) {
    const auto distortion = vc::RandomRadDistortion();
    const auto correspondences = vc::RegistrationCorrespondences<vc::RadDistorter>(
      target_poses[i], lc::RandomIntrinsics(), target_points, distortion);
    const Eigen::Isometry3d noisy_initial_estimate = lc::AddNoiseToIsometry3d(
      correspondences.camera_T_target(), initial_estimate_translation_noise, initial_estimate_rotation_noise);
    const auto pose_estimate = vc::ReprojectionPoseEstimateWithInitialEstimate<vc::RadDistorter>(
      correspondences.correspondences().image_points, correspondences.correspondences().points_3d,
      correspondences.intrinsics(), distortion, params, noisy_initial_estimate, initial_inliers);
    ASSERT_TRUE(pose_estimate != boost::none);
    EXPECT_MATRIX_NEAR(pose_estimate->pose, correspondences.camera_T_target(), 1e-4);
    ASSERT_TRUE(pose_estimate->inliers.size() == target_points.size());
  }
}

TEST(ReprojectionPoseEstimateTester, EvenlySpacedTargetsRadTanDistortionWithNoise) {
  const auto params = vc::DefaultReprojectionPoseEstimateParams();
  const int num_rows = 5;
  const int num_cols = 5;
  const int num_y_levels = 5;
  const auto target_poses = vc::EvenlySpacedTargetPoses(num_rows, num_cols, num_y_levels);
  const auto target_points = vc::TargetPoints(10, 10);
  std::vector<int> initial_inliers(target_points.size());
  // Fill inliers with all indices
  std::iota(initial_inliers.begin(), initial_inliers.end(), 0);
  const double initial_estimate_translation_noise = 0.1;
  const double initial_estimate_rotation_noise = 0.1;
  for (int i = 0; i < static_cast<int>(target_poses.size()); ++i) {
    const auto distortion = vc::RandomRadTanDistortion();
    const auto correspondences = vc::RegistrationCorrespondences<vc::RadTanDistorter>(
      target_poses[i], lc::RandomIntrinsics(), target_points, distortion);
    const Eigen::Isometry3d noisy_initial_estimate = lc::AddNoiseToIsometry3d(
      correspondences.camera_T_target(), initial_estimate_translation_noise, initial_estimate_rotation_noise);
    const auto pose_estimate = vc::ReprojectionPoseEstimateWithInitialEstimate<vc::RadTanDistorter>(
      correspondences.correspondences().image_points, correspondences.correspondences().points_3d,
      correspondences.intrinsics(), distortion, params, noisy_initial_estimate, initial_inliers);
    ASSERT_TRUE(pose_estimate != boost::none);
    EXPECT_MATRIX_NEAR(pose_estimate->pose, correspondences.camera_T_target(), 1e-4);
    ASSERT_TRUE(pose_estimate->inliers.size() == target_points.size());
  }
}
// TODO(rsoussan): Add test with ReprojectionPoseEstimate without initial estimate once pnp issues are resolved

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
