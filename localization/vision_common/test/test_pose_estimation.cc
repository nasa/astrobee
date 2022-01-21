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
#include <vision_common/identity_distorter.h>
#include <vision_common/pose_estimation.h>
#include <vision_common/test_utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace oc = optimization_common;
namespace vc = vision_common;
TEST(PoseEstimationTester, Inliers) {
  const auto params = vc::DefaultReprojectionPoseEstimateParams();
  const double inlier_threshold = 3.0;
  const int num_desired_points = 20;
  for (int i = 0; i < 500; ++i) {
    const auto correspondences = vc::RegistrationCorrespondences<vc::IdentityDistorter>(
      vc::RandomFrontFacingPose(), lc::RandomIntrinsics(), vc::RandomFrontFacingPoints(num_desired_points));
    const int num_points = static_cast<int>(correspondences.correspondences().size());
    std::vector<Eigen::Vector2d> noisy_image_points;
    std::unordered_set<int> noisy_point_indices;
    std::unordered_set<int> inlier_point_indices;
    for (int j = 0; j < num_points; ++j) {
      const auto& image_point = correspondences.correspondences().image_points[j];
      const bool add_noise = lc::RandomBool();
      if (add_noise) {
        const double noise = inlier_threshold + lc::RandomPositiveDouble();
        const Eigen::Vector2d noisy_image_point =
          lc::RandomBool() ? image_point + Eigen::Vector2d(noise, 0) : image_point + Eigen::Vector2d(0, noise);
        noisy_image_points.emplace_back(noisy_image_point);
        noisy_point_indices.emplace(j);
      } else {
        noisy_image_points.emplace_back(image_point);
        inlier_point_indices.emplace(j);
      }
    }
    std::vector<int> inliers;
    const int num_inliers = vc::Inliers<vc::IdentityDistorter>(
      noisy_image_points, correspondences.correspondences().points_3d, correspondences.intrinsics(), Eigen::VectorXd(1),
      correspondences.camera_T_target(), inlier_threshold, inliers);
    ASSERT_EQ(num_points, inlier_point_indices.size() + noisy_point_indices.size());
    ASSERT_EQ(num_inliers, num_points - noisy_point_indices.size());
    for (const auto inlier_index : inliers) {
      ASSERT_GT(inlier_point_indices.count(inlier_index), 0);
      ASSERT_EQ(noisy_point_indices.count(inlier_index), 0);
    }
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
