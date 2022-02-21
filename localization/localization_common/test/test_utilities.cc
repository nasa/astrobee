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
#include <localization_common/utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
TEST(UtilitiesTester, Deg2Rad2Deg) {
  for (int i = 0; i < 500; ++i) {
    const double degrees = lc::RandomDouble(-360.0, 360.0);
    const double radians = lc::Deg2Rad(degrees);
    const double degrees_again = lc::Rad2Deg(radians);
    ASSERT_NEAR(degrees, degrees_again, 1e-6);
  }
}

TEST(UtilitiesTester, EulerAngles) {
  for (int i = 0; i < 500; ++i) {
    const double yaw = lc::RandomDouble(-360.0, 360.0);
    const double pitch = lc::RandomDouble(-360.0, 360.0);
    const double roll = lc::RandomDouble(-360.0, 360.0);
    const Eigen::Matrix3d rotation = lc::RotationFromEulerAngles(yaw, pitch, roll);
    // ypr intrinsics equivalent to rpy extrinsics, and Eigen uses extrinsics convention
    const Eigen::Vector3d angles = rotation.eulerAngles(0, 1, 2);
    const Eigen::Matrix3d rotation_again =
      lc::RotationFromEulerAngles(lc::Rad2Deg(angles[2]), lc::Rad2Deg(angles[1]), lc::Rad2Deg(angles[0]));
    EXPECT_MATRIX_NEAR(rotation, rotation_again, 1e-6);
  }
}

TEST(UtilitiesTester, FrameChangeRelativePose) {
  const Eigen::Isometry3d a_F_b_T_c = lc::Isometry3d(Eigen::Vector3d(0, 0, 1), Eigen::Matrix3d::Identity());
  // Translation only shouldn't change anything
  {
    const Eigen::Isometry3d n_T_a = lc::Isometry3d(Eigen::Vector3d(1, 2, 3), Eigen::Matrix3d::Identity());
    const auto n_F_b_T_c = lc::FrameChangeRelativePose(a_F_b_T_c, n_T_a);
    EXPECT_MATRIX_NEAR(a_F_b_T_c, n_F_b_T_c, 1e-6);
  }
  // Rotation should preserve zero relative rotation
  {
    const Eigen::Isometry3d n_T_a = lc::RandomIsometry3d();
    const auto n_F_b_T_c = lc::FrameChangeRelativePose(a_F_b_T_c, n_T_a);
    EXPECT_MATRIX_NEAR(a_F_b_T_c.linear(), n_F_b_T_c.linear(), 1e-6);
  }
  // Frame change for translation component should be the same as rotating it
  {
    const Eigen::Isometry3d n_T_a = lc::RandomIsometry3d();
    const auto n_F_b_T_c = lc::FrameChangeRelativePose(a_F_b_T_c, n_T_a);
    EXPECT_MATRIX_NEAR((n_T_a.linear() * a_F_b_T_c.translation()), n_F_b_T_c.translation(), 1e-6);
  }
}

TEST(UtilitiesTester, FrameChangeRelativePoseWithRotation) {
  const Eigen::Isometry3d a_F_b_T_c = lc::Isometry3d(Eigen::Vector3d(0, 0, 1), lc::RotationFromEulerAngles(90, 0, 0));
  // Rotation about the same axis shouldn't change relative rotation
  {
    const Eigen::Isometry3d n_T_a = lc::Isometry3d(Eigen::Vector3d::Zero(), lc::RotationFromEulerAngles(90, 0, 0));
    const auto n_F_b_T_c = lc::FrameChangeRelativePose(a_F_b_T_c, n_T_a);
    EXPECT_MATRIX_NEAR(a_F_b_T_c.linear(), n_F_b_T_c.linear(), 1e-6);
  }
  // Rotation should be the same as n_F_b_R_c * a_F_b_R_c * n_F_b_R_c.inv()
  // The rotation a_F_b_R_c can be thought of as a_R_b * b_R_c*b_R_a,
  // which rotates to b's position in frame a, applies the relative rotation from b to c, then
  // subtracts the rotation from b to a.
  // Thus a frame change n_R_a should be applied as n_R_a* a_R_b * b_R_c*b_R_a*a_R_n
  {
    const Eigen::Isometry3d n_T_a = lc::RandomIsometry3d();
    const auto n_F_b_T_c = lc::FrameChangeRelativePose(a_F_b_T_c, n_T_a);
    EXPECT_MATRIX_NEAR((n_T_a.linear() * a_F_b_T_c.linear() * (n_T_a.linear()).transpose()), n_F_b_T_c.linear(), 1e-6);
  }
  // Rotation magnitude shouldn't be affected by frame change
  for (int i = 0; i < 50; ++i) {
    const Eigen::Isometry3d a_F_b_T_c = lc::RandomIsometry3d();
    const Eigen::Isometry3d n_T_a = lc::RandomIsometry3d();
    const auto n_F_b_T_c = lc::FrameChangeRelativePose(a_F_b_T_c, n_T_a);
    // Sometimes axis can be flipped, take min angle distance so this doesn't matter
    const double angle_a =
      std::min(2 * M_PI - Eigen::AngleAxisd(a_F_b_T_c.linear()).angle(), Eigen::AngleAxisd(a_F_b_T_c.linear()).angle());
    const double angle_b =
      std::min(2 * M_PI - Eigen::AngleAxisd(n_F_b_T_c.linear()).angle(), Eigen::AngleAxisd(n_F_b_T_c.linear()).angle());
    EXPECT_NEAR(angle_a, angle_b, 1e-6);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
