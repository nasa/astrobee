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
#include <localization_common/pose_interpolater.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;

TEST(PoseInterpolaterTester, Interpolate) {
  std::vector<Eigen::Isometry3d> poses;
  std::vector<lc::Time> timestamps;
  for (int i = 0; i < 10; ++i) {
    poses.emplace_back(lc::RandomIsometry3d());
    timestamps.emplace_back(i);
  }
  lc::PoseInterpolater interpolater(timestamps, poses);

  // Too low
  {
    const auto interpolated_pose = interpolater.Interpolate(-1);
    EXPECT_TRUE(interpolated_pose == boost::none);
  }
  // Too high
  {
    const auto interpolated_pose = interpolater.Interpolate(11);
    EXPECT_TRUE(interpolated_pose == boost::none);
  }
  // Valid
  {
    const auto interpolated_pose = interpolater.Interpolate(3.3);
    ASSERT_TRUE(interpolated_pose != boost::none);
    const auto expected_interpolated_pose = lc::Interpolate(poses[3], poses[4], 0.3);
    EXPECT_MATRIX_NEAR((*interpolated_pose), expected_interpolated_pose, 1e-6);
  }
  // Exact pose/timestamp
  {
    const auto interpolated_pose = interpolater.Interpolate(8);
    ASSERT_TRUE(interpolated_pose != boost::none);
    EXPECT_MATRIX_NEAR((*interpolated_pose), poses[8], 1e-6);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
