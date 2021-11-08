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
    ASSERT_TRUE(rotation.matrix().isApprox(rotation_again.matrix(), 1e-6));
  }
}
