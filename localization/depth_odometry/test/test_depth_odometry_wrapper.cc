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
#include <depth_odometry/depth_odometry_wrapper.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <point_cloud_common/test_utilities.h>

#include <gtest/gtest.h>

namespace dd = depth_odometry;
namespace lc = localization_common;

TEST(DepthOdometryWrapperTester, A) {
  const lc::Time source_timestamp = 0;
  const auto points_msg = dd::CubicPointsMsg(source_timestamp);
  const auto image_msg = dd::ImageMsg(source_timestamp);
  const auto params = dd::DefaultDepthOdometryWrapperParams();
  dd::DepthOdometryWrapper depth_odometry_wrapper(params);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
