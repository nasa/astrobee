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
  const auto params = dd::DefaultDepthOdometryWrapperParams();
  dd::DepthOdometryWrapper depth_odometry_wrapper(params);
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  // Add first measurement set
  const lc::Time source_timestamp = 0;
  const auto source_points_msg = dd::CubicPointsMsg(source_timestamp);
  const auto source_image_msg = dd::ImageMsg(source_timestamp);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(source_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(source_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  // Add second measurement set
  const lc::Time target_timestamp = 0.1;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_points_msg = dd::TransformPointsMsg(target_timestamp, source_points_msg, target_T_source);
  const auto target_image_msg = dd::ImageMsg(target_timestamp);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    // TODO(rsoussan): check msg result!
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
