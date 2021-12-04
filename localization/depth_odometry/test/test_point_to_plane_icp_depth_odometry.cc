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
#include <depth_odometry/point_to_plane_icp_depth_odometry.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <point_cloud_common/test_utilities.h>

#include <gtest/gtest.h>

namespace dd = depth_odometry;
namespace lc = localization_common;

TEST(PointToPlaneICPDepthOdometryTester, CubicPoints) {
  const auto source_depth_image_measurement = dd::DefaultDepthImageMeasurement(0);
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_depth_image_measurement =
    dd::TransformDepthImageMeasurement(source_depth_image_measurement, 0.1, target_T_source);
  const auto params = dd::DefaultPointToPlaneICPDepthOdometryParams();
  dd::PointToPlaneICPDepthOdometry icp_depth_odometry(params);
  const auto pose_with_covariances = icp_depth_odometry.DepthImageCallback(source_depth_image_measurement);
  /*ASSERT_TRUE(estimated_target_T_source != boost::none);
  EXPECT_PRED2(lc::MatrixEquality<2>, estimated_target_T_source->pose.matrix(), target_T_source.matrix());*/
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
