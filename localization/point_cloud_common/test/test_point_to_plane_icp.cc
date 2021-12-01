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
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <point_cloud_common/point_to_plane_icp.h>
#include <point_cloud_common/utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace pc = point_cloud_common;

TEST(PointToPlaneICPTester, PerfectEstimate) {
  const auto params = pc::DefaultPointToPlaneICPParams();
  constexpr double search_radius = 0.03;
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  pc::PointToPlaneICP<pcl::PointNormal> icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto a_T_points = pc::CubicPoints();
    const auto b_T_a = lc::RandomIsometry3d();
    const auto b_T_points = lc::Transform(a_T_points, b_T_a);
    const auto source_cloud = pc::PointCloud(a_T_points);
    const auto source_cloud_with_normals =
      pc::FilteredPointCloudWithNormals<pcl::PointXYZ, pcl::PointNormal>(source_cloud, search_radius);
    const auto target_cloud = pc::PointCloud(b_T_points);
    const auto target_cloud_with_normals =
      pc::FilteredPointCloudWithNormals<pcl::PointXYZ, pcl::PointNormal>(target_cloud, search_radius);
    const auto noisy_b_T_a = lc::AddNoiseToIsometry3d(b_T_a, translation_stddev, rotation_stddev);
    const auto estimated_b_T_a =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_b_T_a);
    // const auto estimated_b_T_a = icp.ComputeRelativeTransform(source_cloud_with_normals, source_cloud_with_normals,
    // noisy_b_T_a);
    ASSERT_TRUE(estimated_b_T_a != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<6>, estimated_b_T_a->pose.matrix(), b_T_a.matrix());
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
