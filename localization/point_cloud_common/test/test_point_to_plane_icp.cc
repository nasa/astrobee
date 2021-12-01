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

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace pc = point_cloud_common;

TEST(PointToPlaneICPTester, PerfectEstimate) {
  const auto params = pc::DefaultPointToPlaneICPParams();
  PointToPlaneICP icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto a_T_points = pc::CubicPoints();
    const auto b_T_a = lc::RandomIsometry3d();
    const auto b_T_points = lc::Transform(a_T_points, b_T_a);
    const auto source_cloud = pc::PointCloud(a_T_points);
    const auto target_cloud = pc::PointCloud(b_T_points);
    const auto estimated_b_T_a = pc::RelativeTransformUmeyama(a_T_points, b_T_points);
    EXPECT_PRED2(lc::MatrixEquality<6>, estimated_b_T_a.matrix(), b_T_a.matrix());
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
