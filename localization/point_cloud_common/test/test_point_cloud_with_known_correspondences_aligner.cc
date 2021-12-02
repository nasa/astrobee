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
#include <point_cloud_common/point_cloud_with_known_correspondences_aligner.h>
#include <point_cloud_common/utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace pc = point_cloud_common;

/*TEST(PointToPlaneICPTester, NoisyInitialEstimateCubicPoints) {
  const auto params = pc::DefaultPointCloudWithKnownCorrespondencesAlignerParams();
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  pc::PointCloudWithKnownCorrespondencesAligner aligner(params);
  for (int i = 0; i < 50; ++i) {
    const auto source_T_points_and_normals = pc::CubicPoints();
    const auto source_T_target = lc::RandomIsometry3d();
    const auto noisy_source_T_target = lc::AddNoiseToIsometry3d(source_T_target, translation_stddev, rotation_stddev);
    const auto estimated_source_T_target =
      aligner.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_source_T_target);
    ASSERT_TRUE(estimated_source_T_target != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_source_T_target->pose.matrix(), source_T_target.matrix());
  }
}*/

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
