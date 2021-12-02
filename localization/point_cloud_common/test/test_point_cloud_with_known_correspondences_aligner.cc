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

TEST(PointCloudWithKnownCorrespondencesAlignerTester, PointToPointNoisyInitialEstimateCubicPoints) {
  const auto params = pc::DefaultPointCloudWithKnownCorrespondencesAlignerParams();
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  pc::PointCloudWithKnownCorrespondencesAligner aligner(params);
  for (int i = 0; i < 50; ++i) {
    const auto source_T_points_and_normals = pc::CubicPoints();
    const auto target_T_source = lc::RandomIsometry3d();
    const auto target_T_points_and_normals = lc::TransformPointsWithNormals(
      source_T_points_and_normals.first, source_T_points_and_normals.second, target_T_source);
    const auto noisy_target_T_source = lc::AddNoiseToIsometry3d(target_T_source, translation_stddev, rotation_stddev);
    const auto estimated_target_T_source = aligner.ComputeRelativeTransform(
      source_T_points_and_normals.first, target_T_points_and_normals.first, noisy_target_T_source);
    ASSERT_TRUE(estimated_target_T_source != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_target_T_source->pose.matrix(), target_T_source.matrix());
  }
}

TEST(PointCloudWithKnownCorrespondencesAlignerTester, PointToPointNoisyInitialEstimateRandomPoints) {
  const auto params = pc::DefaultPointCloudWithKnownCorrespondencesAlignerParams();
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  constexpr int num_points = 50;
  pc::PointCloudWithKnownCorrespondencesAligner aligner(params);
  for (int i = 0; i < 50; ++i) {
    const auto source_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto target_T_source = lc::RandomIsometry3d();
    const auto target_T_points_and_normals = lc::TransformPointsWithNormals(
      source_T_points_and_normals.first, source_T_points_and_normals.second, target_T_source);
    const auto noisy_target_T_source = lc::AddNoiseToIsometry3d(target_T_source, translation_stddev, rotation_stddev);
    const auto estimated_target_T_source = aligner.ComputeRelativeTransform(
      source_T_points_and_normals.first, target_T_points_and_normals.first, noisy_target_T_source);
    ASSERT_TRUE(estimated_target_T_source != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_target_T_source->pose.matrix(), target_T_source.matrix());
  }
}

TEST(PointCloudWithKnownCorrespondencesAlignerTester, PointToPlaneNoisyInitialEstimateCubicPoints) {
  auto params = pc::DefaultPointCloudWithKnownCorrespondencesAlignerParams();
  params.use_point_to_plane_cost = true;
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  pc::PointCloudWithKnownCorrespondencesAligner aligner(params);
  for (int i = 0; i < 50; ++i) {
    const auto source_T_points_and_normals = pc::CubicPoints();
    const auto target_T_source = lc::RandomIsometry3d();
    const auto target_T_points_and_normals = lc::TransformPointsWithNormals(
      source_T_points_and_normals.first, source_T_points_and_normals.second, target_T_source);
    const auto noisy_target_T_source = lc::AddNoiseToIsometry3d(target_T_source, translation_stddev, rotation_stddev);
    const auto estimated_target_T_source =
      aligner.ComputeRelativeTransform(source_T_points_and_normals.first, target_T_points_and_normals.first,
                                       boost::none, target_T_points_and_normals.second, noisy_target_T_source);
    ASSERT_TRUE(estimated_target_T_source != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_target_T_source->pose.matrix(), target_T_source.matrix());
  }
}

TEST(PointCloudWithKnownCorrespondencesAlignerTester, PointToPlaneNoisyInitialEstimateRandomPoints) {
  auto params = pc::DefaultPointCloudWithKnownCorrespondencesAlignerParams();
  params.use_point_to_plane_cost = true;
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  constexpr int num_points = 50;
  pc::PointCloudWithKnownCorrespondencesAligner aligner(params);
  for (int i = 0; i < 50; ++i) {
    const auto source_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto target_T_source = lc::RandomIsometry3d();
    const auto target_T_points_and_normals = lc::TransformPointsWithNormals(
      source_T_points_and_normals.first, source_T_points_and_normals.second, target_T_source);
    const auto noisy_target_T_source = lc::AddNoiseToIsometry3d(target_T_source, translation_stddev, rotation_stddev);
    const auto estimated_target_T_source =
      aligner.ComputeRelativeTransform(source_T_points_and_normals.first, target_T_points_and_normals.first,
                                       boost::none, target_T_points_and_normals.second, noisy_target_T_source);
    ASSERT_TRUE(estimated_target_T_source != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_target_T_source->pose.matrix(), target_T_source.matrix());
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
