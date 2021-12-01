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

#include <pcl/common/transforms.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace pc = point_cloud_common;

TEST(PointToPlaneICPTester, NoisyInitialEstimateCubicPoints) {
  const auto params = pc::DefaultPointToPlaneICPParams();
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  pc::PointToPlaneICP<pcl::PointNormal> icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto a_T_points_and_normals = pc::CubicPoints();
    const auto b_T_a = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(a_T_points_and_normals.first, a_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(b_T_a.matrix()));
    const auto noisy_b_T_a = lc::AddNoiseToIsometry3d(b_T_a, translation_stddev, rotation_stddev);
    const auto estimated_a_T_b =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_b_T_a);
    ASSERT_TRUE(estimated_a_T_b != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_a_T_b->pose.matrix(), b_T_a.inverse().matrix());
  }
}

TEST(PointToPlaneICPTester, NoisyInitialEstimateRandomPoints) {
  const auto params = pc::DefaultPointToPlaneICPParams();
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  constexpr int num_points = 50;
  pc::PointToPlaneICP<pcl::PointNormal> icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto a_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto b_T_a = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(a_T_points_and_normals.first, a_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(b_T_a.matrix()));
    const auto noisy_b_T_a = lc::AddNoiseToIsometry3d(b_T_a, translation_stddev, rotation_stddev);
    const auto estimated_a_T_b =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_b_T_a);
    ASSERT_TRUE(estimated_a_T_b != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_a_T_b->pose.matrix(), b_T_a.inverse().matrix());
  }
}

TEST(PointToPlaneICPTester, NoisyInitialEstimateRandomPointsCorrespondencesTest) {
  const auto params = pc::DefaultPointToPlaneICPParams();
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  constexpr int num_points = 50;
  pc::PointToPlaneICP<pcl::PointNormal> icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto a_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto b_T_a = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(a_T_points_and_normals.first, a_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(b_T_a.matrix()));
    const auto noisy_b_T_a = lc::AddNoiseToIsometry3d(b_T_a, translation_stddev, rotation_stddev);
    const auto estimated_a_T_b =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_b_T_a);
    ASSERT_TRUE(estimated_a_T_b != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_a_T_b->pose.matrix(), b_T_a.inverse().matrix());
    const auto correspondences = icp.correspondences();
    ASSERT_TRUE(correspondences != boost::none);
    for (int i = 0; i < correspondences->size(); ++i) {
      const auto& source_point = correspondences->source_points[i];
      const auto& target_point = correspondences->target_points[i];
      const Eigen::Vector3d transformed_target_point = estimated_a_T_b->pose * target_point;
      EXPECT_PRED2(lc::MatrixEquality<2>, source_point.matrix(), transformed_target_point.matrix());
    }
  }
}

TEST(PointToPlaneICPTester, NoisyInitialEstimateSymmetricCostCubicPoints) {
  auto params = pc::DefaultPointToPlaneICPParams();
  params.symmetric_objective = true;
  params.enforce_same_direction_normals = true;
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  pc::PointToPlaneICP<pcl::PointNormal> icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto a_T_points_and_normals = pc::CubicPoints();
    const auto b_T_a = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(a_T_points_and_normals.first, a_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(b_T_a.matrix()));
    const auto noisy_b_T_a = lc::AddNoiseToIsometry3d(b_T_a, translation_stddev, rotation_stddev);
    const auto estimated_a_T_b =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_b_T_a);
    ASSERT_TRUE(estimated_a_T_b != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_a_T_b->pose.matrix(), b_T_a.inverse().matrix());
  }
}

TEST(PointToPlaneICPTester, NoisyInitialEstimateSymmetricCostRandomPoints) {
  auto params = pc::DefaultPointToPlaneICPParams();
  params.symmetric_objective = true;
  params.enforce_same_direction_normals = true;
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  constexpr int num_points = 50;
  pc::PointToPlaneICP<pcl::PointNormal> icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto a_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto b_T_a = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(a_T_points_and_normals.first, a_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(b_T_a.matrix()));
    const auto noisy_b_T_a = lc::AddNoiseToIsometry3d(b_T_a, translation_stddev, rotation_stddev);
    const auto estimated_a_T_b =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_b_T_a);
    ASSERT_TRUE(estimated_a_T_b != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_a_T_b->pose.matrix(), b_T_a.inverse().matrix());
  }
}

TEST(PointToPlaneICPTester, NoisyInitialEstimateCorrespondenceRejectorRandomPoints) {
  auto params = pc::DefaultPointToPlaneICPParams();
  params.correspondence_rejector_surface_normal = true;
  params.correspondence_rejector_surface_normal_threshold = 0.75;
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  constexpr int num_points = 50;
  pc::PointToPlaneICP<pcl::PointNormal> icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto a_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto b_T_a = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(a_T_points_and_normals.first, a_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(b_T_a.matrix()));
    const auto noisy_b_T_a = lc::AddNoiseToIsometry3d(b_T_a, translation_stddev, rotation_stddev);
    const auto estimated_a_T_b =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_b_T_a);
    ASSERT_TRUE(estimated_a_T_b != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_a_T_b->pose.matrix(), b_T_a.inverse().matrix());
  }
}

TEST(PointToPlaneICPTester, NoisyInitialEstimateDownsampledRandomPoints) {
  auto params = pc::DefaultPointToPlaneICPParams();
  params.downsample = true;
  params.downsample_leaf_size = 0.3;
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  constexpr int num_points = 50;
  pc::PointToPlaneICP<pcl::PointNormal> icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto a_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto b_T_a = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(a_T_points_and_normals.first, a_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(b_T_a.matrix()));
    const auto noisy_b_T_a = lc::AddNoiseToIsometry3d(b_T_a, translation_stddev, rotation_stddev);
    const auto estimated_a_T_b =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_b_T_a);
    ASSERT_TRUE(estimated_a_T_b != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_a_T_b->pose.matrix(), b_T_a.inverse().matrix());
  }
}

TEST(PointToPlaneICPTester, NoisyInitialEstimateCoarseToFineRandomPoints) {
  auto params = pc::DefaultPointToPlaneICPParams();
  params.coarse_to_fine = true;
  params.num_coarse_to_fine_levels = 3;
  params.coarse_to_fine_final_leaf_size = 0.3;
  params.downsample_last_coarse_to_fine_iteration = true;
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  constexpr int num_points = 50;
  pc::PointToPlaneICP<pcl::PointNormal> icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto a_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto b_T_a = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(a_T_points_and_normals.first, a_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(b_T_a.matrix()));
    const auto noisy_b_T_a = lc::AddNoiseToIsometry3d(b_T_a, translation_stddev, rotation_stddev);
    const auto estimated_a_T_b =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_b_T_a);
    ASSERT_TRUE(estimated_a_T_b != boost::none);
    EXPECT_PRED2(lc::MatrixEquality<2>, estimated_a_T_b->pose.matrix(), b_T_a.inverse().matrix());
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
