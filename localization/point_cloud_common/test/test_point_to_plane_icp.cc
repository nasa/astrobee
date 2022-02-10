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
#include <point_cloud_common/point_to_plane_icp.h>
#include <point_cloud_common/utilities.h>
#include <point_cloud_common/test_utilities.h>

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
    const auto source_T_points_and_normals = pc::CubicPoints();
    const auto target_T_source = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(source_T_points_and_normals.first, source_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(target_T_source.matrix()));
    const auto noisy_target_T_source = lc::AddNoiseToIsometry3d(target_T_source, translation_stddev, rotation_stddev);
    const auto estimated_target_T_source =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_target_T_source);
    ASSERT_TRUE(estimated_target_T_source != boost::none);
    EXPECT_MATRIX_NEAR(estimated_target_T_source->pose, target_T_source, 1e-2);
  }
}

TEST(PointToPlaneICPTester, NoisyInitialEstimateRandomPoints) {
  const auto params = pc::DefaultPointToPlaneICPParams();
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  constexpr int num_points = 50;
  pc::PointToPlaneICP<pcl::PointNormal> icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto source_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto target_T_source = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(source_T_points_and_normals.first, source_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(target_T_source.matrix()));
    const auto noisy_target_T_source = lc::AddNoiseToIsometry3d(target_T_source, translation_stddev, rotation_stddev);
    const auto estimated_target_T_source =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_target_T_source);
    ASSERT_TRUE(estimated_target_T_source != boost::none);
    EXPECT_MATRIX_NEAR(estimated_target_T_source->pose, target_T_source, 1e-2);
  }
}

TEST(PointToPlaneICPTester, NoisyInitialEstimateRandomPointsCorrespondencesTest) {
  const auto params = pc::DefaultPointToPlaneICPParams();
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  constexpr int num_points = 50;
  pc::PointToPlaneICP<pcl::PointNormal> icp(params);
  for (int i = 0; i < 50; ++i) {
    const auto source_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto target_T_source = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(source_T_points_and_normals.first, source_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(target_T_source.matrix()));
    const auto noisy_target_T_source = lc::AddNoiseToIsometry3d(target_T_source, translation_stddev, rotation_stddev);
    const auto estimated_target_T_source =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_target_T_source);
    ASSERT_TRUE(estimated_target_T_source != boost::none);
    EXPECT_MATRIX_NEAR(estimated_target_T_source->pose, target_T_source, 1e-2);
    const auto correspondences = icp.correspondences();
    ASSERT_TRUE(correspondences != boost::none);
    for (int i = 0; i < correspondences->size(); ++i) {
      const auto& source_point = correspondences->source_points[i];
      const auto& target_point = correspondences->target_points[i];
      const Eigen::Vector3d transformed_source_point = estimated_target_T_source->pose * source_point;
      EXPECT_MATRIX_NEAR(target_point, transformed_source_point, 1e-2);
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
    const auto source_T_points_and_normals = pc::CubicPoints();
    const auto target_T_source = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(source_T_points_and_normals.first, source_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(target_T_source.matrix()));
    const auto noisy_target_T_source = lc::AddNoiseToIsometry3d(target_T_source, translation_stddev, rotation_stddev);
    const auto estimated_target_T_source =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_target_T_source);
    ASSERT_TRUE(estimated_target_T_source != boost::none);
    EXPECT_MATRIX_NEAR(estimated_target_T_source->pose, target_T_source, 1e-2);
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
    const auto source_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto target_T_source = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(source_T_points_and_normals.first, source_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(target_T_source.matrix()));
    const auto noisy_target_T_source = lc::AddNoiseToIsometry3d(target_T_source, translation_stddev, rotation_stddev);
    const auto estimated_target_T_source =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_target_T_source);
    ASSERT_TRUE(estimated_target_T_source != boost::none);
    EXPECT_MATRIX_NEAR(estimated_target_T_source->pose, target_T_source, 1e-2);
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
    const auto source_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto target_T_source = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(source_T_points_and_normals.first, source_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(target_T_source.matrix()));
    const auto noisy_target_T_source = lc::AddNoiseToIsometry3d(target_T_source, translation_stddev, rotation_stddev);
    const auto estimated_target_T_source =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_target_T_source);
    ASSERT_TRUE(estimated_target_T_source != boost::none);
    EXPECT_MATRIX_NEAR(estimated_target_T_source->pose, target_T_source, 1e-2);
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
    const auto source_T_points_and_normals = pc::RandomPointsWithNormals(num_points);
    const auto target_T_source = lc::RandomIsometry3d();
    const auto source_cloud_with_normals =
      pc::PointCloudWithNormals(source_T_points_and_normals.first, source_T_points_and_normals.second);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*source_cloud_with_normals, *target_cloud_with_normals,
                                        Eigen::Affine3d(target_T_source.matrix()));
    const auto noisy_target_T_source = lc::AddNoiseToIsometry3d(target_T_source, translation_stddev, rotation_stddev);
    const auto estimated_target_T_source =
      icp.ComputeRelativeTransform(source_cloud_with_normals, target_cloud_with_normals, noisy_target_T_source);
    ASSERT_TRUE(estimated_target_T_source != boost::none);
    EXPECT_MATRIX_NEAR(estimated_target_T_source->pose, target_T_source, 1e-2);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
