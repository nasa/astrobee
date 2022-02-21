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

TEST(PointToPlaneICPDepthOdometryTester, PointToPlaneCubicPoints) {
  auto params = dd::DefaultPointToPlaneICPDepthOdometryParams();
  params.icp.search_radius = 1.0;
  dd::PointToPlaneICPDepthOdometry icp_depth_odometry(params);

  const auto source_depth_image_measurement = dd::DefaultDepthImageMeasurement(0);
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_depth_image_measurement =
    dd::TransformDepthImageMeasurement(source_depth_image_measurement, 0.1, target_T_source);
  {
    const auto pose_with_covariance = icp_depth_odometry.DepthImageCallback(source_depth_image_measurement);
    ASSERT_TRUE(pose_with_covariance == boost::none);
  }
  const auto pose_with_covariance = icp_depth_odometry.DepthImageCallback(target_depth_image_measurement);
  ASSERT_TRUE(pose_with_covariance != boost::none);
  EXPECT_MATRIX_NEAR(pose_with_covariance->pose_with_covariance.pose, target_T_source.inverse(), 1e-4);
  const auto& correspondences = pose_with_covariance->depth_correspondences;
  for (int i = 0; i < correspondences.source_image_points.size(); ++i) {
    EXPECT_MATRIX_NEAR(target_T_source * correspondences.source_3d_points[i], correspondences.target_3d_points[i],
                       1e-2);
  }
}

TEST(PointToPlaneICPDepthOdometryTester, PointToPlaneDownsampledCubicPoints) {
  auto params = dd::DefaultPointToPlaneICPDepthOdometryParams();
  params.icp.search_radius = 1.0;
  params.downsample = true;
  params.downsample_leaf_size = 0.03;
  dd::PointToPlaneICPDepthOdometry icp_depth_odometry(params);

  const auto source_depth_image_measurement = dd::DefaultDepthImageMeasurement(0);
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_depth_image_measurement =
    dd::TransformDepthImageMeasurement(source_depth_image_measurement, 0.1, target_T_source);
  {
    const auto pose_with_covariance = icp_depth_odometry.DepthImageCallback(source_depth_image_measurement);
    ASSERT_TRUE(pose_with_covariance == boost::none);
  }
  const auto pose_with_covariance = icp_depth_odometry.DepthImageCallback(target_depth_image_measurement);
  ASSERT_TRUE(pose_with_covariance != boost::none);
  EXPECT_MATRIX_NEAR(pose_with_covariance->pose_with_covariance.pose, target_T_source.inverse(), 1e-4);
}

TEST(PointToPlaneICPDepthOdometryTester, SymmetricPointToPlaneCubicPoints) {
  auto params = dd::DefaultPointToPlaneICPDepthOdometryParams();
  params.icp.search_radius = 1.0;
  params.icp.symmetric_objective = true;
  dd::PointToPlaneICPDepthOdometry icp_depth_odometry(params);

  const auto source_depth_image_measurement = dd::DefaultDepthImageMeasurement(0);
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_depth_image_measurement =
    dd::TransformDepthImageMeasurement(source_depth_image_measurement, 0.1, target_T_source);
  {
    const auto pose_with_covariance = icp_depth_odometry.DepthImageCallback(source_depth_image_measurement);
    ASSERT_TRUE(pose_with_covariance == boost::none);
  }
  const auto pose_with_covariance = icp_depth_odometry.DepthImageCallback(target_depth_image_measurement);
  ASSERT_TRUE(pose_with_covariance != boost::none);
  EXPECT_MATRIX_NEAR(pose_with_covariance->pose_with_covariance.pose, target_T_source.inverse(), 1e-4);
}

TEST(PointToPlaneICPDepthOdometryTester, CorrespondenceRejectorPointToPlaneCubicPoints) {
  auto params = dd::DefaultPointToPlaneICPDepthOdometryParams();
  params.icp.search_radius = 1.0;
  params.icp.correspondence_rejector_surface_normal = true;
  dd::PointToPlaneICPDepthOdometry icp_depth_odometry(params);

  const auto source_depth_image_measurement = dd::DefaultDepthImageMeasurement(0);
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_depth_image_measurement =
    dd::TransformDepthImageMeasurement(source_depth_image_measurement, 0.1, target_T_source);
  {
    const auto pose_with_covariance = icp_depth_odometry.DepthImageCallback(source_depth_image_measurement);
    ASSERT_TRUE(pose_with_covariance == boost::none);
  }
  const auto pose_with_covariance = icp_depth_odometry.DepthImageCallback(target_depth_image_measurement);
  ASSERT_TRUE(pose_with_covariance != boost::none);
  EXPECT_MATRIX_NEAR(pose_with_covariance->pose_with_covariance.pose, target_T_source.inverse(), 1e-4);
}

TEST(PointToPlaneICPDepthOdometryTester, PointToPlane3MeasurementsCubicPoints) {
  auto params = dd::DefaultPointToPlaneICPDepthOdometryParams();
  params.icp.search_radius = 1.0;
  dd::PointToPlaneICPDepthOdometry icp_depth_odometry(params);

  const auto source_depth_image_measurement = dd::DefaultDepthImageMeasurement(0);
  constexpr double translation_stddev = 0.01;
  constexpr double rotation_stddev = 0.01;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_depth_image_measurement =
    dd::TransformDepthImageMeasurement(source_depth_image_measurement, 0.1, target_T_source);
  {
    const auto pose_with_covariance = icp_depth_odometry.DepthImageCallback(source_depth_image_measurement);
    ASSERT_TRUE(pose_with_covariance == boost::none);
  }
  const auto pose_with_covariance = icp_depth_odometry.DepthImageCallback(target_depth_image_measurement);
  ASSERT_TRUE(pose_with_covariance != boost::none);
  EXPECT_MATRIX_NEAR(pose_with_covariance->pose_with_covariance.pose, target_T_source.inverse(), 1e-4);

  // Add third measurement
  const auto target2_T_target =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target2_depth_image_measurement =
    dd::TransformDepthImageMeasurement(target_depth_image_measurement, 0.2, target2_T_target);
  const auto pose_with_covariance2 = icp_depth_odometry.DepthImageCallback(target2_depth_image_measurement);
  ASSERT_TRUE(pose_with_covariance2 != boost::none);
  EXPECT_MATRIX_NEAR(pose_with_covariance2->pose_with_covariance.pose, target2_T_target.inverse(), 1e-4);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
