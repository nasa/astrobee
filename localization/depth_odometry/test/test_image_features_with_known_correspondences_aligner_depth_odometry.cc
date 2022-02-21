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
#include <depth_odometry/image_features_with_known_correspondences_aligner_depth_odometry.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <point_cloud_common/test_utilities.h>

#include <gtest/gtest.h>

namespace dd = depth_odometry;
namespace lc = localization_common;

TEST(ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryTester, RampedPoints) {
  auto params = dd::DefaultImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams();
  dd::ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry image_features_depth_odometry(params);
  const auto source_depth_image_measurement = dd::ImageFeatureDepthImageMeasurement(0);
  constexpr double translation_stddev = 1;
  constexpr double rotation_stddev = 1;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const cv::Point2i offset(5, 5);
  const auto target_depth_image_measurement =
    dd::OffsetImageFeatureDepthImageMeasurement(0.1, source_depth_image_measurement, offset, target_T_source);
  {
    const auto pose_with_covariance = image_features_depth_odometry.DepthImageCallback(source_depth_image_measurement);
    ASSERT_TRUE(pose_with_covariance == boost::none);
  }
  const auto pose_with_covariance = image_features_depth_odometry.DepthImageCallback(target_depth_image_measurement);
  ASSERT_TRUE(pose_with_covariance != boost::none);
  EXPECT_MATRIX_NEAR(pose_with_covariance->pose_with_covariance.pose, target_T_source.inverse(), 1e-2);
  const auto& correspondences = pose_with_covariance->depth_correspondences;
  for (int i = 0; i < correspondences.source_image_points.size(); ++i) {
    EXPECT_MATRIX_NEAR(correspondences.source_image_points[i],
                       (correspondences.target_image_points[i] - Eigen::Vector2d(offset.x, offset.y)), 1e-2);
    EXPECT_MATRIX_NEAR(target_T_source * correspondences.source_3d_points[i], correspondences.target_3d_points[i],
                       1e-2);
  }
}

TEST(ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryTester, PointToPlaneRampedPoints) {
  auto params = dd::DefaultImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams();
  params.aligner.use_point_to_plane_cost = true;
  params.aligner.normal_search_radius = 3.0;
  params.position_covariance_threshold = 10;
  params.orientation_covariance_threshold = 10;
  dd::ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry image_features_depth_odometry(params);
  const auto source_depth_image_measurement = dd::ImageFeatureDepthImageMeasurement(0);
  constexpr double translation_stddev = 1;
  constexpr double rotation_stddev = 1;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const cv::Point2i offset(5, 5);
  const auto target_depth_image_measurement =
    dd::OffsetImageFeatureDepthImageMeasurement(0.1, source_depth_image_measurement, offset, target_T_source);
  {
    const auto pose_with_covariance = image_features_depth_odometry.DepthImageCallback(source_depth_image_measurement);
    ASSERT_TRUE(pose_with_covariance == boost::none);
  }
  const auto pose_with_covariance = image_features_depth_odometry.DepthImageCallback(target_depth_image_measurement);
  ASSERT_TRUE(pose_with_covariance != boost::none);
  EXPECT_MATRIX_NEAR(pose_with_covariance->pose_with_covariance.pose, target_T_source.inverse(), 1e-2);
  const auto& correspondences = pose_with_covariance->depth_correspondences;
  for (int i = 0; i < correspondences.source_image_points.size(); ++i) {
    EXPECT_MATRIX_NEAR(correspondences.source_image_points[i],
                       (correspondences.target_image_points[i] - Eigen::Vector2d(offset.x, offset.y)), 1e-2);
    EXPECT_MATRIX_NEAR(target_T_source * correspondences.source_3d_points[i], correspondences.target_3d_points[i],
                       1e-2);
  }
}

TEST(ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryTester, SymmetricPointToPlaneRampedPoints) {
  auto params = dd::DefaultImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams();
  params.aligner.use_symmetric_point_to_plane_cost = true;
  params.aligner.normal_search_radius = 3.0;
  params.position_covariance_threshold = 10;
  params.orientation_covariance_threshold = 10;
  dd::ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry image_features_depth_odometry(params);
  const auto source_depth_image_measurement = dd::ImageFeatureDepthImageMeasurement(0);
  constexpr double translation_stddev = 1;
  constexpr double rotation_stddev = 1;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const cv::Point2i offset(5, 5);
  const auto target_depth_image_measurement =
    dd::OffsetImageFeatureDepthImageMeasurement(0.1, source_depth_image_measurement, offset, target_T_source);
  {
    const auto pose_with_covariance = image_features_depth_odometry.DepthImageCallback(source_depth_image_measurement);
    ASSERT_TRUE(pose_with_covariance == boost::none);
  }
  const auto pose_with_covariance = image_features_depth_odometry.DepthImageCallback(target_depth_image_measurement);
  ASSERT_TRUE(pose_with_covariance != boost::none);
  EXPECT_MATRIX_NEAR(pose_with_covariance->pose_with_covariance.pose, target_T_source.inverse(), 1e-2);
  const auto& correspondences = pose_with_covariance->depth_correspondences;
  for (int i = 0; i < correspondences.source_image_points.size(); ++i) {
    EXPECT_MATRIX_NEAR(correspondences.source_image_points[i],
                       (correspondences.target_image_points[i] - Eigen::Vector2d(offset.x, offset.y)), 1e-2);
    EXPECT_MATRIX_NEAR(target_T_source * correspondences.source_3d_points[i], correspondences.target_3d_points[i],
                       1e-2);
  }
}

TEST(ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryTester, UmeyamaRampedPoints) {
  auto params = dd::DefaultImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams();
  params.aligner.use_single_iteration_umeyama = true;
  dd::ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry image_features_depth_odometry(params);
  const auto source_depth_image_measurement = dd::ImageFeatureDepthImageMeasurement(0);
  constexpr double translation_stddev = 1;
  constexpr double rotation_stddev = 1;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const cv::Point2i offset(5, 5);
  const auto target_depth_image_measurement =
    dd::OffsetImageFeatureDepthImageMeasurement(0.1, source_depth_image_measurement, offset, target_T_source);
  {
    const auto pose_with_covariance = image_features_depth_odometry.DepthImageCallback(source_depth_image_measurement);
    ASSERT_TRUE(pose_with_covariance == boost::none);
  }
  const auto pose_with_covariance = image_features_depth_odometry.DepthImageCallback(target_depth_image_measurement);
  ASSERT_TRUE(pose_with_covariance != boost::none);
  EXPECT_MATRIX_NEAR(pose_with_covariance->pose_with_covariance.pose, target_T_source.inverse(), 1e-2);
  const auto& correspondences = pose_with_covariance->depth_correspondences;
  for (int i = 0; i < correspondences.source_image_points.size(); ++i) {
    EXPECT_MATRIX_NEAR(correspondences.source_image_points[i],
                       (correspondences.target_image_points[i] - Eigen::Vector2d(offset.x, offset.y)), 1e-2);
    EXPECT_MATRIX_NEAR(target_T_source * correspondences.source_3d_points[i], correspondences.target_3d_points[i],
                       1e-2);
  }
}

TEST(ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryTester, UmeyamaInitialGeussRampedPoints) {
  auto params = dd::DefaultImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams();
  params.aligner.use_umeyama_initial_guess = true;
  dd::ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry image_features_depth_odometry(params);
  const auto source_depth_image_measurement = dd::ImageFeatureDepthImageMeasurement(0);
  constexpr double translation_stddev = 1;
  constexpr double rotation_stddev = 1;
  const auto target_T_source =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const cv::Point2i offset(5, 5);
  const auto target_depth_image_measurement =
    dd::OffsetImageFeatureDepthImageMeasurement(0.1, source_depth_image_measurement, offset, target_T_source);
  {
    const auto pose_with_covariance = image_features_depth_odometry.DepthImageCallback(source_depth_image_measurement);
    ASSERT_TRUE(pose_with_covariance == boost::none);
  }
  const auto pose_with_covariance = image_features_depth_odometry.DepthImageCallback(target_depth_image_measurement);
  ASSERT_TRUE(pose_with_covariance != boost::none);
  EXPECT_MATRIX_NEAR(pose_with_covariance->pose_with_covariance.pose, target_T_source.inverse(), 1e-2);
  const auto& correspondences = pose_with_covariance->depth_correspondences;
  for (int i = 0; i < correspondences.source_image_points.size(); ++i) {
    EXPECT_MATRIX_NEAR(correspondences.source_image_points[i],
                       (correspondences.target_image_points[i] - Eigen::Vector2d(offset.x, offset.y)), 1e-2);
    EXPECT_MATRIX_NEAR(target_T_source * correspondences.source_3d_points[i], correspondences.target_3d_points[i],
                       1e-2);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
