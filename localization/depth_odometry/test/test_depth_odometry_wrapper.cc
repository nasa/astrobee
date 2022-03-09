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
#include <localization_common/utilities.h>
#include <point_cloud_common/test_utilities.h>

#include <gtest/gtest.h>

namespace dd = depth_odometry;
namespace lc = localization_common;

TEST(DepthOdometryWrapperTester, PointToPlaneICP) {
  auto params = dd::DefaultDepthOdometryWrapperParams();
  params.icp.icp.search_radius = 1.0;
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
    const auto sensor_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_source_T_target, target_T_source.inverse(), 1e-4);
    const auto body_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_source_T_target =
      params.body_T_haz_cam * target_T_source.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_source_T_target, true_body_F_source_T_target, 1e-4);
  }
  // Add third measurement set
  const lc::Time target_2_timestamp = 0.1;
  const auto target_2_T_target =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_2_points_msg = dd::TransformPointsMsg(target_2_timestamp, target_points_msg, target_2_T_target);
  const auto target_2_image_msg = dd::ImageMsg(target_2_timestamp);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_2_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_2_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_target_T_target_2, target_2_T_target.inverse(), 1e-4);
    const auto body_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_target_T_target_2 =
      params.body_T_haz_cam * target_2_T_target.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_target_T_target_2, true_body_F_target_T_target_2, 1e-4);
  }
}

TEST(DepthOdometryWrapperTester, SymmetricPointToPlaneICP) {
  auto params = dd::DefaultDepthOdometryWrapperParams();
  params.icp.icp.search_radius = 1.0;
  params.icp.icp.symmetric_objective = true;
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
    const auto sensor_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_source_T_target, target_T_source.inverse(), 1e-4);
    const auto body_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_source_T_target =
      params.body_T_haz_cam * target_T_source.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_source_T_target, true_body_F_source_T_target, 1e-4);
  }
  // Add third measurement set
  const lc::Time target_2_timestamp = 0.1;
  const auto target_2_T_target =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_2_points_msg = dd::TransformPointsMsg(target_2_timestamp, target_points_msg, target_2_T_target);
  const auto target_2_image_msg = dd::ImageMsg(target_2_timestamp);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_2_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_2_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_target_T_target_2, target_2_T_target.inverse(), 1e-4);
    const auto body_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_target_T_target_2 =
      params.body_T_haz_cam * target_2_T_target.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_target_T_target_2, true_body_F_target_T_target_2, 1e-4);
  }
}

TEST(DepthOdometryWrapperTester, CorrespondenceRejectorSymmetricPointToPlaneICP) {
  auto params = dd::DefaultDepthOdometryWrapperParams();
  params.icp.icp.search_radius = 1.0;
  params.icp.icp.symmetric_objective = true;
  params.icp.icp.correspondence_rejector_surface_normal = true;
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
    const auto sensor_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_source_T_target, target_T_source.inverse(), 1e-4);
    const auto body_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_source_T_target =
      params.body_T_haz_cam * target_T_source.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_source_T_target, true_body_F_source_T_target, 1e-4);
  }
  // Add third measurement set
  const lc::Time target_2_timestamp = 0.1;
  const auto target_2_T_target =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_2_points_msg = dd::TransformPointsMsg(target_2_timestamp, target_points_msg, target_2_T_target);
  const auto target_2_image_msg = dd::ImageMsg(target_2_timestamp);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_2_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_2_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_target_T_target_2, target_2_T_target.inverse(), 1e-4);
    const auto body_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_target_T_target_2 =
      params.body_T_haz_cam * target_2_T_target.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_target_T_target_2, true_body_F_target_T_target_2, 1e-4);
  }
}

TEST(DepthOdometryWrapperTester, ImageFeatureAligner) {
  auto params = dd::DefaultDepthOdometryWrapperParams();
  params.method = "image_feature";
  dd::DepthOdometryWrapper depth_odometry_wrapper(params);
  constexpr double translation_stddev = 1;
  constexpr double rotation_stddev = 1;
  const cv::Point2i offset(5, 5);
  // Add first measurement set
  const lc::Time source_timestamp = 0;
  const auto source_points_msg = dd::RampedPointsMsg(source_timestamp);
  const auto source_image_msg = dd::MarkerImageMsg(source_timestamp);
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
  const auto target_points_msg =
    dd::OffsetAndTransformPointsMsg(target_timestamp, source_points_msg, offset, target_T_source);
  const auto target_image_msg = dd::MarkerImageMsg(target_timestamp, offset);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_source_T_target, target_T_source.inverse(), 1e-2);
    const auto body_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_source_T_target =
      params.body_T_haz_cam * target_T_source.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_source_T_target, true_body_F_source_T_target, 1e-2);
  }
  // Add third measurement set
  const lc::Time target_2_timestamp = 0.1;
  const auto target_2_T_target =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_2_points_msg =
    dd::OffsetAndTransformPointsMsg(target_2_timestamp, target_points_msg, offset, target_2_T_target);
  const auto target_2_image_msg = dd::MarkerImageMsg(target_2_timestamp, offset * 2);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_2_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_2_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_target_T_target_2, target_2_T_target.inverse(), 1e-2);
    const auto body_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_target_T_target_2 =
      params.body_T_haz_cam * target_2_T_target.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_target_T_target_2, true_body_F_target_T_target_2, 1e-2);
  }
}

TEST(DepthOdometryWrapperTester, PointToPlaneImageFeatureAligner) {
  auto params = dd::DefaultDepthOdometryWrapperParams();
  params.method = "image_feature";
  params.image_features.aligner.use_point_to_plane_cost = true;
  params.image_features.aligner.normal_search_radius = 3.0;
  params.image_features.position_covariance_threshold = 10;
  params.image_features.orientation_covariance_threshold = 10;
  dd::DepthOdometryWrapper depth_odometry_wrapper(params);
  constexpr double translation_stddev = 1;
  constexpr double rotation_stddev = 1;
  const cv::Point2i offset(5, 5);
  // Add first measurement set
  const lc::Time source_timestamp = 0;
  const auto source_points_msg = dd::RampedPointsMsg(source_timestamp);
  const auto source_image_msg = dd::MarkerImageMsg(source_timestamp);
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
  const auto target_points_msg =
    dd::OffsetAndTransformPointsMsg(target_timestamp, source_points_msg, offset, target_T_source);
  const auto target_image_msg = dd::MarkerImageMsg(target_timestamp, offset);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_source_T_target, target_T_source.inverse(), 1e-2);
    const auto body_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_source_T_target =
      params.body_T_haz_cam * target_T_source.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_source_T_target, true_body_F_source_T_target, 1e-2);
  }
  // Add third measurement set
  const lc::Time target_2_timestamp = 0.1;
  const auto target_2_T_target =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_2_points_msg =
    dd::OffsetAndTransformPointsMsg(target_2_timestamp, target_points_msg, offset, target_2_T_target);
  const auto target_2_image_msg = dd::MarkerImageMsg(target_2_timestamp, offset * 2);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_2_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_2_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_target_T_target_2, target_2_T_target.inverse(), 1e-2);
    const auto body_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_target_T_target_2 =
      params.body_T_haz_cam * target_2_T_target.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_target_T_target_2, true_body_F_target_T_target_2, 1e-2);
  }
}

TEST(DepthOdometryWrapperTester, SymmetricPointToPlaneImageFeatureAligner) {
  auto params = dd::DefaultDepthOdometryWrapperParams();
  params.method = "image_feature";
  params.image_features.aligner.use_symmetric_point_to_plane_cost = true;
  params.image_features.aligner.normal_search_radius = 3.0;
  params.image_features.position_covariance_threshold = 10;
  params.image_features.orientation_covariance_threshold = 10;
  dd::DepthOdometryWrapper depth_odometry_wrapper(params);
  constexpr double translation_stddev = 1;
  constexpr double rotation_stddev = 1;
  const cv::Point2i offset(5, 5);
  // Add first measurement set
  const lc::Time source_timestamp = 0;
  const auto source_points_msg = dd::RampedPointsMsg(source_timestamp);
  const auto source_image_msg = dd::MarkerImageMsg(source_timestamp);
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
  const auto target_points_msg =
    dd::OffsetAndTransformPointsMsg(target_timestamp, source_points_msg, offset, target_T_source);
  const auto target_image_msg = dd::MarkerImageMsg(target_timestamp, offset);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_source_T_target, target_T_source.inverse(), 1e-2);
    const auto body_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_source_T_target =
      params.body_T_haz_cam * target_T_source.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_source_T_target, true_body_F_source_T_target, 1e-2);
  }
  // Add third measurement set
  const lc::Time target_2_timestamp = 0.1;
  const auto target_2_T_target =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_2_points_msg =
    dd::OffsetAndTransformPointsMsg(target_2_timestamp, target_points_msg, offset, target_2_T_target);
  const auto target_2_image_msg = dd::MarkerImageMsg(target_2_timestamp, offset * 2);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_2_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_2_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_target_T_target_2, target_2_T_target.inverse(), 1e-2);
    const auto body_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_target_T_target_2 =
      params.body_T_haz_cam * target_2_T_target.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_target_T_target_2, true_body_F_target_T_target_2, 1e-2);
  }
}

TEST(DepthOdometryWrapperTester, SingleIterationUmeyamaImageFeatureAligner) {
  auto params = dd::DefaultDepthOdometryWrapperParams();
  params.method = "image_feature";
  params.image_features.aligner.use_single_iteration_umeyama = true;
  dd::DepthOdometryWrapper depth_odometry_wrapper(params);
  constexpr double translation_stddev = 1;
  constexpr double rotation_stddev = 1;
  const cv::Point2i offset(5, 5);
  // Add first measurement set
  const lc::Time source_timestamp = 0;
  const auto source_points_msg = dd::RampedPointsMsg(source_timestamp);
  const auto source_image_msg = dd::MarkerImageMsg(source_timestamp);
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
  const auto target_points_msg =
    dd::OffsetAndTransformPointsMsg(target_timestamp, source_points_msg, offset, target_T_source);
  const auto target_image_msg = dd::MarkerImageMsg(target_timestamp, offset);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_source_T_target, target_T_source.inverse(), 1e-2);
    const auto body_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_source_T_target =
      params.body_T_haz_cam * target_T_source.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_source_T_target, true_body_F_source_T_target, 1e-2);
  }
  // Add third measurement set
  const lc::Time target_2_timestamp = 0.1;
  const auto target_2_T_target =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_2_points_msg =
    dd::OffsetAndTransformPointsMsg(target_2_timestamp, target_points_msg, offset, target_2_T_target);
  const auto target_2_image_msg = dd::MarkerImageMsg(target_2_timestamp, offset * 2);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_2_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_2_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_target_T_target_2, target_2_T_target.inverse(), 1e-2);
    const auto body_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_target_T_target_2 =
      params.body_T_haz_cam * target_2_T_target.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_target_T_target_2, true_body_F_target_T_target_2, 1e-2);
  }
}

TEST(DepthOdometryWrapperTester, UmeyamaInitialGuessImageFeatureAligner) {
  auto params = dd::DefaultDepthOdometryWrapperParams();
  params.method = "image_feature";
  params.image_features.aligner.use_umeyama_initial_guess = true;
  dd::DepthOdometryWrapper depth_odometry_wrapper(params);
  constexpr double translation_stddev = 1;
  constexpr double rotation_stddev = 1;
  const cv::Point2i offset(5, 5);
  // Add first measurement set
  const lc::Time source_timestamp = 0;
  const auto source_points_msg = dd::RampedPointsMsg(source_timestamp);
  const auto source_image_msg = dd::MarkerImageMsg(source_timestamp);
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
  const auto target_points_msg =
    dd::OffsetAndTransformPointsMsg(target_timestamp, source_points_msg, offset, target_T_source);
  const auto target_image_msg = dd::MarkerImageMsg(target_timestamp, offset);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_source_T_target, target_T_source.inverse(), 1e-2);
    const auto body_F_source_T_target =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_source_T_target =
      params.body_T_haz_cam * target_T_source.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_source_T_target, true_body_F_source_T_target, 1e-2);
  }
  // Add third measurement set
  const lc::Time target_2_timestamp = 0.1;
  const auto target_2_T_target =
    lc::AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
  const auto target_2_points_msg =
    dd::OffsetAndTransformPointsMsg(target_2_timestamp, target_points_msg, offset, target_2_T_target);
  const auto target_2_image_msg = dd::MarkerImageMsg(target_2_timestamp, offset * 2);
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.PointCloudCallback(target_2_points_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 0);
  }
  {
    const auto depth_odometry_msgs = depth_odometry_wrapper.ImageCallback(target_2_image_msg);
    ASSERT_EQ(depth_odometry_msgs.size(), 1);
    const auto sensor_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.sensor_F_source_T_target.pose));
    EXPECT_MATRIX_NEAR(sensor_F_target_T_target_2, target_2_T_target.inverse(), 1e-2);
    const auto body_F_target_T_target_2 =
      lc::EigenPose(lc::PoseFromMsg(depth_odometry_msgs[0].odometry.body_F_source_T_target.pose));
    const Eigen::Isometry3d true_body_F_target_T_target_2 =
      params.body_T_haz_cam * target_2_T_target.inverse() * (params.body_T_haz_cam).inverse();
    EXPECT_MATRIX_NEAR(body_F_target_T_target_2, true_body_F_target_T_target_2, 1e-2);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
