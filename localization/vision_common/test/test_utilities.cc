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
#include <vision_common/test_utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace vc = vision_common;
TEST(UtilitiesTester, FocalLengths) {
  Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
  const Eigen::Vector2d focal_lengths = lc::RandomVector2d();
  vc::SetFocalLengths(focal_lengths, intrinsics);
  const auto test_focal_lengths = vc::FocalLengths(intrinsics);
  EXPECT_NEAR(test_focal_lengths[0], focal_lengths[0], 1e-6);
  EXPECT_NEAR(test_focal_lengths[1], focal_lengths[1], 1e-6);
}

TEST(UtilitiesTester, PrincipalPoints) {
  Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
  const Eigen::Vector2d principal_points = lc::RandomVector2d();
  vc::SetPrincipalPoints(principal_points, intrinsics);
  const auto test_principal_points = vc::PrincipalPoints(intrinsics);
  EXPECT_NEAR(test_principal_points[0], principal_points[0], 1e-6);
  EXPECT_NEAR(test_principal_points[1], principal_points[1], 1e-6);
}

TEST(UtilitiesTester, IdentityProjection) {
  const Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
  {
    const Eigen::Vector3d cam_t_point(1, 1, 1);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_TRUE(lc::MatrixEquality<6>(projected_point, Eigen::Vector2d(1, 1)));
  }
  {
    const Eigen::Vector3d cam_t_point(1, 1, 2);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_TRUE(lc::MatrixEquality<6>(projected_point, Eigen::Vector2d(0.5, 0.5)));
  }
  {
    const Eigen::Vector3d cam_t_point(4, 4, 2);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_TRUE(lc::MatrixEquality<6>(projected_point, Eigen::Vector2d(2, 2)));
  }
}

TEST(UtilitiesTester, FocalLengthProjection) {
  Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
  const double focal_length = 500;
  const Eigen::Vector2d focal_lengths(focal_length, focal_length);
  vc::SetFocalLengths(focal_lengths, intrinsics);
  {
    const Eigen::Vector3d cam_t_point(1, 1, 1);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_TRUE(lc::MatrixEquality<6>(projected_point, cam_t_point.head<2>() * focal_length));
  }
  {
    const Eigen::Vector3d cam_t_point(1, 1, 2);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_TRUE(lc::MatrixEquality<6>(projected_point, cam_t_point.head<2>() * focal_length / cam_t_point.z()));
  }
  {
    const Eigen::Vector3d cam_t_point(4, 4, 2);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_TRUE(lc::MatrixEquality<6>(projected_point, cam_t_point.head<2>() * focal_length / cam_t_point.z()));
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
