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

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/NoiseModel.h>

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
    EXPECT_MATRIX_NEAR(projected_point, Eigen::Vector2d(1, 1), 1e-6);
  }
  {
    const Eigen::Vector3d cam_t_point(1, 1, 2);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_MATRIX_NEAR(projected_point, Eigen::Vector2d(0.5, 0.5), 1e-6);
  }
  {
    const Eigen::Vector3d cam_t_point(4, 4, 2);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_MATRIX_NEAR(projected_point, Eigen::Vector2d(2, 2), 1e-6);
  }
}

TEST(UtilitiesTester, FocalLengthProjection) {
  Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
  const Eigen::Vector2d focal_lengths(500, 505);
  vc::SetFocalLengths(focal_lengths, intrinsics);
  {
    const Eigen::Vector3d cam_t_point(1, 1, 1);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_MATRIX_NEAR(projected_point, cam_t_point.head<2>().cwiseProduct(focal_lengths), 1e-6);
  }
  {
    const Eigen::Vector3d cam_t_point(1, 1, 2);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_MATRIX_NEAR(projected_point, (cam_t_point.head<2>().cwiseProduct(focal_lengths) / cam_t_point.z()), 1e-6);
  }
  {
    const Eigen::Vector3d cam_t_point(4, 4, 2);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_MATRIX_NEAR(projected_point, (cam_t_point.head<2>().cwiseProduct(focal_lengths) / cam_t_point.z()), 1e-6);
  }
}

TEST(UtilitiesTester, PrincipalPointProjection) {
  Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
  const Eigen::Vector2d principal_points(50, 53.5);
  vc::SetPrincipalPoints(principal_points, intrinsics);
  {
    const Eigen::Vector3d cam_t_point(1, 1, 1);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_MATRIX_NEAR(projected_point, (cam_t_point.head<2>() + principal_points), 1e-6);
  }
  {
    const Eigen::Vector3d cam_t_point(1, 1, 2);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_MATRIX_NEAR(projected_point, (cam_t_point.head<2>() / cam_t_point.z() + principal_points), 1e-6);
  }
  {
    const Eigen::Vector3d cam_t_point(4, 4, 2);
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    EXPECT_MATRIX_NEAR(projected_point, (cam_t_point.head<2>() / cam_t_point.z() + principal_points), 1e-6);
  }
}

TEST(UtilitiesTester, ProjectionJacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 cam_t_point = lc::RandomPoint3d();
    const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
    const auto noise = gtsam::noiseModel::Unit::Create(2);
    gtsam::Matrix H;
    const auto projection = vc::Project(cam_t_point, intrinsics, H);
    const auto numerical_H = gtsam::numericalDerivative11<Eigen::Vector2d, Eigen::Vector3d>(
      boost::function<Eigen::Vector2d(const Eigen::Vector3d&)>(boost::bind(&vc::Project, _1, intrinsics, boost::none)),
      cam_t_point, 1e-5);
    EXPECT_MATRIX_NEAR(numerical_H, H, 1e-6);
  }
}

TEST(UtilitiesTester, Backprojection) {
  for (int i = 0; i < 50; ++i) {
    const auto cam_t_point = lc::RandomFrontFacingPoint();
    const auto intrinsics = lc::RandomIntrinsics();
    const auto projected_point = vc::Project(cam_t_point, intrinsics);
    const auto backprojected_point = vc::Backproject(projected_point, intrinsics, cam_t_point.z());
    EXPECT_MATRIX_NEAR(cam_t_point, backprojected_point, 1e-6);
  }
}

TEST(UtilitiesTester, BackprojectJacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point2 measurement = lc::RandomVector<2>();
    const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
    const double z = lc::RandomPositiveDouble();
    const auto noise = gtsam::noiseModel::Unit::Create(3);
    gtsam::Matrix H;
    const auto backprojection = vc::Backproject(measurement, intrinsics, z, H);
    const auto numerical_H = gtsam::numericalDerivative11<Eigen::Vector3d, double>(
      boost::function<Eigen::Vector3d(const double)>(
        boost::bind(&vc::Backproject, measurement, intrinsics, _1, boost::none)),
      z, 1e-5);
    EXPECT_MATRIX_NEAR(numerical_H, H, 1e-6);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
