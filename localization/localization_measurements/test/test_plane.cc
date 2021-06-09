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
#include <localization_measurements/plane.h>

#include <gtsam/base/numericalDerivative.h>

#include <gtest/gtest.h>

namespace lm = localization_measurements;

TEST(PlaneTester, PointToPlaneJacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 point = Eigen::Vector3d::Random();
    const gtsam::Point3 plane_point = Eigen::Vector3d::Random();
    const gtsam::Vector3 plane_normal = Eigen::Vector3d::Random();
    const lm::Plane plane(plane_point, plane_normal);
    gtsam::Matrix H;
    const double distance = plane.Distance(point, H);
    const auto numerical_H = gtsam::numericalDerivative11<double, gtsam::Point3>(
      boost::function<double(const gtsam::Point3&)>(boost::bind(&lm::Plane::Distance, plane, _1, boost::none)), point,
      1e-5);
    ASSERT_TRUE(numerical_H.isApprox(H.matrix(), 1e-6));
  }
}

TEST(PlaneTester, PointToPlaneDistanceXAxisPlane) {
  const gtsam::Point3 point(0, 0, 0);
  const gtsam::Vector3 normal(1, 0, 0);
  // Plane perpendicular to x-axis
  const lm::Plane x_axis_plane(point, normal);
  // In front of plane
  {
    const gtsam::Point3 test_point(1, 0, 0);
    const double distance = x_axis_plane.Distance(test_point);
    EXPECT_NEAR(1.0, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(1, 2, 3);
    const double distance = x_axis_plane.Distance(test_point);
    EXPECT_NEAR(1.0, distance, 1e-6);
  }
  // Behind plane
  {
    const gtsam::Point3 test_point(-1, 0, 0);
    const double distance = x_axis_plane.Distance(test_point);
    EXPECT_NEAR(-1.0, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(-1, 2, 3);
    const double distance = x_axis_plane.Distance(test_point);
    EXPECT_NEAR(-1.0, distance, 1e-6);
  }
  // On plane
  {
    const gtsam::Point3 test_point(0, 0, 0);
    const double distance = x_axis_plane.Distance(test_point);
    EXPECT_NEAR(0, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(0, 2, 3);
    const double distance = x_axis_plane.Distance(test_point);
    EXPECT_NEAR(0, distance, 1e-6);
  }
}

TEST(PlaneTester, PointToPlaneDistanceYAxisPlane) {
  const gtsam::Point3 point(0, 1.1, 0);
  const gtsam::Vector3 normal(0, 1, 0);
  // Plane perpendicular to y-axis
  const lm::Plane y_axis_plane(point, normal);
  // In front of plane
  {
    const gtsam::Point3 test_point(0, 2.3, 0);
    const double distance = y_axis_plane.Distance(test_point);
    EXPECT_NEAR(1.2, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(1, 2.3, 3);
    const double distance = y_axis_plane.Distance(test_point);
    EXPECT_NEAR(1.2, distance, 1e-6);
  }
  // Behind plane
  {
    const gtsam::Point3 test_point(-1, -7.9, 100);
    const double distance = y_axis_plane.Distance(test_point);
    EXPECT_NEAR(-9, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(57, -7.9, 42.5);
    const double distance = y_axis_plane.Distance(test_point);
    EXPECT_NEAR(-9, distance, 1e-6);
  }
  // On plane
  {
    const gtsam::Point3 test_point(-3, 1.1, 100);
    const double distance = y_axis_plane.Distance(test_point);
    EXPECT_NEAR(0, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(21, 1.1, -3.3);
    const double distance = y_axis_plane.Distance(test_point);
    EXPECT_NEAR(0, distance, 1e-6);
  }
}
TEST(PlaneTester, PointToPlaneDistanceZAxisPlane) {
  const gtsam::Point3 point(2, 37, -88.2);
  const gtsam::Vector3 normal(0, 0, 1);
  // Plane perpendicular to z-axis
  const lm::Plane z_axis_plane(point, normal);
  // In front of plane
  {
    const gtsam::Point3 test_point(0, 0, 20);
    const double distance = z_axis_plane.Distance(test_point);
    EXPECT_NEAR(108.2, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(1, 2.3, 23);
    const double distance = z_axis_plane.Distance(test_point);
    EXPECT_NEAR(111.2, distance, 1e-6);
  }
  // Behind plane
  {
    const gtsam::Point3 test_point(-2, -3.9, -100);
    const double distance = z_axis_plane.Distance(test_point);
    EXPECT_NEAR(-11.8, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(2.1, 0.23, -89.0);
    const double distance = z_axis_plane.Distance(test_point);
    EXPECT_NEAR(-0.8, distance, 1e-6);
  }
  // On plane
  {
    const gtsam::Point3 test_point(-3, 1.1, -88.2);
    const double distance = z_axis_plane.Distance(test_point);
    EXPECT_NEAR(0, distance, 1e-6);
  }
  {
    const gtsam::Point3 test_point(21, 1.1, -88.2);
    const double distance = z_axis_plane.Distance(test_point);
    EXPECT_NEAR(0, distance, 1e-6);
  }
}
