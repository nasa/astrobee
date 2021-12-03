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
#include <point_cloud_common/utilities.h>
#include <point_cloud_common/test_utilities.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtest/gtest.h>

#include <limits>

namespace lc = localization_common;
namespace pc = point_cloud_common;
namespace sym = gtsam::symbol_shorthand;

double PointToPlaneError(const gtsam::Point3& point_1, const gtsam::Point3& point_2, const gtsam::Vector3& normal_2,
                         const gtsam::Pose3& relative_transform) {
  return (relative_transform * point_1 - point_2).dot(normal_2);
}

TEST(UtilitiesTester, PointToPlaneJacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 point_1 = lc::RandomVector();
    const gtsam::Point3 point_2 = lc::RandomVector();
    const gtsam::Vector3 normal_2 = lc::RandomVector();
    const gtsam::Pose3 relative_transform = lc::RandomPose();
    const gtsam::Matrix H = pc::PointToPlaneJacobian(point_1, normal_2, relative_transform);
    const auto numerical_H = gtsam::numericalDerivative11<double, gtsam::Pose3>(
      boost::function<double(const gtsam::Pose3&)>(boost::bind(&PointToPlaneError, point_1, point_2, normal_2, _1)),
      relative_transform, 1e-5);
    EXPECT_TRUE(numerical_H.isApprox(H.matrix(), 1e-6));
  }
}

gtsam::Vector3 PointToPointError(const gtsam::Point3& point_1, const gtsam::Point3& point_2,
                                 const gtsam::Pose3& relative_transform) {
  return relative_transform * point_1 - point_2;
}

TEST(UtilitiesTester, PointToPointJacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 point_1 = lc::RandomVector();
    const gtsam::Point3 point_2 = lc::RandomVector();
    const gtsam::Pose3 relative_transform = lc::RandomPose();
    const gtsam::Matrix H = pc::PointToPointJacobian(point_1, relative_transform);
    const auto numerical_H = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
      boost::function<gtsam::Vector3(const gtsam::Pose3&)>(boost::bind(&PointToPointError, point_1, point_2, _1)),
      relative_transform, 1e-5);
    EXPECT_TRUE(numerical_H.isApprox(H.matrix(), 1e-6));
  }
}

TEST(UtilitiesTester, ValidPoint_XYZ) {
  // Valid
  {
    pcl::PointXYZ p(1, 2, 3);
    EXPECT_TRUE(pc::ValidPoint(p));
  }
  // Invalid nan
  {
    pcl::PointXYZ p(std::numeric_limits<double>::quiet_NaN(), 2, 3);
    EXPECT_FALSE(pc::ValidPoint(p));
  }
  // Invalid inf
  {
    pcl::PointXYZ p(std::numeric_limits<double>::infinity(), 2, 3);
    EXPECT_FALSE(pc::ValidPoint(p));
  }
}

TEST(UtilitiesTester, ValidPoint_Normal) {
  // Valid
  {
    pcl::PointNormal p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.normal[0] = 1;
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_TRUE(pc::ValidPoint(p));
  }
  // Invalid nan
  {
    // point
    pcl::PointNormal p;
    p.x = std::numeric_limits<double>::quiet_NaN();
    p.y = 2;
    p.z = 3;
    p.normal[0] = 1;
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidPoint(p));

    // normal
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.normal[0] = std::numeric_limits<double>::quiet_NaN();
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidPoint(p));
  }
  // Invalid inf
  {
    pcl::PointNormal p;
    // point
    p.x = std::numeric_limits<double>::infinity();
    p.y = 2;
    p.z = 3;
    p.normal[0] = 1;
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidPoint(p));

    // normal
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.normal[0] = std::numeric_limits<double>::infinity();
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidPoint(p));
  }
  // Invalid Zero Normal
  {
    pcl::PointNormal p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.normal[0] = 0;
    p.normal[1] = 0;
    p.normal[2] = 0;
    EXPECT_FALSE(pc::ValidPoint(p));
  }
}

TEST(UtilitiesTester, ValidPoint_XYZI) {
  // Valid
  {
    pcl::PointXYZI p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = 4;
    EXPECT_TRUE(pc::ValidPoint(p));
  }
  // Invalid nan
  {
    pcl::PointXYZI p;
    p.x = std::numeric_limits<double>::quiet_NaN();
    p.y = 2;
    p.z = 3;
    p.intensity = 4;
    EXPECT_FALSE(pc::ValidPoint(p));
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(pc::ValidPoint(p));
  }
  // Invalid inf
  {
    pcl::PointXYZI p;
    p.x = std::numeric_limits<double>::infinity();
    p.y = 2;
    p.z = 3;
    p.intensity = 1;
    EXPECT_FALSE(pc::ValidPoint(p));

    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(pc::ValidPoint(p));
  }
}

TEST(UtilitiesTester, ValidPoint_XYZINormal) {
  // Valid
  {
    pcl::PointXYZINormal p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = 4;
    p.normal[0] = 1;
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_TRUE(pc::ValidPoint(p));
  }
  // Invalid nan
  {
    // point
    pcl::PointXYZINormal p;
    p.x = std::numeric_limits<double>::quiet_NaN();
    p.y = 2;
    p.z = 3;
    p.intensity = 4;
    p.normal[0] = 1;
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidPoint(p));

    // intensity
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = std::numeric_limits<double>::quiet_NaN();
    p.normal[0] = 1;
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidPoint(p));

    // normal
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = 4;
    p.normal[0] = std::numeric_limits<double>::quiet_NaN();
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidPoint(p));
  }
  // Invalid inf
  {
    pcl::PointXYZINormal p;
    // point
    p.x = std::numeric_limits<double>::infinity();
    p.y = 2;
    p.z = 3;
    p.intensity = 1;
    p.normal[0] = 1;
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidPoint(p));

    // intensity
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = std::numeric_limits<double>::infinity();
    p.normal[0] = 1;
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidPoint(p));

    // normal
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = 1;
    p.normal[0] = std::numeric_limits<double>::infinity();
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidPoint(p));
  }
  // Invalid Zero Normal
  {
    pcl::PointXYZINormal p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = 1;
    p.normal[0] = 0;
    p.normal[1] = 0;
    p.normal[2] = 0;
    EXPECT_FALSE(pc::ValidPoint(p));
  }
}

TEST(UtilitiesTester, ValidPointXYZ) {
  // Valid
  {
    pcl::PointXYZ p(1, 2, 3);
    EXPECT_TRUE(pc::ValidPointXYZ(p));
  }
  // Invalid nan
  {
    pcl::PointXYZ p(std::numeric_limits<double>::quiet_NaN(), 2, 3);
    EXPECT_FALSE(pc::ValidPointXYZ(p));
  }
  // Invalid inf
  {
    pcl::PointXYZ p(std::numeric_limits<double>::infinity(), 2, 3);
    EXPECT_FALSE(pc::ValidPointXYZ(p));
  }
}

TEST(UtilitiesTester, ValidNormal) {
  // Valid
  {
    pcl::PointNormal p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.normal[0] = 1;
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_TRUE(pc::ValidNormal(p));
  }
  // Invalid nan
  {
    pcl::PointNormal p;
    p.x = std::numeric_limits<double>::quiet_NaN();
    p.y = 2;
    p.z = 3;
    p.normal[0] = 1;
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_TRUE(pc::ValidNormal(p));

    // normal
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.normal[0] = std::numeric_limits<double>::quiet_NaN();
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidNormal(p));
  }
  // Invalid inf
  {
    pcl::PointNormal p;
    // point
    p.x = std::numeric_limits<double>::infinity();
    p.y = 2;
    p.z = 3;
    p.normal[0] = 1;
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_TRUE(pc::ValidNormal(p));

    // normal
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.normal[0] = std::numeric_limits<double>::infinity();
    p.normal[1] = 2;
    p.normal[2] = 3;
    EXPECT_FALSE(pc::ValidNormal(p));
  }
  // Invalid Zero Normal
  {
    pcl::PointNormal p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.normal[0] = 0;
    p.normal[1] = 0;
    p.normal[2] = 0;
    EXPECT_FALSE(pc::ValidNormal(p));
  }
}

TEST(UtilitiesTester, ValidIntensity) {
  // Valid
  {
    pcl::PointXYZI p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = 4;
    EXPECT_TRUE(pc::ValidIntensity(p));
  }
  // Invalid nan
  {
    pcl::PointXYZI p;
    p.x = std::numeric_limits<double>::quiet_NaN();
    p.y = 2;
    p.z = 3;
    p.intensity = 4;
    EXPECT_TRUE(pc::ValidIntensity(p));
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(pc::ValidIntensity(p));
  }
  // Invalid inf
  {
    pcl::PointXYZI p;
    p.x = std::numeric_limits<double>::infinity();
    p.y = 2;
    p.z = 3;
    p.intensity = 1;
    EXPECT_TRUE(pc::ValidIntensity(p));

    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(pc::ValidIntensity(p));
  }
}

TEST(UtilitiesTester, ApproxZero) {
  EXPECT_FALSE(pc::ApproxZero(1.0));
  EXPECT_TRUE(pc::ApproxZero(0.0));
  EXPECT_TRUE(pc::ApproxZero(1e-9));
  EXPECT_TRUE(pc::ApproxZero(1, 2));
}

TEST(UtilitiesTester, Pcl2EigenVec3) {
  pcl::PointXYZ p(1, 2, 3);
  const auto eigen_p = pc::Vector3d(p);
  EXPECT_NEAR(eigen_p.x(), p.x, 1e-6);
  EXPECT_NEAR(eigen_p.y(), p.y, 1e-6);
  EXPECT_NEAR(eigen_p.z(), p.z, 1e-6);
}

TEST(UtilitiesTester, Pcl2EigenNormal) {
  pcl::PointNormal p;
  p.normal[0] = 1;
  p.normal[1] = 2;
  p.normal[2] = 3;
  const auto eigen_normal = pc::NormalVector3d(p);
  EXPECT_NEAR(eigen_normal.x(), p.normal[0], 1e-6);
  EXPECT_NEAR(eigen_normal.y(), p.normal[1], 1e-6);
  EXPECT_NEAR(eigen_normal.z(), p.normal[2], 1e-6);
}

TEST(UtilitiesTester, RemoveInvalidAndZeroPoints) {
  pcl::PointXYZ p_valid(0, 1, 2);
  pcl::PointXYZ p_nan(std::numeric_limits<double>::quiet_NaN(), 2, 3);
  pcl::PointXYZ p_inf(std::numeric_limits<double>::infinity(), 2, 3);
  pcl::PointXYZ p_zero(0, 0, 0);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.emplace_back(p_valid);
  cloud.points.emplace_back(p_nan);
  cloud.points.emplace_back(p_inf);
  cloud.points.emplace_back(p_zero);
  ASSERT_EQ(cloud.points.size(), 4);
  pc::RemoveInvalidAndZeroPoints(cloud);
  ASSERT_EQ(cloud.points.size(), 1);
  EXPECT_NEAR(cloud.points[0].x, p_valid.x, 1e-6);
  EXPECT_NEAR(cloud.points[0].y, p_valid.y, 1e-6);
  EXPECT_NEAR(cloud.points[0].z, p_valid.z, 1e-6);
}

TEST(UtilitiesTester, FilteredPointCloud) {
  pcl::PointXYZ p_valid(0, 1, 2);
  pcl::PointXYZ p_nan(std::numeric_limits<double>::quiet_NaN(), 2, 3);
  pcl::PointXYZ p_inf(std::numeric_limits<double>::infinity(), 2, 3);
  pcl::PointXYZ p_zero(0, 0, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->points.emplace_back(p_valid);
  cloud->points.emplace_back(p_nan);
  cloud->points.emplace_back(p_inf);
  cloud->points.emplace_back(p_zero);
  ASSERT_EQ(cloud->points.size(), 4);
  const auto filtered_cloud = pc::FilteredPointCloud<pcl::PointXYZ>(cloud);
  ASSERT_EQ(filtered_cloud->points.size(), 1);
  EXPECT_NEAR(filtered_cloud->points[0].x, p_valid.x, 1e-6);
  EXPECT_NEAR(filtered_cloud->points[0].y, p_valid.y, 1e-6);
  EXPECT_NEAR(filtered_cloud->points[0].z, p_valid.z, 1e-6);
}

TEST(UtilitiesTester, FilteredPointCloudWithNormalsNoValidNormals) {
  pcl::PointXYZ p_valid(0, 1, 2);
  pcl::PointXYZ p_nan(std::numeric_limits<double>::quiet_NaN(), 2, 3);
  pcl::PointXYZ p_inf(std::numeric_limits<double>::infinity(), 2, 3);
  pcl::PointXYZ p_zero(0, 0, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->points.emplace_back(p_valid);
  cloud->points.emplace_back(p_nan);
  cloud->points.emplace_back(p_inf);
  cloud->points.emplace_back(p_zero);
  ASSERT_EQ(cloud->points.size(), 4);
  constexpr double search_radius = 1.0;
  const auto filtered_cloud = pc::FilteredPointCloudWithNormals<pcl::PointXYZ, pcl::PointNormal>(cloud, search_radius);
  ASSERT_EQ(filtered_cloud->points.size(), 0);
}

TEST(UtilitiesTester, FilteredPointCloudWithNormalsZAxisNormals) {
  pcl::PointXYZ p_xy_plane_1(0, 1, 0);
  pcl::PointXYZ p_xy_plane_2(0, 1.1, 0);
  pcl::PointXYZ p_xy_plane_3(0.1, 1, 0);
  pcl::PointXYZ p_no_neighbors(100, 200, 300);
  pcl::PointXYZ p_nan(std::numeric_limits<double>::quiet_NaN(), 2, 3);
  pcl::PointXYZ p_inf(std::numeric_limits<double>::infinity(), 2, 3);
  pcl::PointXYZ p_zero(0, 0, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->points.emplace_back(p_xy_plane_1);
  cloud->points.emplace_back(p_xy_plane_2);
  cloud->points.emplace_back(p_xy_plane_3);
  cloud->points.emplace_back(p_no_neighbors);
  cloud->points.emplace_back(p_nan);
  cloud->points.emplace_back(p_inf);
  cloud->points.emplace_back(p_zero);
  ASSERT_EQ(cloud->points.size(), 7);
  constexpr double search_radius = 1.0;
  const auto filtered_cloud = pc::FilteredPointCloudWithNormals<pcl::PointXYZ, pcl::PointNormal>(cloud, search_radius);
  // Successful normals for each XY planar point
  ASSERT_EQ(filtered_cloud->points.size(), 3);
  // p1
  {
    const auto& filtered_point_with_normal = filtered_cloud->points[0];
    EXPECT_NEAR(filtered_point_with_normal.x, p_xy_plane_1.x, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.y, p_xy_plane_1.y, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.z, p_xy_plane_1.z, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[0], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[1], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[2], 1, 1e-6);
  }
  // p2
  {
    const auto& filtered_point_with_normal = filtered_cloud->points[1];
    EXPECT_NEAR(filtered_point_with_normal.x, p_xy_plane_2.x, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.y, p_xy_plane_2.y, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.z, p_xy_plane_2.z, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[0], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[1], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[2], 1, 1e-6);
  }
  // p3
  {
    const auto& filtered_point_with_normal = filtered_cloud->points[2];
    EXPECT_NEAR(filtered_point_with_normal.x, p_xy_plane_3.x, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.y, p_xy_plane_3.y, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.z, p_xy_plane_3.z, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[0], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[1], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[2], 1, 1e-6);
  }
}

TEST(UtilitiesTester, EstimateNormals) {
  pcl::PointXYZ p_xy_plane_1(0, 1, 0);
  pcl::PointXYZ p_xy_plane_2(0, 1.1, 0);
  pcl::PointXYZ p_xy_plane_3(0.1, 1, 0);
  pcl::PointXYZ p_no_neighbors(100, 200, 300);
  pcl::PointXYZ p_nan(std::numeric_limits<double>::quiet_NaN(), 2, 3);
  pcl::PointXYZ p_inf(std::numeric_limits<double>::infinity(), 2, 3);
  pcl::PointXYZ p_zero(0, 0, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->points.emplace_back(p_xy_plane_1);
  cloud->points.emplace_back(p_xy_plane_2);
  cloud->points.emplace_back(p_xy_plane_3);
  cloud->points.emplace_back(p_no_neighbors);
  cloud->points.emplace_back(p_nan);
  cloud->points.emplace_back(p_inf);
  cloud->points.emplace_back(p_zero);
  ASSERT_EQ(cloud->points.size(), 7);
  constexpr double search_radius = 1.0;
  pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
  pc::EstimateNormals<pcl::PointXYZ, pcl::PointNormal>(cloud, search_radius, cloud_with_normals);
  // EstimateNormals doesn't remove points with invalid points or normals, expect same size output cloud
  ASSERT_EQ(cloud_with_normals.points.size(), 7);
  // p1
  {
    const auto& filtered_point_with_normal = cloud_with_normals.points[0];
    EXPECT_NEAR(filtered_point_with_normal.x, p_xy_plane_1.x, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.y, p_xy_plane_1.y, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.z, p_xy_plane_1.z, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[0], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[1], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[2], 1, 1e-6);
  }
  // p2
  {
    const auto& filtered_point_with_normal = cloud_with_normals.points[1];
    EXPECT_NEAR(filtered_point_with_normal.x, p_xy_plane_2.x, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.y, p_xy_plane_2.y, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.z, p_xy_plane_2.z, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[0], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[1], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[2], 1, 1e-6);
  }
  // p3
  {
    const auto& filtered_point_with_normal = cloud_with_normals.points[2];
    EXPECT_NEAR(filtered_point_with_normal.x, p_xy_plane_3.x, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.y, p_xy_plane_3.y, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.z, p_xy_plane_3.z, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[0], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[1], 0, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.normal[2], 1, 1e-6);
  }
  // Invalid Points
  // No Neighbors
  {
    const auto& filtered_point_with_normal = cloud_with_normals.points[3];
    EXPECT_NEAR(filtered_point_with_normal.x, p_no_neighbors.x, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.y, p_no_neighbors.y, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.z, p_no_neighbors.z, 1e-6);
    // Invalid normal
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.normal[0]));
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.normal[1]));
    ASSERT_TRUE(std::isnan(filtered_point_with_normal.normal[2]));
  }
  // Nan
  {
    const auto& filtered_point_with_normal = cloud_with_normals.points[4];
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.x));
    EXPECT_NEAR(filtered_point_with_normal.y, p_nan.y, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.z, p_nan.z, 1e-6);
    // Invalid normal
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.normal[0]));
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.normal[1]));
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.normal[2]));
  }
  // Inf
  {
    const auto& filtered_point_with_normal = cloud_with_normals.points[5];
    EXPECT_FALSE(std::isfinite(filtered_point_with_normal.x));
    EXPECT_NEAR(filtered_point_with_normal.y, p_inf.y, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.z, p_inf.z, 1e-6);
    // Invalid normal
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.normal[0]));
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.normal[1]));
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.normal[2]));
  }
  // Zero Point
  {
    const auto& filtered_point_with_normal = cloud_with_normals.points[6];
    EXPECT_NEAR(filtered_point_with_normal.x, p_zero.x, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.y, p_zero.y, 1e-6);
    EXPECT_NEAR(filtered_point_with_normal.z, p_zero.z, 1e-6);
    // Invalid normal
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.normal[0]));
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.normal[1]));
    EXPECT_TRUE(std::isnan(filtered_point_with_normal.normal[2]));
  }
}

TEST(UtilitiesTester, GetNormal) {
  pcl::PointXYZ p_xy_plane_1(0, 1, 0);
  pcl::PointXYZ p_xy_plane_2(0, 1.1, 0);
  pcl::PointXYZ p_xy_plane_3(0.1, 1, 0);
  pcl::PointXYZ p_no_neighbors(100, 200, 300);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->points.emplace_back(p_xy_plane_1);
  cloud->points.emplace_back(p_xy_plane_2);
  cloud->points.emplace_back(p_xy_plane_3);
  cloud->points.emplace_back(p_no_neighbors);
  ASSERT_EQ(cloud->points.size(), 4);
  constexpr double search_radius = 1.0;
  pcl::search::KdTree<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  // p1
  {
    const auto normal = pc::GetNormal(pc::Vector3d(p_xy_plane_1), *cloud, kdtree, search_radius);
    ASSERT_TRUE(normal != boost::none);
    EXPECT_NEAR(normal->x(), 0, 1e-6);
    EXPECT_NEAR(normal->y(), 0, 1e-6);
    EXPECT_NEAR(normal->z(), 1, 1e-6);
  }
  // p2
  {
    const auto normal = pc::GetNormal(pc::Vector3d(p_xy_plane_2), *cloud, kdtree, search_radius);
    ASSERT_TRUE(normal != boost::none);
    EXPECT_NEAR(normal->x(), 0, 1e-6);
    EXPECT_NEAR(normal->y(), 0, 1e-6);
    EXPECT_NEAR(normal->z(), 1, 1e-6);
  }
  // p3
  {
    const auto normal = pc::GetNormal(pc::Vector3d(p_xy_plane_3), *cloud, kdtree, search_radius);
    ASSERT_TRUE(normal != boost::none);
    EXPECT_NEAR(normal->x(), 0, 1e-6);
    EXPECT_NEAR(normal->y(), 0, 1e-6);
    EXPECT_NEAR(normal->z(), 1, 1e-6);
  }
  // No neighbors
  {
    const auto normal = pc::GetNormal(pc::Vector3d(p_no_neighbors), *cloud, kdtree, search_radius);
    ASSERT_EQ(normal, boost::none);
  }
}

TEST(UtilitiesTester, FilterCorrespondences) {
  pcl::PointXYZINormal p_valid;
  p_valid.x = 1;
  p_valid.y = 2;
  p_valid.z = 3;
  p_valid.intensity = 4;
  p_valid.normal[0] = 1;
  p_valid.normal[1] = 2;
  p_valid.normal[2] = 3;

  pcl::PointXYZINormal p_nan;
  p_nan.x = std::numeric_limits<double>::quiet_NaN();
  p_nan.y = 2;
  p_nan.z = 3;
  p_nan.intensity = 4;
  p_nan.normal[0] = 1;
  p_nan.normal[1] = 2;
  p_nan.normal[2] = 3;

  pcl::PointXYZINormal p_inf;
  p_inf.x = std::numeric_limits<double>::infinity();
  p_inf.y = 2;
  p_inf.z = 3;
  p_inf.intensity = 1;
  p_inf.normal[0] = 1;
  p_inf.normal[1] = 2;
  p_inf.normal[2] = 3;
  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  cloud.points.emplace_back(p_valid);
  cloud.points.emplace_back(p_nan);
  cloud.points.emplace_back(p_inf);
  ASSERT_EQ(cloud.points.size(), 3);
  pcl::Correspondences correspondences;
  // Only 0,0 is valid
  correspondences.emplace_back(pcl::Correspondence(0, 0, 1));
  correspondences.emplace_back(pcl::Correspondence(1, 0, 1));
  correspondences.emplace_back(pcl::Correspondence(0, 1, 1));
  correspondences.emplace_back(pcl::Correspondence(2, 0, 1));
  correspondences.emplace_back(pcl::Correspondence(0, 2, 1));
  pc::FilterCorrespondences(cloud, cloud, correspondences);
  ASSERT_EQ(correspondences.size(), 1);
  EXPECT_EQ(correspondences[0].index_query, 0);
  EXPECT_EQ(correspondences[0].index_match, 0);
}

TEST(UtilitiesTester, DownsamplePointCloud) {
  pcl::PointXYZ p1(0.3, 0, 0);
  pcl::PointXYZ p2(0, 0.3, 0);
  pcl::PointXYZ p3(0, 0, 0.3);
  pcl::PointXYZ p_no_neighbors(100, 200, 300);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->points.emplace_back(p1);
  cloud->points.emplace_back(p2);
  cloud->points.emplace_back(p3);
  cloud->points.emplace_back(p_no_neighbors);
  ASSERT_EQ(cloud->points.size(), 4);
  constexpr double leaf_size = 1.0;
  const auto downsampled_cloud = pc::DownsamplePointCloud<pcl::PointXYZ>(cloud, leaf_size);
  ASSERT_EQ(downsampled_cloud->points.size(), 2);
  // Average of first three points is 0.1, 0.1, 0.1
  {
    const auto& p = downsampled_cloud->points[0];
    EXPECT_NEAR(p.x, 0.1, 1e-6);
    EXPECT_NEAR(p.y, 0.1, 1e-6);
    EXPECT_NEAR(p.z, 0.1, 1e-6);
  }
  // Point with no neighbors should be preserved
  {
    const auto& p = downsampled_cloud->points[1];
    EXPECT_NEAR(p.x, p_no_neighbors.x, 1e-6);
    EXPECT_NEAR(p.y, p_no_neighbors.y, 1e-6);
    EXPECT_NEAR(p.z, p_no_neighbors.z, 1e-6);
  }
}

TEST(UtilitiesTester, Interpolate) {
  pcl::PointXYZI p1;
  p1.x = 0;
  p1.y = 0;
  p1.z = 0;
  p1.intensity = 0;

  pcl::PointXYZI p2;
  p2.x = 10;
  p2.y = 100;
  p2.z = 1000;
  p2.intensity = 10000;

  {
    const auto interpolated_p = pc::Interpolate(p1, p2, 0.5);
    EXPECT_NEAR(interpolated_p.x, 5, 1e-6);
    EXPECT_NEAR(interpolated_p.y, 50, 1e-6);
    EXPECT_NEAR(interpolated_p.z, 500, 1e-6);
    EXPECT_NEAR(interpolated_p.intensity, 5000, 1e-6);
  }

  {
    const auto interpolated_p = pc::Interpolate(p1, p2, 0.3);
    EXPECT_NEAR(interpolated_p.x, 3, 1e-6);
    EXPECT_NEAR(interpolated_p.y, 30, 1e-6);
    EXPECT_NEAR(interpolated_p.z, 300, 1e-6);
    EXPECT_NEAR(interpolated_p.intensity, 3000, 1e-6);
  }
}

TEST(UmeyamaTester, PerfectEstimate) {
  for (int i = 0; i < 50; ++i) {
    const auto a_T_points = pc::RandomPoints(20);
    const auto b_T_a = lc::RandomIsometry3d();
    const auto b_T_points = lc::Transform(a_T_points, b_T_a);
    const auto estimated_b_T_a = pc::RelativeTransformUmeyama(a_T_points, b_T_points);
    EXPECT_PRED2(lc::MatrixEquality<6>, estimated_b_T_a.matrix(), b_T_a.matrix());
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
