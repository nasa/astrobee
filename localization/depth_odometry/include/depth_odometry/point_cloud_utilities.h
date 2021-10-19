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
#ifndef DEPTH_ODOMETRY_POINT_CLOUD_UTILITIES_H_
#define DEPTH_ODOMETRY_POINT_CLOUD_UTILITIES_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace depth_odometry {
void EstimateNormals(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const double search_radius,
                     pcl::PointCloud<pcl::PointXYZINormal>& cloud_with_normals);
Eigen::Matrix4f RansacIA(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud,
                         const pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud);

pcl::PointCloud<pcl::FPFHSignature33>::Ptr EstimateHistogramFeatures(
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);
Eigen::Matrix<double, 1, 6> Jacobian(const gtsam::Point3& point, const gtsam::Vector3& normal,
                                     const gtsam::Pose3& relative_transform);

Eigen::Isometry3d ComputeRelativeTransformUmeyama(const std::vector<Eigen::Vector3d>& source_points,
                                                  const std::vector<Eigen::Vector3d>& target_points);

boost::optional<Eigen::Vector3d> GetNormal(const Eigen::Vector3d& point, const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                           const pcl::search::KdTree<pcl::PointXYZI>& kdtree,
                                           const double search_radius = 0.03);

bool computePointNormal(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<int>& indices, float& nx,
                        float& ny, float& nz, float& curvature);

void flipNormalTowardsViewpoint(const pcl::PointXYZI& point, float vp_x, float vp_y, float vp_z, float& nx, float& ny,
                                float& nz);

template <typename PointType>
bool ValidPoint(const PointType& point) = delete;

template <>
bool ValidPoint<pcl::PointXYZ>(const pcl::PointXYZ& point);

template <>
bool ValidPoint<pcl::PointXYZI>(const pcl::PointXYZI& point);

template <>
bool ValidPoint<pcl::PointNormal>(const pcl::PointNormal& point);

template <>
bool ValidPoint<pcl::PointXYZINormal>(const pcl::PointXYZINormal& point);

template <typename Type>
bool ApproxZero(const Type& point, const double epsilon = 1e-5) {
  return std::abs(point) <= epsilon;
}

template <typename PointXYZType>
bool ValidPointXYZ(const PointXYZType& point) {
  const bool finite_point = pcl_isfinite(point.x) && pcl_isfinite(point.y) && pcl_isfinite(point.z);
  const bool nonzero_point = !ApproxZero(point.x) || !ApproxZero(point.y) || !ApproxZero(point.z);
  return finite_point && nonzero_point;
}

template <typename PointNormalType>
bool ValidNormal(const PointNormalType& point) {
  const bool finite_normal =
    pcl_isfinite(point.normal_x) && pcl_isfinite(point.normal_y) && pcl_isfinite(point.normal_z);
  const bool nonzero_normal = !ApproxZero(point.normal_x) || !ApproxZero(point.normal_y) || !ApproxZero(point.normal_z);
  return finite_normal && nonzero_normal;
}

template <typename PointIntensityType>
bool ValidIntensity(const PointIntensityType& point) {
  return pcl_isfinite(point.intensity);
}

template <typename PointType>
void RemoveNansAndZerosFromPoints(pcl::PointCloud<PointType>& cloud) {
  size_t new_index = 0;
  for (const auto& point : cloud.points) {
    const bool valid_point = ValidPoint(point);
    if (!valid_point) continue;
    cloud.points[new_index++] = point;
  }
  if (new_index != cloud.points.size()) {
    cloud.points.resize(new_index);
  }

  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(new_index);
  cloud.is_dense = true;
}

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr DownsamplePointCloud(const typename pcl::PointCloud<PointType>::Ptr cloud,
                                                              const double leaf_size) {
  typename pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>());
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid.filter(*downsampled_cloud);
  return downsampled_cloud;
}
}  // namespace depth_odometry
#endif  // DEPTH_ODOMETRY_POINT_CLOUD_UTILITIES_H_
