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
#ifndef POINT_CLOUD_COMMON_UTILITIES_H_
#define POINT_CLOUD_COMMON_UTILITIES_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/search.hpp>

#include <vector>

namespace point_cloud_common {
template <typename PointType, typename PointWithNormalType>
void EstimateNormals(const typename pcl::PointCloud<PointType>::Ptr cloud, const double search_radius,
                     pcl::PointCloud<PointWithNormalType>& cloud_with_normals);

Eigen::Matrix4f RansacIA(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud,
                         const pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud);

pcl::PointCloud<pcl::FPFHSignature33>::Ptr EstimateHistogramFeatures(
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);

Eigen::Matrix<double, 1, 6> PointToPlaneJacobian(const gtsam::Point3& point, const gtsam::Vector3& normal,
                                                 const gtsam::Pose3& relative_transform);

Eigen::Matrix<double, 1, 6> PointToPointJacobian(const gtsam::Point3& source_point,
                                                 const gtsam::Pose3& relative_transform);

Eigen::Isometry3d RelativeTransformUmeyama(const std::vector<Eigen::Vector3d>& source_points,
                                           const std::vector<Eigen::Vector3d>& target_points);

boost::optional<Eigen::Vector3d> GetNormal(const Eigen::Vector3d& point, const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                           const pcl::search::KdTree<pcl::PointXYZI>& kdtree,
                                           const double search_radius = 0.03);

bool computePointNormal(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<int>& indices, float& nx,
                        float& ny, float& nz, float& curvature);

void flipNormalTowardsViewpoint(const pcl::PointXYZI& point, float vp_x, float vp_y, float vp_z, float& nx, float& ny,
                                float& nz);

bool ValidVector6d(const Eigen::Matrix<double, 1, 6>& vector);

Eigen::Matrix<double, 6, 6> PointToPointCovariance(const std::vector<Eigen::Vector3d>& source_points,
                                                   const Eigen::Isometry3d& relative_transform);

Eigen::Matrix<double, 6, 6> PointToPlaneCovariance(const std::vector<Eigen::Vector3d>& source_points,
                                                   const std::vector<Eigen::Vector3d>& target_normals,
                                                   const Eigen::Isometry3d& relative_transform);

template <typename PointType>
Eigen::Vector3d Vector3d(const PointType& point);

template <typename PointType>
Eigen::Vector3d NormalVector3d(const PointType& point_with_normal);

pcl::PointXYZI Interpolate(const double alpha, const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b);

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
bool ValidPointXYZ(const PointXYZType& point);

template <typename PointNormalType>
bool ValidNormal(const PointNormalType& point);

template <typename PointIntensityType>
bool ValidIntensity(const PointIntensityType& point);

template <typename PointType>
void RemoveNansAndZerosFromPoints(pcl::PointCloud<PointType>& cloud);

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr DownsamplePointCloud(const typename pcl::PointCloud<PointType>::Ptr cloud,
                                                              const double leaf_size);

void FilterCorrespondences(const pcl::PointCloud<pcl::PointXYZINormal>& input_cloud,
                           const pcl::PointCloud<pcl::PointXYZINormal>& target_cloud,
                           pcl::Correspondences& correspondences);

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr FilteredPointCloud(
  const typename pcl::PointCloud<PointType>::Ptr unfiltered_cloud);

template <typename PointType, typename PointWithNormalType>
typename pcl::PointCloud<PointWithNormalType>::Ptr FilteredPointCloudWithNormals(
  const typename pcl::PointCloud<PointType>::Ptr unfiltered_cloud, const double search_radius);

// Implementation
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

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr FilteredPointCloud(
  const typename pcl::PointCloud<PointType>::Ptr unfiltered_cloud) {
  typename pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>());
  pcl::copyPointCloud(*unfiltered_cloud, *filtered_cloud);
  RemoveNansAndZerosFromPoints(*filtered_cloud);
  return filtered_cloud;
}

template <typename PointType, typename PointWithNormalType>
void EstimateNormals(const typename pcl::PointCloud<PointType>::Ptr cloud, const double search_radius,
                     typename pcl::PointCloud<PointWithNormalType>& cloud_with_normals) {
  typename pcl::NormalEstimation<PointType, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  typename pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(search_radius);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*cloud_normals);
  pcl::concatenateFields(*cloud, *cloud_normals, cloud_with_normals);
}

template <typename PointType, typename PointWithNormalType>
typename pcl::PointCloud<PointWithNormalType>::Ptr FilteredPointCloudWithNormals(
  const typename pcl::PointCloud<PointType>::Ptr unfiltered_cloud, const double search_radius) {
  typename pcl::PointCloud<PointType>::Ptr filtered_cloud = FilteredPointCloud<PointType>(unfiltered_cloud);
  typename pcl::PointCloud<PointWithNormalType>::Ptr filtered_cloud_with_normals(
    new pcl::PointCloud<PointWithNormalType>());
  EstimateNormals<PointType, PointWithNormalType>(filtered_cloud, search_radius, *filtered_cloud_with_normals);
  RemoveNansAndZerosFromPoints(*filtered_cloud_with_normals);
  return filtered_cloud_with_normals;
}

template <typename PointType>
Eigen::Vector3d Vector3d(const PointType& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

template <typename PointType>
Eigen::Vector3d NormalVector3d(const PointType& point_with_normal) {
  return Eigen::Vector3d(point_with_normal.normal[0], point_with_normal.normal[1], point_with_normal.normal[2]);
}
}  // namespace point_cloud_common
#endif  // POINT_CLOUD_COMMON_UTILITIES_H_
