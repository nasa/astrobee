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

#include <localization_common/logger.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/impl/fast_bilateral.hpp>
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

template <typename PointType, typename PointWithNormalType>
void EstimateOrganizedNormals(const typename pcl::PointCloud<PointType>::Ptr cloud,
                              const double max_depth_change_factor, const double normal_smoothing_size,
                              pcl::PointCloud<PointWithNormalType>& cloud_with_normals);

Eigen::Matrix4f RansacIA(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud,
                         const pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud);

pcl::PointCloud<pcl::FPFHSignature33>::Ptr EstimateHistogramFeatures(
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);

Eigen::Matrix<double, 1, 6> PointToPlaneJacobian(const gtsam::Point3& source_point, const gtsam::Vector3& normal,
                                                 const gtsam::Pose3& target_T_source);

Eigen::Matrix<double, 3, 6> PointToPointJacobian(const gtsam::Point3& source_point,
                                                 const gtsam::Pose3& target_T_source);

Eigen::Isometry3d RelativeTransformUmeyama(const std::vector<Eigen::Vector3d>& source_points,
                                           const std::vector<Eigen::Vector3d>& target_points);

template <typename PointType>
boost::optional<Eigen::Vector3d> GetNormal(const Eigen::Vector3d& point, const pcl::PointCloud<PointType>& cloud,
                                           const pcl::search::KdTree<PointType>& kdtree,
                                           const double search_radius = 0.03);

template <typename PointType>
bool computePointNormal(const pcl::PointCloud<PointType>& cloud, const std::vector<int>& indices, float& normal_x,
                        float& normal_y, float& normal_z, float& curvature);

boost::optional<Eigen::Matrix<double, 6, 6>> PointToPointCovariance(const std::vector<Eigen::Vector3d>& source_points,
                                                                    const Eigen::Isometry3d& target_T_source);

boost::optional<Eigen::Matrix<double, 6, 6>> PointToPlaneCovariance(const std::vector<Eigen::Vector3d>& source_points,
                                                                    const std::vector<Eigen::Vector3d>& target_normals,
                                                                    const Eigen::Isometry3d& target_T_source);

template <typename PointType>
Eigen::Vector3d Vector3d(const PointType& point);

template <typename PointType>
Eigen::Vector3d NormalVector3d(const PointType& point_with_normal);

pcl::PointXYZI Interpolate(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, const double alpha);

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
void RemoveInvalidAndZeroPoints(pcl::PointCloud<PointType>& cloud);

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr DownsamplePointCloud(const typename pcl::PointCloud<PointType>::Ptr cloud,
                                                              const double leaf_size);

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr BilateralFilterOrganizedCloud(
  const typename pcl::PointCloud<PointType>::Ptr cloud, const double sigma_s, const double sigma_r);
template <typename PointType>
void FilterCorrespondences(const typename pcl::PointCloud<PointType>& input_cloud,
                           const typename pcl::PointCloud<PointType>& target_cloud,
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
void RemoveInvalidAndZeroPoints(pcl::PointCloud<PointType>& cloud) {
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
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  typename pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>());
  voxel_grid.filter(*downsampled_cloud);
  return downsampled_cloud;
}

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr BilateralFilterOrganizedCloud(
  const typename pcl::PointCloud<PointType>::Ptr cloud, const double sigma_s, const double sigma_r) {
  typename pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>());
  typename pcl::FastBilateralFilter<PointType> bilateral_filter;
  bilateral_filter.setInputCloud(cloud);
  bilateral_filter.setSigmaS(sigma_s);
  bilateral_filter.setSigmaR(sigma_r);
  bilateral_filter.filter(*downsampled_cloud);
  return downsampled_cloud;
}

template <typename PointType>
void FilterCorrespondences(const typename pcl::PointCloud<PointType>& input_cloud,
                           const typename pcl::PointCloud<PointType>& target_cloud,
                           pcl::Correspondences& correspondences) {
  for (auto correspondence_it = correspondences.begin(); correspondence_it != correspondences.end();) {
    const auto& input_point = (input_cloud)[correspondence_it->index_query];
    const auto& target_point = (target_cloud)[correspondence_it->index_match];
    const bool invalid_correspondence = !std::isfinite(input_point.x) || !std::isfinite(input_point.y) ||
                                        !std::isfinite(input_point.z) || !std::isfinite(target_point.x) ||
                                        !std::isfinite(target_point.y) || !std::isfinite(target_point.z) ||
                                        !std::isfinite(target_point.normal_x) ||
                                        !std::isfinite(target_point.normal_y) || !std::isfinite(target_point.normal_z);
    if (invalid_correspondence) {
      correspondence_it = correspondences.erase(correspondence_it);
      continue;
    }
    ++correspondence_it;
  }
}

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr FilteredPointCloud(
  const typename pcl::PointCloud<PointType>::Ptr unfiltered_cloud) {
  typename pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>());
  pcl::copyPointCloud(*unfiltered_cloud, *filtered_cloud);
  RemoveInvalidAndZeroPoints(*filtered_cloud);
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
  pcl::PointCloud<pcl::Normal> cloud_normals;
  ne.compute(cloud_normals);
  pcl::concatenateFields(*cloud, cloud_normals, cloud_with_normals);
}

template <typename PointType, typename PointWithNormalType>
void EstimateOrganizedNormals(const typename pcl::PointCloud<PointType>::Ptr cloud,
                              const double max_depth_change_factor, const double normal_smoothing_size,
                              typename pcl::PointCloud<PointWithNormalType>& cloud_with_normals) {
  typename pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(max_depth_change_factor);
  ne.setNormalSmoothingSize(normal_smoothing_size);
  ne.setInputCloud(cloud);
  pcl::PointCloud<pcl::Normal> cloud_normals;
  ne.compute(cloud_normals);
  pcl::concatenateFields(*cloud, cloud_normals, cloud_with_normals);
}

template <typename PointType, typename PointWithNormalType>
typename pcl::PointCloud<PointWithNormalType>::Ptr FilteredPointCloudWithNormals(
  const typename pcl::PointCloud<PointType>::Ptr unfiltered_cloud, const double search_radius) {
  typename pcl::PointCloud<PointType>::Ptr filtered_cloud = FilteredPointCloud<PointType>(unfiltered_cloud);
  typename pcl::PointCloud<PointWithNormalType>::Ptr filtered_cloud_with_normals(
    new pcl::PointCloud<PointWithNormalType>());
  EstimateNormals<PointType, PointWithNormalType>(filtered_cloud, search_radius, *filtered_cloud_with_normals);
  RemoveInvalidAndZeroPoints(*filtered_cloud_with_normals);
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

template <typename PointType>
boost::optional<Eigen::Vector3d> GetNormal(const Eigen::Vector3d& point, const pcl::PointCloud<PointType>& cloud,
                                           const pcl::search::KdTree<PointType>& kdtree, const double search_radius) {
  // Adapted from pcl code
  PointType pcl_point;
  pcl_point.x = point.x();
  pcl_point.y = point.y();
  pcl_point.z = point.z();

  std::vector<int> nn_indices;
  std::vector<float> distances;
  if (kdtree.radiusSearch(pcl_point, search_radius, nn_indices, distances, 0) < 3) {
    LogDebug("GetNormal: Failed to get enough neighboring points for query point.");
    return boost::none;
  }

  float normal_x;
  float normal_y;
  float normal_z;
  float curvature;
  if (!computePointNormal(cloud, nn_indices, normal_x, normal_y, normal_z, curvature)) {
    LogDebug("GetNormal: Failed to compute point normal.");
    return boost::none;
  }

  const double vpx = cloud.sensor_origin_.coeff(0);
  const double vpy = cloud.sensor_origin_.coeff(1);
  const double vpz = cloud.sensor_origin_.coeff(2);
  flipNormalTowardsViewpoint(pcl_point, vpx, vpy, vpz, normal_x, normal_y, normal_z);
  return Eigen::Vector3d(normal_x, normal_y, normal_z);
}

template <typename PointType>
bool computePointNormal(const pcl::PointCloud<PointType>& cloud, const std::vector<int>& indices, float& normal_x,
                        float& normal_y, float& normal_z, float& curvature) {
  // Adapted from pcl::common::centroid.h, should be a static or free function but is a member function instead
  /** \brief Placeholder for the 3x3 covariance matrix at each surface patch. */
  static EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix_;
  /** \brief 16-bytes aligned placeholder for the XYZ centroid of a surface patch. */
  static Eigen::Vector4f xyz_centroid_;
  if (indices.size() < 3 ||
      pcl::computeMeanAndCovarianceMatrix(cloud, indices, covariance_matrix_, xyz_centroid_) == 0) {
    return false;
  }

  // Get the plane normal and surface curvature
  pcl::solvePlaneParameters(covariance_matrix_, normal_x, normal_y, normal_z, curvature);
  return true;
}
}  // namespace point_cloud_common
#endif  // POINT_CLOUD_COMMON_UTILITIES_H_
