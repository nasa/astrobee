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
#ifndef POINT_CLOUD_COMMON_POINT_TO_PLANE_ICP_H_
#define POINT_CLOUD_COMMON_POINT_TO_PLANE_ICP_H_

#include <localization_common/pose_with_covariance.h>
#include <localization_common/time.h>
#include <point_cloud_common/point_to_plane_icp_params.h>

#include <boost/optional.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace point_cloud_common {
class PointToPlaneICP {
 public:
  explicit PointToPlaneICP(const PointToPlaneICPParams& params);
  const boost::optional<pcl::Correspondences>& correspondences() const { return correspondences_; }
  boost::optional<localization_common::PoseWithCovariance> ComputeRelativeTransform(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud_with_normals,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud_with_normals,
    const Eigen::Isometry3d& initial_estimate = Eigen::Isometry3d::Identity());

 private:
  boost::optional<localization_common::PoseWithCovariance> RunPointToPlaneICP(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud_with_normals,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud_with_normals,
    const Eigen::Isometry3d& initial_estimate = Eigen::Isometry3d::Identity());
  boost::optional<localization_common::PoseWithCovariance> RunCoarseToFinePointToPlaneICP(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud_with_normals,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud_with_normals,
    const Eigen::Isometry3d& initial_estimate = Eigen::Isometry3d::Identity());
  Eigen::Matrix<double, 6, 6> PointToPlaneCovariance(
    const pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>& icp,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud_transformed,
    const Eigen::Isometry3d& relative_transform);
  boost::optional<pcl::Correspondences> correspondences_;
  PointToPlaneICPParams params_;
};
}  // namespace point_cloud_common

#endif  // POINT_CLOUD_COMMON_POINT_TO_PLANE_ICP_H_
