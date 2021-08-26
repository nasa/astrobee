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
#ifndef DEPTH_ODOMETRY_ICP_H_
#define DEPTH_ODOMETRY_ICP_H_

#include <depth_odometry/icp_params.h>
#include <localization_common/pose_with_covariance.h>
#include <localization_common/time.h>

#include <boost/optional.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace depth_odometry {
class ICP {
 public:
  ICP(const ICPParams& params);
  const boost::optional<pcl::Correspondences>& correspondences() const { return correspondences_; }
  boost::optional<localization_common::PoseWithCovariance> ComputeRelativeTransform(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,
    const Eigen::Isometry3d& initial_estimate = Eigen::Isometry3d::Identity());

 private:
  boost::optional<localization_common::PoseWithCovariance> RunICP(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,
    const Eigen::Isometry3d& initial_estimate = Eigen::Isometry3d::Identity());
  boost::optional<localization_common::PoseWithCovariance> RunCoarseToFineICP(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,
    const Eigen::Isometry3d& initial_estimate = Eigen::Isometry3d::Identity());
  void FilterCorrespondences(const pcl::PointCloud<pcl::PointXYZINormal>& input_cloud,
                             const pcl::PointCloud<pcl::PointXYZINormal>& target_cloud,
                             pcl::Correspondences& correspondences) const;
  Eigen::Matrix<double, 6, 6> ComputeCovarianceMatrix(
    const pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>& icp,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud_transformed,
    const Eigen::Isometry3d& relative_transform);
  Eigen::Matrix<double, 1, 6> Jacobian(const pcl::PointXYZINormal& source_point,
                                       const pcl::PointXYZINormal& target_point,
                                       const Eigen::Isometry3d& relative_transform) const;

  boost::optional<pcl::Correspondences> correspondences_;
  ICPParams params_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_ICP_H_
