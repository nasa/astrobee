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

#include <depth_odometry/depth_odometry.h>
#include <localization_common/logger.h>

#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace depth_odometry {
namespace lc = localization_common;

DepthOdometry::DepthOdometry() {}

boost::optional<Eigen::Isometry3d> DepthOdometry::DepthCloudCallback(
  std::pair<lc::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> depth_cloud) {
  if (!previous_depth_cloud_.second && !latest_depth_cloud_.second) latest_depth_cloud_ = depth_cloud;
  if (depth_cloud.first < latest_depth_cloud_.first) {
    LogWarning("DepthCloudCallback: Out of order measurement received.");
    return boost::none;
  }
  previous_depth_cloud_ = latest_depth_cloud_;
  latest_depth_cloud_ = depth_cloud;

  return Icp(previous_depth_cloud_.second, latest_depth_cloud_.second);
}

Eigen::Isometry3d DepthOdometry::Icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b) const {
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_a);
  icp.setInputTarget(cloud_b);

  pcl::PointCloud<pcl::PointXYZ> result;
  icp.align(result);

  // TODO(rsoussan): clean this up
  const Eigen::Isometry3d relative_transform(
    Eigen::Isometry3f(icp.getFinalTransformation().matrix()).cast<double>());  //.cast<double>();
  return relative_transform;
  // TODO(rsoussan): get covariance!!!
}
}  // namespace depth_odometry
