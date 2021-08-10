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
#include <depth_odometry/transformation_estimation_symmetric_point_to_plane_lls.h>
#include <depth_odometry/point_cloud_utilities.h>
#include <depth_odometry/parameter_reader.h>
#include <localization_common/logger.h>
#include <localization_common/timer.h>
#include <localization_common/utilities.h>

#include <gtsam/geometry/Pose3.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
// TODO(rsoussan): Switch back to this when PCL bug is fixed
//#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <depth_odometry/correspondence_rejection_surface_normal2.h>
#include <pcl/registration/icp.h>

namespace depth_odometry {
namespace lc = localization_common;

DepthOdometry::DepthOdometry() {
  // TODO(rsoussan): remove this
  config_reader::ConfigReader config;
  config.AddFile("transforms.config");
  config.AddFile("geometry.config");
  config.AddFile("localization/depth_odometry.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  LoadDepthOdometryParams(config, params_);
  icp_.reset(new ICP(params_.icp));
}

std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> DepthOdometry::previous_depth_cloud() const {
  return previous_depth_cloud_;
}
std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> DepthOdometry::latest_depth_cloud() const {
  return latest_depth_cloud_;
}

Eigen::Isometry3d DepthOdometry::latest_relative_transform() const { return latest_relative_transform_; }

const pcl::Correspondences& DepthOdometry::correspondences() const { return icp_->correspondences(); }

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> DepthOdometry::DepthCloudCallback(
  std::pair<lc::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> depth_cloud) {
  RemoveNansAndZerosFromPointXYZs(*(depth_cloud.second));
  if (!previous_depth_cloud_.second && !latest_depth_cloud_.second) latest_depth_cloud_ = depth_cloud;
  if (depth_cloud.first < latest_depth_cloud_.first) {
    LogWarning("DepthCloudCallback: Out of order measurement received.");
    return boost::none;
  }
  LogError("t: " << std::setprecision(15) << depth_cloud.first);
  previous_depth_cloud_ = latest_depth_cloud_;
  latest_depth_cloud_ = depth_cloud;
  auto relative_transform = icp_->ComputeRelativeTransform(previous_depth_cloud_.second, latest_depth_cloud_.second);
  if (!relative_transform) {
    LogWarning("DepthCloudCallback: Failed to get relative transform.");
    return boost::none;
  }

  if (!CovarianceSane(relative_transform->second)) {
    LogWarning("DepthCloudCallback: Sanity check failed - invalid covariance.");
    return boost::none;
  }

  if (params_.frame_change_transform) {
    relative_transform->first = params_.body_T_haz_cam * relative_transform->first * params_.body_T_haz_cam.inverse();
    // TODO: rotate covariance matrix!!!! use exp map jacobian!!! sandwich withthis! (translation should be rotated by
    // rotation matrix)
  }

  // LogError("cov: " << std::endl << relative_transform->second.matrix());
  if (relative_transform->first.translation().norm() > 0.5) LogError("large position jump!!");
  latest_relative_transform_ = relative_transform->first;
  return relative_transform;
}

void DepthOdometry::DepthImageCallback(const cv::Mat& depth_image) {}

bool DepthOdometry::CovarianceSane(const Eigen::Matrix<double, 6, 6>& covariance) const {
  const auto position_covariance_norm = covariance.block<3, 3>(0, 0).diagonal().norm();
  const auto orientation_covariance_norm = covariance.block<3, 3>(3, 3).diagonal().norm();
  LogError("pcov: " << position_covariance_norm << ", ocov: " << orientation_covariance_norm);
  return (position_covariance_norm <= params_.position_covariance_threshold &&
          orientation_covariance_norm <= params_.orientation_covariance_threshold);
}
}  // namespace depth_odometry
