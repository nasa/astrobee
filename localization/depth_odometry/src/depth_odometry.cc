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
#include <depth_odometry/point_cloud_utilities.h>
#include <depth_odometry/parameter_reader.h>
#include <localization_common/logger.h>
#include <localization_common/timer.h>
#include <localization_common/utilities.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;

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
  depth_image_aligner_.reset(new DepthImageAligner(params_.depth_image_aligner));
  icp_.reset(new ICP(params_.icp));
}

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> DepthOdometry::DepthCloudCallback(
  std::pair<lc::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> depth_cloud) {
  RemoveNansAndZerosFromPointXYZs(*(depth_cloud.second));
  if (!previous_depth_cloud_.second && !latest_depth_cloud_.second) {
    latest_depth_cloud_ = depth_cloud;
    return boost::none;
  }
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

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> DepthOdometry::DepthImageCallback(
  const lm::DepthImageMeasurement& depth_image) {
  if (!previous_depth_image_ && !latest_depth_image_) {
    latest_depth_image_ = depth_image;
    return boost::none;
  }
  if (depth_image.timestamp < latest_depth_image_->timestamp) {
    LogWarning("DepthImageCallback: Out of order measurement received.");
    return boost::none;
  }
  LogError("t: " << std::setprecision(15) << depth_image.timestamp);
  previous_depth_image_ = latest_depth_image_;
  latest_depth_image_ = depth_image;
  depth_image_aligner_->AddLatestImage(latest_depth_image_->intensities, latest_depth_image_->timestamp);
  auto relative_transform = depth_image_aligner_->ComputeRelativeTransform();
  if (!relative_transform) {
    LogWarning("DepthImageCallback: Failed to get relative transform.");
    return boost::none;
  }

  if (params_.frame_change_transform) {
    relative_transform->first = params_.body_T_haz_cam * relative_transform->first * params_.body_T_haz_cam.inverse();
  }

  // LogError("cov: " << std::endl << relative_transform->second.matrix());
  // if (relative_transform->first.translation().norm() > 0.5) LogError("large position jump!!");
  // latest_relative_transform_ = relative_transform->first;
  return relative_transform;
}

bool DepthOdometry::CovarianceSane(const Eigen::Matrix<double, 6, 6>& covariance) const {
  const auto position_covariance_norm = covariance.block<3, 3>(0, 0).diagonal().norm();
  const auto orientation_covariance_norm = covariance.block<3, 3>(3, 3).diagonal().norm();
  LogError("pcov: " << position_covariance_norm << ", ocov: " << orientation_covariance_norm);
  return (position_covariance_norm <= params_.position_covariance_threshold &&
          orientation_covariance_norm <= params_.orientation_covariance_threshold);
}
}  // namespace depth_odometry
