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
  config.AddFile("cameras.config");
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

boost::optional<lc::PoseWithCovariance> DepthOdometry::DepthImageCallback(
  const lm::DepthImageMeasurement& depth_image_measurement) {
  // TODO(rsoussan): Ensure only one of these is enabled
  if (params_.depth_point_cloud_registration_enabled) return PointCloudCallback(depth_image_measurement);
  if (params_.depth_image_registration_enabled) return ImageCallback(depth_image_measurement);
  return boost::none;
}

boost::optional<lc::PoseWithCovariance> DepthOdometry::PointCloudCallback(
  const lm::DepthImageMeasurement& depth_image_measurement) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*(depth_image_measurement.point_cloud), *filtered_cloud);
  RemoveNansAndZerosFromPoints(*filtered_cloud);
  if (!previous_depth_cloud_.second && !latest_depth_cloud_.second) {
    latest_depth_cloud_ = std::make_pair(depth_image_measurement.timestamp, filtered_cloud);
    return boost::none;
  }
  const lc::Time timestamp = depth_image_measurement.timestamp;
  if (timestamp < latest_depth_cloud_.first) {
    LogWarning("PointCloudCallback: Out of order measurement received.");
    return boost::none;
  }
  previous_depth_cloud_ = latest_depth_cloud_;
  latest_depth_cloud_ = std::make_pair(depth_image_measurement.timestamp, filtered_cloud);

  const double time_diff = latest_depth_cloud_.first - previous_depth_cloud_.first;
  if (time_diff > params_.max_time_diff) {
    LogWarning("PointCloudCallback: Time difference too large, time diff: " << time_diff);
    return boost::none;
  }
  auto relative_transform = icp_->ComputeRelativeTransform(previous_depth_cloud_.second, latest_depth_cloud_.second);
  if (!relative_transform) {
    LogWarning("PointCloudCallback: Failed to get relative transform.");
    return boost::none;
  }

  if (!CovarianceSane(relative_transform->covariance)) {
    LogWarning("PointCloudCallback: Sanity check failed - invalid covariance.");
    return boost::none;
  }

  /*if (params_.frame_change_transform) {
    relative_transform->first = params_.body_T_haz_cam * relative_transform->first * params_.body_T_haz_cam.inverse();
    // TODO: rotate covariance matrix!!!! use exp map jacobian!!! sandwich withthis! (translation should be rotated by
    // rotation matrix)
  }*/

  // LogError("cov: " << std::endl << relative_transform->second.matrix());
  // if (relative_transform->pose.translation().norm() > 0.5) LogError("large position jump!!");
  latest_relative_transform_ = relative_transform->pose;
  return relative_transform;
}

boost::optional<lc::PoseWithCovariance> DepthOdometry::ImageCallback(const lm::DepthImageMeasurement& depth_image) {
  if (!previous_depth_image_ && !latest_depth_image_) {
    latest_depth_image_ = depth_image;
    return boost::none;
  }
  if (depth_image.timestamp < latest_depth_image_->timestamp) {
    LogWarning("ImageCallback: Out of order measurement received.");
    return boost::none;
  }
  previous_depth_image_ = latest_depth_image_;
  latest_depth_image_ = depth_image;
  const double time_diff = latest_depth_image_->timestamp - previous_depth_image_->timestamp;
  if (time_diff > params_.max_time_diff) {
    LogWarning("ImageCallback: Time difference too large, time diff: " << time_diff);
    return boost::none;
  }

  depth_image_aligner_->AddLatestDepthImage(*latest_depth_image_);
  auto relative_transform = depth_image_aligner_->ComputeRelativeTransform();
  if (!relative_transform) {
    LogError("ImageCallback: Failed to get relative transform.");
    return boost::none;
  }
  /*if (params_.frame_change_transform) {
    LogError("do bTh: " << std::endl << params_.body_T_haz_cam.matrix());
    LogError("do rel trafo pre change: " << std::endl << relative_transform->first.matrix());
    relative_transform->first = params_.body_T_haz_cam * relative_transform->first * params_.body_T_haz_cam.inverse();
  }*/

  // LogError("cov: " << std::endl << relative_transform->second.matrix());
  // if (relative_transform->pose.translation().norm() > 0.5) LogError("large position jump!!");
  // latest_relative_transform_ = relative_transform->first;
  return relative_transform;
}

bool DepthOdometry::CovarianceSane(const Eigen::Matrix<double, 6, 6>& covariance) const {
  const auto position_covariance_norm = covariance.block<3, 3>(0, 0).diagonal().norm();
  const auto orientation_covariance_norm = covariance.block<3, 3>(3, 3).diagonal().norm();
  // LogError("pcov: " << position_covariance_norm << ", ocov: " << orientation_covariance_norm);
  return (position_covariance_norm <= params_.position_covariance_threshold &&
          orientation_covariance_norm <= params_.orientation_covariance_threshold);
}
}  // namespace depth_odometry
