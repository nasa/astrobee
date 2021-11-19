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

#include <depth_odometry/icp_depth_odometry.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <point_cloud_common/utilities.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace pcc = point_cloud_common;

DepthOdometry::DepthOdometry(const DepthOdometryParams& params) : params_(params) {
  icp_.reset(new pcc::ICP(params_.icp));
}

boost::optional<lc::PoseWithCovarianceAndMatches> DepthOdometry::DepthImageCallback(
  const lm::DepthImageMeasurement& depth_image_measurement) {
  // TODO(rsoussan): add pointcloudwithnormals? store previous and latest as member vars!
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*(depth_image_measurement.point_cloud), *filtered_cloud);
  pcc::RemoveNansAndZerosFromPoints(*filtered_cloud);
  if (!previous_depth_cloud_.second && !latest_depth_cloud_.second) {
    latest_depth_cloud_ = std::make_pair(depth_image_measurement.timestamp, filtered_cloud);
    return boost::none;
  }
  const lc::Time timestamp = depth_image_measurement.timestamp;
  if (timestamp < latest_depth_cloud_.first) {
    LogWarning("GetPointCloudAlignerRelativeTransform: Out of order measurement received.");
    return boost::none;
  }
  previous_depth_cloud_ = latest_depth_cloud_;
  latest_depth_cloud_ = std::make_pair(depth_image_measurement.timestamp, filtered_cloud);

  const double time_diff = latest_depth_cloud_.first - previous_depth_cloud_.first;
  if (time_diff > params_.max_time_diff) {
    LogWarning("GetPointCloudAlignerRelativeTransform: Time difference too large, time diff: " << time_diff);
    return boost::none;
  }
  auto relative_transform = icp_->ComputeRelativeTransform(previous_depth_cloud_.second, latest_depth_cloud_.second);
  if (!relative_transform) {
    LogWarning("GetPointCloudAlignerRelativeTransform: Failed to get relative transform.");
    return boost::none;
  }

  if (!CovarianceSane(relative_transform->covariance)) {
    LogWarning("GetPointCloudAlignerRelativeTransform: Sanity check failed - invalid covariance.");
    return boost::none;
  }

  latest_relative_transform_ = relative_transform->pose;
  return relative_transform;
}
}  // namespace depth_odometry
