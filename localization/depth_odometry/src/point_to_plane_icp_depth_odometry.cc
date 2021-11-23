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

#include <depth_odometry/point_to_plane_icp_depth_odometry.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <point_cloud_common/utilities.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace pcc = point_cloud_common;

PointToPlaneICPDepthOdometry::PointToPlaneICPDepthOdometry(const PointToPlaneICPDepthOdometryParams& params)
    : params_(params) {
  icp_.reset(new pcc::PointToPlaneICP(params_.icp));
}

boost::optional<PoseWithCovarianceAndCorrespondences> PointToPlaneICPDepthOdometry::DepthImageCallback(
  const lm::DepthImageMeasurement& depth_image_measurement) {
  if (!previous_point_cloud_with_normals_ && !latest_point_cloud_with_normals_) {
    latest_point_cloud_with_normals_ = pcc::FilteredPointCloudWithNormals<pcl::PointXYZI, pcl::PointXYZINormal>(
      depth_image_measurement.depth_image.unfiltered_point_cloud(), params_.icp.search_radius);
    latest_timestamp_ = depth_image_measurement.timestamp;
    return boost::none;
  }
  const lc::Time timestamp = depth_image_measurement.timestamp;
  if (timestamp < latest_timestamp_) {
    LogWarning("DepthImageCallback: Out of order measurement received.");
    return boost::none;
  }

  previous_point_cloud_with_normals_ = latest_point_cloud_with_normals_;
  previous_timestamp_ = latest_timestamp_;
  latest_point_cloud_with_normals_ = pcc::FilteredPointCloudWithNormals<pcl::PointXYZI, pcl::PointXYZINormal>(
    depth_image_measurement.depth_image.unfiltered_point_cloud(), params_.icp.search_radius);
  latest_timestamp_ = timestamp;

  const double time_diff = latest_timestamp_ - previous_timestamp_;
  if (time_diff > params_.max_time_diff) {
    LogWarning("DepthImageCallback: Time difference too large, time diff: " << time_diff);
    return boost::none;
  }
  auto relative_transform =
    icp_->ComputeRelativeTransform(previous_point_cloud_with_normals_, latest_point_cloud_with_normals_);
  if (!relative_transform) {
    LogWarning("DepthImageCallback: Failed to get relative transform.");
    return boost::none;
  }

  if (!lc::PoseCovarianceSane(relative_transform->covariance, params_.position_covariance_threshold,
                              params_.orientation_covariance_threshold)) {
    LogWarning("DepthImageCallback: Sanity check failed - invalid covariance.");
    return boost::none;
  }

  const auto correspondences = icp_->correspondences();
  if (!correspondences) {
    LogWarning("DepthImageCallback: Failed to get correspondences.");
    return boost::none;
  }

  return PoseWithCovarianceAndCorrespondences(*relative_transform, *correspondences);
}
}  // namespace depth_odometry
