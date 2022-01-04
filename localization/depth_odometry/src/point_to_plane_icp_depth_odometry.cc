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
namespace pc = point_cloud_common;

PointToPlaneICPDepthOdometry::PointToPlaneICPDepthOdometry(const PointToPlaneICPDepthOdometryParams& params)
    : params_(params), icp_(params.icp) {}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr PointToPlaneICPDepthOdometry::DownsampleAndFilterCloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
  if (params_.use_organized_normal_estimation) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>(*cloud));
    pc::ReplaceZerosWithNans(*filtered_cloud);
    pc::EstimateOrganizedNormals<pcl::PointXYZI, pcl::PointXYZINormal>(
      filtered_cloud, params_.max_depth_change_factor, params_.normal_smoothing_size, *filtered_cloud_with_normals);
    pc::RemoveInvalidAndZeroPoints(*filtered_cloud_with_normals);
  } else {
    // Only downsample if not doing organized normal estimation
    if (params_.downsample) {
      const auto downsampled_cloud = pc::DownsamplePointCloud<pcl::PointXYZI>(cloud, params_.downsample_leaf_size);
      filtered_cloud_with_normals = pc::FilteredPointCloudWithNormals<pcl::PointXYZI, pcl::PointXYZINormal>(
        downsampled_cloud, params_.icp.search_radius);
    } else {
      filtered_cloud_with_normals =
        pc::FilteredPointCloudWithNormals<pcl::PointXYZI, pcl::PointXYZINormal>(cloud, params_.icp.search_radius);
    }
  }
  if (params_.use_normal_space_sampling) {
    pc::NormalSpaceSubsampling<pcl::PointXYZINormal>(filtered_cloud_with_normals, params_.bins_per_axis,
                                                     params_.num_samples);
  }
  return filtered_cloud_with_normals;
}

boost::optional<PoseWithCovarianceAndCorrespondences> PointToPlaneICPDepthOdometry::DepthImageCallback(
  const lm::DepthImageMeasurement& depth_image_measurement) {
  return DepthImageCallbackWithEstimate(depth_image_measurement);
}

boost::optional<PoseWithCovarianceAndCorrespondences> PointToPlaneICPDepthOdometry::DepthImageCallbackWithEstimate(
  const localization_measurements::DepthImageMeasurement& depth_image_measurement,
  const boost::optional<Eigen::Isometry3d&> target_T_source_initial_estimate) {
  if (!previous_point_cloud_with_normals_ && !latest_point_cloud_with_normals_) {
    latest_point_cloud_with_normals_ =
      DownsampleAndFilterCloud(depth_image_measurement.depth_image.unfiltered_point_cloud());
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
  latest_point_cloud_with_normals_ =
    DownsampleAndFilterCloud(depth_image_measurement.depth_image.unfiltered_point_cloud());
  latest_timestamp_ = timestamp;

  const double time_diff = latest_timestamp_ - previous_timestamp_;
  if (time_diff > params_.max_time_diff) {
    LogWarning("DepthImageCallback: Time difference too large, time diff: " << time_diff);
    return boost::none;
  }

  if (target_T_source_initial_estimate) {
    pcl::transformPointCloudWithNormals(*previous_point_cloud_with_normals_, *previous_point_cloud_with_normals_,
                                        target_T_source_initial_estimate->matrix());
  }
  auto target_T_source =
    icp_.ComputeRelativeTransform(previous_point_cloud_with_normals_, latest_point_cloud_with_normals_);
  if (!target_T_source) {
    LogWarning("DepthImageCallback: Failed to get relative transform.");
    return boost::none;
  }

  if (target_T_source_initial_estimate) {
    target_T_source->pose = target_T_source->pose * *target_T_source_initial_estimate;
    // TODO(rsoussan): Frame change covariance!
  }

  const auto source_T_target = lc::InvertPoseWithCovariance(*target_T_source);

  if (!lc::PoseCovarianceSane(source_T_target.covariance, params_.position_covariance_threshold,
                              params_.orientation_covariance_threshold)) {
    LogWarning("DepthImageCallback: Sanity check failed - invalid covariance.");
    return boost::none;
  }

  const auto correspondences = icp_.correspondences();
  if (!correspondences) {
    LogWarning("DepthImageCallback: Failed to get correspondences.");
    return boost::none;
  }

  return PoseWithCovarianceAndCorrespondences(source_T_target, *correspondences, previous_timestamp_,
                                              latest_timestamp_);
}
}  // namespace depth_odometry
