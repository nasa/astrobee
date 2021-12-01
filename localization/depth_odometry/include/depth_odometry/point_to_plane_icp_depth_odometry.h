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
#ifndef DEPTH_ODOMETRY_POINT_TO_PLANE_ICP_DEPTH_ODOMETRY_H_
#define DEPTH_ODOMETRY_POINT_TO_PLANE_ICP_DEPTH_ODOMETRY_H_

#include <depth_odometry/depth_odometry.h>
#include <depth_odometry/point_to_plane_icp_depth_odometry_params.h>
#include <depth_odometry/pose_with_covariance_and_correspondences.h>
#include <point_cloud_common/point_to_plane_icp.h>

namespace depth_odometry {
class PointToPlaneICPDepthOdometry : public DepthOdometry {
 public:
  explicit PointToPlaneICPDepthOdometry(const PointToPlaneICPDepthOdometryParams& params);
  boost::optional<PoseWithCovarianceAndCorrespondences> DepthImageCallback(
    const localization_measurements::DepthImageMeasurement& depth_image) final;
  const PointToPlaneICPDepthOdometryParams& params() const { return params_; }

 private:
  point_cloud_common::PointToPlaneICP<pcl::PointXYZINormal> icp_;
  PointToPlaneICPDepthOdometryParams params_;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr previous_point_cloud_with_normals_;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr latest_point_cloud_with_normals_;
  localization_common::Time previous_timestamp_;
  localization_common::Time latest_timestamp_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_POINT_TO_PLANE_ICP_DEPTH_ODOMETRY_H_
