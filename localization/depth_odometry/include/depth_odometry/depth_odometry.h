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
#ifndef DEPTH_ODOMETRY_DEPTH_ODOMETRY_H_
#define DEPTH_ODOMETRY_DEPTH_ODOMETRY_H_

#include <depth_odometry/depth_image_aligner.h>
#include <depth_odometry/depth_odometry_params.h>
#include <localization_common/pose_with_covariance.h>
#include <localization_common/time.h>
#include <localization_measurements/depth_image_measurement.h>
#include <localization_measurements/image_measurement.h>
#include <point_cloud_common/icp.h>

#include <boost/optional.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <utility>

namespace depth_odometry {
class DepthOdometry {
 public:
  DepthOdometry();
  boost::optional<localization_common::PoseWithCovariance> DepthImageCallback(
    const localization_measurements::DepthImageMeasurement& depth_image);
  std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZI>::Ptr> previous_depth_cloud() const {
    return previous_depth_cloud_;
  }
  std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZI>::Ptr> latest_depth_cloud() const {
    return latest_depth_cloud_;
  }
  const boost::optional<pcl::Correspondences>& point_cloud_correspondences() const { return icp_->correspondences(); }
  const boost::optional<DepthMatches>& image_correspondences() const { return depth_image_aligner_->matches(); }
  Eigen::Isometry3d latest_relative_transform() const { return latest_relative_transform_; }
  const DepthOdometryParams& params() const { return params_; }

 private:
  boost::optional<localization_common::PoseWithCovariance> GetPointCloudAlignerRelativeTransform(
    const localization_measurements::DepthImageMeasurement& depth_image);
  boost::optional<localization_common::PoseWithCovariance> GetDepthImageAlignerRelativeTransform(
    const localization_measurements::DepthImageMeasurement& depth_image);

  bool CovarianceSane(const Eigen::Matrix<double, 6, 6>& covariance) const;

  std::unique_ptr<point_cloud_common::ICP> icp_;
  std::unique_ptr<DepthImageAligner> depth_image_aligner_;
  std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZI>::Ptr> previous_depth_cloud_;
  std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZI>::Ptr> latest_depth_cloud_;
  boost::optional<localization_measurements::DepthImageMeasurement> previous_depth_image_;
  boost::optional<localization_measurements::DepthImageMeasurement> latest_depth_image_;
  DepthOdometryParams params_;
  Eigen::Isometry3d latest_relative_transform_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_ODOMETRY_H_
