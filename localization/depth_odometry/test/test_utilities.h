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
#ifndef DEPTH_ODOMETRY_TEST_UTILITIES_H_  // NOLINT
#define DEPTH_ODOMETRY_TEST_UTILITIES_H_  // NOLINT

#include <depth_odometry/point_to_plane_icp_depth_odometry_params.h>
#include <depth_odometry/depth_odometry_wrapper_params.h>
#include <localization_common/time.h>
#include <localization_measurements/depth_image_measurement.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <point_cloud_common/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <vector>

namespace depth_odometry {
localization_measurements::DepthImageMeasurement DefaultDepthImageMeasurement(
  const localization_common::Time timestamp);

localization_measurements::DepthImageMeasurement ImageFeatureDepthImageMeasurement(
  const localization_common::Time timestamp, const cv::Point2i& offset = cv::Point2i(0, 0));

localization_measurements::DepthImageMeasurement TransformDepthImageMeasurement(
  const localization_measurements::DepthImageMeasurement& depth_image_measurement,
  const localization_common::Time timestamp, const Eigen::Isometry3d& target_T_source);

localization_measurements::DepthImageMeasurement OffsetImageFeatureDepthImageMeasurement(
  const localization_common::Time timestamp,
  const localization_measurements::DepthImageMeasurement& depth_image_measurement, const cv::Point2i& offset,
  const Eigen::Isometry3d& target_T_source);

std::vector<Eigen::Vector3d> RampedPoints(int cols, int rows);

sensor_msgs::PointCloud2ConstPtr RampedPointsMsg(const localization_common::Time timestamp);

sensor_msgs::PointCloud2ConstPtr CubicPointsMsg(const localization_common::Time timestamp);

sensor_msgs::ImageConstPtr ImageMsg(const localization_common::Time timestamp);

sensor_msgs::ImageConstPtr MarkerImageMsg(const localization_common::Time timestamp,
                                          const cv::Point2i& offset = cv::Point2i(0, 0));

sensor_msgs::PointCloud2ConstPtr TransformPointsMsg(const localization_common::Time timestamp,
                                                    const sensor_msgs::PointCloud2ConstPtr old_msg,
                                                    const Eigen::Isometry3d& transform);

sensor_msgs::PointCloud2ConstPtr OffsetAndTransformPointsMsg(const localization_common::Time timestamp,
                                                             const sensor_msgs::PointCloud2ConstPtr old_msg,
                                                             const cv::Point2i& offset,
                                                             const Eigen::Isometry3d& target_T_source);

PointToPlaneICPDepthOdometryParams DefaultPointToPlaneICPDepthOdometryParams();

ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams
DefaultImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams();

void DefaultDepthOdometryParams(DepthOdometryParams& params);

DepthOdometryWrapperParams DefaultDepthOdometryWrapperParams();
}  // namespace depth_odometry
#endif  // DEPTH_ODOMETRY_TEST_UTILITIES_H_ // NOLINT
