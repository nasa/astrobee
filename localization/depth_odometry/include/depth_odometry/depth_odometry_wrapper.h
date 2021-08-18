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
#ifndef DEPTH_ODOMETRY_DEPTH_ODOMETRY_WRAPPER_H_
#define DEPTH_ODOMETRY_DEPTH_ODOMETRY_WRAPPER_H_

#include <depth_odometry/depth_odometry.h>
#include <ff_msgs/ImageCorrespondences.h>
#include <ff_msgs/PointCloudCorrespondences.h>
#include <localization_common/measurement_buffer.h>
#include <localization_common/time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace depth_odometry {
class DepthOdometryWrapper {
 public:
  std::vector<geometry_msgs::PoseWithCovarianceStamped> DepthCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg);
  std::vector<geometry_msgs::PoseWithCovarianceStamped> DepthImageCallback(
    const sensor_msgs::ImageConstPtr& depth_image_msg);
  ff_msgs::PointCloudCorrespondences GetPointCloudCorrespondencesMsg() const;
  ff_msgs::ImageCorrespondences GetDepthImageCorrespondencesMsg() const;
  sensor_msgs::PointCloud2 GetPreviousPointCloudMsg() const;
  sensor_msgs::PointCloud2 GetLatestPointCloudMsg() const;
  sensor_msgs::PointCloud2 GetTransformedPreviousPointCloudMsg() const;

 private:
  std::vector<geometry_msgs::PoseWithCovarianceStamped> ProcessDepthImageAndCloudMeasurementsIfAvailable();

  DepthOdometry depth_odometry_;
  localization_common::MeasurementBuffer<sensor_msgs::PointCloud2ConstPtr> point_cloud_buffer_;
  localization_common::MeasurementBuffer<sensor_msgs::ImageConstPtr> image_buffer_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_ODOMETRY_WRAPPER_H_
