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

#include "test_utilities.h"  // NOLINT
#include <depth_odometry/point_to_plane_icp_depth_odometry_params.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <point_cloud_common/test_utilities.h>

#include <pcl/common/transforms.h>

#include <cv_bridge/cv_bridge.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace pc = point_cloud_common;

lm::DepthImageMeasurement DefaultDepthImageMeasurement(const lc::Time timestamp) {
  const auto cubic_points = pc::CubicPoints();
  const auto point_cloud = pc::PointCloud<pcl::PointXYZI>(cubic_points.first);
  return lm::DepthImageMeasurement(cv::Mat(), point_cloud, timestamp);
}

lm::DepthImageMeasurement TransformDepthImageMeasurement(const lm::DepthImageMeasurement& depth_image_measurement,
                                                         const lc::Time timestamp,
                                                         const Eigen::Isometry3d& target_T_source) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*(depth_image_measurement.depth_image.unfiltered_point_cloud()), *transformed_cloud,
                           Eigen::Affine3d(target_T_source.matrix()));
  return lm::DepthImageMeasurement(depth_image_measurement.depth_image.image(), transformed_cloud, timestamp);
}

sensor_msgs::PointCloud2ConstPtr CubicPointsMsg(const lc::Time timestamp) {
  const auto cubic_points = pc::CubicPoints();
  const auto point_cloud = pc::PointCloud<pcl::PointXYZ>(cubic_points.first);
  sensor_msgs::PointCloud2 msg;
  lc::TimeToHeader(timestamp, msg.header);
  pcl::toROSMsg(*point_cloud, msg);
  return sensor_msgs::PointCloud2ConstPtr(new sensor_msgs::PointCloud2(msg));
}

sensor_msgs::ImageConstPtr ImageMsg(const lc::Time timestamp) {
  cv_bridge::CvImage msg_bridge;
  msg_bridge.encoding = sensor_msgs::image_encodings::MONO8;
  msg_bridge.image = cv::Mat(10, 10, CV_8UC1);
  auto msg = msg_bridge.toImageMsg();
  lc::TimeToHeader(timestamp, msg->header);
  return sensor_msgs::ImageConstPtr(msg);
}

PointToPlaneICPDepthOdometryParams DefaultPointToPlaneICPDepthOdometryParams() {
  PointToPlaneICPDepthOdometryParams params;
  params.icp = pc::DefaultPointToPlaneICPParams();
  DefaultDepthOdometryParams(params);
  return params;
}

void DefaultDepthOdometryParams(DepthOdometryParams& params) {
  params.max_time_diff = 1.0;
  params.position_covariance_threshold = 1.0;
  params.orientation_covariance_threshold = 1.0;
}
}  // namespace depth_odometry
