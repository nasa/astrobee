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

#ifndef LOCALIZATION_MEASUREMENTS_MEASUREMENT_CONVERSIONS_H_
#define LOCALIZATION_MEASUREMENTS_MEASUREMENT_CONVERSIONS_H_

#include <ff_msgs/DepthLandmarks.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/VisualLandmarks.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_measurements/depth_image_measurement.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/feature_points_measurement.h>
#include <localization_measurements/handrail_points_measurement.h>
#include <localization_measurements/image_measurement.h>
#include <localization_measurements/imu_measurement.h>
#include <localization_measurements/matched_projections_measurement.h>
#include <localization_measurements/plane.h>
#include <localization_measurements/point_cloud_measurement.h>
#include <localization_measurements/timestamped_handrail_pose.h>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <utility>

namespace localization_measurements {
MatchedProjectionsMeasurement MakeMatchedProjectionsMeasurement(const ff_msgs::VisualLandmarks& visual_landmarks);

HandrailPointsMeasurement MakeHandrailPointsMeasurement(const ff_msgs::DepthLandmarks& depth_landmarks,
                                                        const TimestampedHandrailPose& world_T_handrail);

Plane MakeHandrailPlane(const gtsam::Pose3& world_T_handrail, const double distance_to_wall);

std::pair<gtsam::Point3, gtsam::Point3> MakeHandrailEndpoints(const gtsam::Pose3& world_T_handrail,
                                                              const double length);

MatchedProjectionsMeasurement FrameChangeMatchedProjectionsMeasurement(
  const MatchedProjectionsMeasurement& matched_projections_measurement,
  const gtsam::Pose3& new_frame_T_measurement_origin);

FeaturePointsMeasurement MakeFeaturePointsMeasurement(const ff_msgs::Feature2dArray& optical_flow_tracks);

FanSpeedMode ConvertFanSpeedMode(const uint8_t speed);

boost::optional<ImageMeasurement> MakeImageMeasurement(const sensor_msgs::ImageConstPtr& image_msg,
                                                       const std::string& encoding);

PointCloudMeasurement MakePointCloudMeasurement(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg);

boost::optional<DepthImageMeasurement> MakeDepthImageMeasurement(
  const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg, const sensor_msgs::ImageConstPtr& image_msg,
  const Eigen::Isometry3d image_T_depth_cam = Eigen::Isometry3d::Identity());

// TODO(rsoussan): Move this somewhere else?
template <typename PointType>
sensor_msgs::PointCloud2 MakePointCloudMsg(const pcl::PointCloud<PointType>& cloud,
                                           const localization_common::Time timestamp, const std::string frame) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  localization_common::TimeToHeader(timestamp, cloud_msg.header);
  cloud_msg.header.frame_id = "haz_cam";
  return cloud_msg;
}
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_MEASUREMENT_CONVERSIONS_H_
