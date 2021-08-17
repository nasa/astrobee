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
#include <depth_odometry/depth_odometry_wrapper.h>
#include <ff_msgs/DepthCorrespondences.h>
#include <ff_util/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/image_encodings.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

boost::optional<geometry_msgs::PoseWithCovarianceStamped> DepthOdometryWrapper::DepthCloudCallback(
  const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg) {
  if (!depth_odometry_.params().depth_point_cloud_registration_enabled) return boost::none;
  const lc::Time timestamp = lc::TimeFromHeader(depth_cloud_msg->header);
  std::pair<lc::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> depth_cloud{
    timestamp, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>())};
  pcl::fromROSMsg(*depth_cloud_msg, *(depth_cloud.second));
  const auto relative_pose = depth_odometry_.DepthCloudCallback(depth_cloud);
  if (!relative_pose) {
    LogError("DepthCloudCallback: Failed to get relative pose.");
    return boost::none;
  }

  // LogError("rel pose: " << std::endl << relative_pose->first.matrix());

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  mc::EigenPoseCovarianceToMsg(relative_pose->first, relative_pose->second, pose_msg);
  lc::TimeToHeader(depth_cloud.first, pose_msg.header);
  return pose_msg;
}

boost::optional<geometry_msgs::PoseWithCovarianceStamped> DepthOdometryWrapper::DepthImageCallback(
  const sensor_msgs::ImageConstPtr& depth_image_msg) {
  if (!depth_odometry_.params().depth_image_registration_enabled) return boost::none;
  const auto depth_image = lm::MakeImageMeasurement(depth_image_msg, sensor_msgs::image_encodings::MONO16);
  if (!depth_image) {
    LogError("DepthImageCallback: Failed to make depth image.");
    return boost::none;
  }

  const auto relative_pose = depth_odometry_.DepthImageCallback(*depth_image);
  if (!relative_pose) {
    LogError("DepthImageCallback: Failed to get relative pose.");
    return boost::none;
  }

  // TODO(rsoussan): Make function that does this, use new PoseWithCovariance type (add both to loc common)
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  mc::EigenPoseCovarianceToMsg(relative_pose->first, relative_pose->second, pose_msg);
  lc::TimeToHeader(depth_image->timestamp, pose_msg.header);
  return pose_msg;
}

ff_msgs::PointCloudCorrespondences DepthOdometryWrapper::GetPointCloudCorrespondencesMsg() const {
  const auto& correspondences = depth_odometry_.point_cloud_correspondences();
  const auto previous_time = depth_odometry_.previous_depth_cloud().first;
  const auto latest_time = depth_odometry_.latest_depth_cloud().first;
  ff_msgs::PointCloudCorrespondences correspondences_msg;
  lc::TimeToHeader(latest_time, correspondences_msg.header);
  correspondences_msg.header.frame_id = "haz_cam";
  correspondences_msg.source_time = ros::Time(previous_time);
  correspondences_msg.target_time = ros::Time(latest_time);

  for (const auto& correspondence : correspondences) {
    ff_msgs::PointCloudCorrespondence correspondence_msg;
    correspondence_msg.source_index = correspondence.index_query;
    correspondence_msg.target_index = correspondence.index_match;
    correspondences_msg.correspondences.push_back(correspondence_msg);
  }
  return correspondences_msg;
}

ff_msgs::ImageCorrespondences DepthOdometryWrapper::GetDepthImageCorrespondencesMsg() const {
  const auto& correspondences = depth_odometry_.image_correspondences();
  ff_msgs::ImageCorrespondences correspondences_msg;
  lc::TimeToHeader(correspondences.target_time(), correspondences_msg.header);
  correspondences_msg.header.frame_id = "haz_cam";
  correspondences_msg.source_time = ros::Time(correspondences.source_time());
  correspondences_msg.target_time = ros::Time(correspondences.target_time());

  for (const auto& correspondence : correspondences.correspondences()) {
    ff_msgs::ImageCorrespondence correspondence_msg;
    correspondence_msg.source_point.x = correspondence.source_point.x();
    correspondence_msg.source_point.y = correspondence.source_point.y();
    correspondence_msg.target_point.x = correspondence.target_point.x();
    correspondence_msg.target_point.y = correspondence.target_point.y();
    correspondences_msg.correspondences.push_back(correspondence_msg);
  }
  return correspondences_msg;
}

sensor_msgs::PointCloud2 DepthOdometryWrapper::GetPreviousPointCloudMsg() const {
  // Use latest timestamp so point clouds can be visualized at same time
  return lm::MakePointCloudMsg(*(depth_odometry_.previous_depth_cloud().second),
                               depth_odometry_.latest_depth_cloud().first, "haz_cam");
}

sensor_msgs::PointCloud2 DepthOdometryWrapper::GetLatestPointCloudMsg() const {
  return lm::MakePointCloudMsg(*(depth_odometry_.latest_depth_cloud().second),
                               depth_odometry_.latest_depth_cloud().first, "haz_cam");
}

sensor_msgs::PointCloud2 DepthOdometryWrapper::GetTransformedPreviousPointCloudMsg() const {
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  const Eigen::Isometry3d latest_relative_transform = depth_odometry_.latest_relative_transform();
  pcl::transformPointCloud(*(depth_odometry_.latest_depth_cloud().second), *transformed_cloud,
                           latest_relative_transform.matrix().cast<float>());
  return lm::MakePointCloudMsg(*transformed_cloud, depth_odometry_.latest_depth_cloud().first, "haz_cam");
}
}  // namespace depth_odometry
