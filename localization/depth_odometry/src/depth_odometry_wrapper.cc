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
#include <msg_conversions/msg_conversions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/image_encodings.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

std::vector<ff_msgs::Odometry> DepthOdometryWrapper::DepthCloudCallback(
  const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg) {
  point_cloud_buffer_.Add(lc::TimeFromHeader(depth_cloud_msg->header), depth_cloud_msg);
  return ProcessDepthImageAndCloudMeasurementsIfAvailable();
}

std::vector<ff_msgs::Odometry> DepthOdometryWrapper::DepthImageCallback(
  const sensor_msgs::ImageConstPtr& depth_image_msg) {
  image_buffer_.Add(lc::TimeFromHeader(depth_image_msg->header), depth_image_msg);
  return ProcessDepthImageAndCloudMeasurementsIfAvailable();
}

std::vector<ff_msgs::Odometry> DepthOdometryWrapper::ProcessDepthImageAndCloudMeasurementsIfAvailable() {
  std::vector<lm::DepthImageMeasurement> depth_image_measurements;
  boost::optional<lc::Time> latest_added_point_cloud_msg_time;
  boost::optional<lc::Time> latest_added_image_msg_time;
  // Point clouds and depth images for the same measurement arrive on different topics.
  // Correlate pairs of these if possible.
  for (const auto& depth_image_msg : image_buffer_.measurements()) {
    const auto depth_image_msg_timestamp = depth_image_msg.first;
    const auto point_cloud_msg = point_cloud_buffer_.GetNearby(
      depth_image_msg_timestamp, depth_odometry_.params().max_image_and_point_cloud_time_diff);
    if (point_cloud_msg) {
      const auto depth_image_measurement = lm::MakeDepthImageMeasurement(*point_cloud_msg, depth_image_msg.second,
                                                                         depth_odometry_.params().haz_cam_A_haz_depth);
      if (!depth_image_measurement) {
        LogError("ProcessDepthImageAndCloudMeasurementsIfAvailable: Failed to create depth image measurement.");
        continue;
      }
      depth_image_measurements.emplace_back(*depth_image_measurement);
      latest_added_point_cloud_msg_time = lc::TimeFromHeader((*point_cloud_msg)->header);
      latest_added_image_msg_time = depth_image_msg_timestamp;
    }
  }

  if (latest_added_point_cloud_msg_time) point_cloud_buffer_.EraseIncluding(*latest_added_point_cloud_msg_time);
  if (latest_added_image_msg_time) image_buffer_.EraseIncluding(*latest_added_image_msg_time);

  std::vector<ff_msgs::Odometry> relative_pose_msgs;
  for (const auto& depth_image_measurement : depth_image_measurements) {
    auto relative_transform = depth_odometry_.DepthImageCallback(depth_image_measurement);
    if (relative_transform) {
      // TODO(rsoussan): Make function that does this, use new PoseWithCovariance type (add both to loc common)
      ff_msgs::Odometry pose_msg;
      const Eigen::Isometry3d body_F_a_T_b = depth_odometry_.params().body_T_haz_cam * relative_transform->pose *
                                             depth_odometry_.params().body_T_haz_cam.inverse();
      // TODO: rotate covariance matrix!!!! use exp map jacobian!!! sandwich withthis! (translation should be rotated
      // by rotation matrix)
      mc::EigenPoseCovarianceToMsg(relative_transform->pose, relative_transform->covariance, pose_msg.sensor_F_a_T_b);
      mc::EigenPoseCovarianceToMsg(body_F_a_T_b, relative_transform->covariance, pose_msg.body_F_a_T_b);
      lc::TimeToHeader(depth_image_measurement.timestamp, pose_msg.header);
      relative_pose_msgs.emplace_back(pose_msg);
    }
  }
  return relative_pose_msgs;
}

boost::optional<ff_msgs::DepthCorrespondences> DepthOdometryWrapper::GetPointCloudCorrespondencesMsg() const {
  const auto& correspondences = depth_odometry_.point_cloud_correspondences();
  if (!correspondences) return boost::none;
  const auto previous_time = depth_odometry_.previous_depth_cloud().first;
  const auto latest_time = depth_odometry_.latest_depth_cloud().first;
  ff_msgs::DepthCorrespondences correspondences_msg;
  lc::TimeToHeader(latest_time, correspondences_msg.header);
  correspondences_msg.header.frame_id = "haz_cam";
  correspondences_msg.source_time = ros::Time(previous_time);
  correspondences_msg.target_time = ros::Time(latest_time);

  for (const auto& correspondence : *correspondences) {
    ff_msgs::DepthCorrespondence correspondence_msg;
    // TODO(rsoussan): Fill this in!!!
    // correspondence_msg.source_index = correspondence.index_query;
    // correspondence_msg.target_index = correspondence.index_match;
    correspondences_msg.correspondences.push_back(correspondence_msg);
  }
  return correspondences_msg;
}

boost::optional<ff_msgs::DepthCorrespondences> DepthOdometryWrapper::GetImageCorrespondencesMsg() const {
  const auto& correspondences = depth_odometry_.image_correspondences();
  if (!correspondences) return boost::none;
  ff_msgs::DepthCorrespondences correspondences_msg;
  lc::TimeToHeader(correspondences->target_time, correspondences_msg.header);
  correspondences_msg.header.frame_id = "haz_cam";
  correspondences_msg.source_time = ros::Time(correspondences->source_time);
  correspondences_msg.target_time = ros::Time(correspondences->target_time);

  for (int i = 0; i < static_cast<int>(correspondences->source_image_points.size()); ++i) {
    ff_msgs::DepthCorrespondence correspondence_msg;
    mc::Vector2dToMsg(correspondences->source_image_points[i], correspondence_msg.source_image_point);
    mc::Vector2dToMsg(correspondences->target_image_points[i], correspondence_msg.target_image_point);
    mc::VectorToMsg(correspondences->source_3d_points[i], correspondence_msg.source_3d_point);
    mc::VectorToMsg(correspondences->target_3d_points[i], correspondence_msg.target_3d_point);
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
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  const Eigen::Isometry3d latest_relative_transform = depth_odometry_.latest_relative_transform();
  pcl::transformPointCloud(*(depth_odometry_.latest_depth_cloud().second), *transformed_cloud,
                           latest_relative_transform.matrix().cast<float>());
  return lm::MakePointCloudMsg(*transformed_cloud, depth_odometry_.latest_depth_cloud().first, "haz_cam");
}
}  // namespace depth_odometry
