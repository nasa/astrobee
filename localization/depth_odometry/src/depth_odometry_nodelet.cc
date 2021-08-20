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

#include <depth_odometry/depth_odometry_nodelet.h>
#include <depth_odometry/parameter_reader.h>
#include <ff_msgs/DepthImageCorrespondences.h>
#include <ff_util/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace mc = msg_conversions;

DepthOdometryNodelet::DepthOdometryNodelet() : ff_util::FreeFlyerNodelet(NODE_DEPTH_ODOM, true) {
  config_reader::ConfigReader config;
  config.AddFile("localization/depth_odometry.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  LoadDepthOdometryNodeletParams(config, params_);
}

void DepthOdometryNodelet::Initialize(ros::NodeHandle* nh) { SubscribeAndAdvertise(nh); }

void DepthOdometryNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  const std::string depth_cloud_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                        static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                        static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX);
  point_cloud_sub_ = nh->subscribe<sensor_msgs::PointCloud2>(
    depth_cloud_topic, 10, &DepthOdometryNodelet::DepthCloudCallback, this, ros::TransportHints().tcpNoDelay());

  image_transport::ImageTransport image_transport(*nh);
  const std::string depth_image_topic = /*static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                  static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                  static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_DEPTH_IMAGE);*/
    "/hw/depth_haz/extended/amplitude_int";
  depth_image_sub_ = image_transport.subscribe(depth_image_topic, 10, &DepthOdometryNodelet::DepthImageCallback, this);
  odom_pub_ = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(TOPIC_LOCALIZATION_DEPTH_ODOM, 10);
  depth_image_correspondences_pub_ =
    nh->advertise<ff_msgs::DepthImageCorrespondences>(TOPIC_LOCALIZATION_DEPTH_IMAGE_CORRESPONDENCES, 10);
  if (params_.publish_point_clouds) {
    source_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>("source_cloud", 10);
    target_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>("target_cloud", 10);
    point_cloud_result_pub_ = nh->advertise<sensor_msgs::PointCloud2>("point_cloud_result", 10);
  }
}

void DepthOdometryNodelet::DepthCloudCallback(const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg) {
  const auto pose_msgs = depth_odometry_wrapper_.DepthCloudCallback(depth_cloud_msg);
  for (const auto& pose_msg : pose_msgs) {
    odom_pub_.publish(pose_msg);
  }
  if (depth_odometry_wrapper_.depth_point_cloud_registration_enabled()) {
    const auto correspondences_msg = depth_odometry_wrapper_.GetPointCloudCorrespondencesMsg();
    if (!correspondences_msg) return;
    depth_image_correspondences_pub_.publish(*correspondences_msg);
    if (params_.publish_point_clouds) {
      PublishPointClouds();
    }
  }
}

void DepthOdometryNodelet::DepthImageCallback(const sensor_msgs::ImageConstPtr& depth_image_msg) {
  const auto pose_msgs = depth_odometry_wrapper_.DepthImageCallback(depth_image_msg);
  for (const auto& pose_msg : pose_msgs) {
    odom_pub_.publish(pose_msg);
  }
  if (depth_odometry_wrapper_.depth_image_registration_enabled()) {
    const auto correspondences_msg = depth_odometry_wrapper_.GetImageCorrespondencesMsg();
    if (!correspondences_msg) return;
    depth_image_correspondences_pub_.publish(*correspondences_msg);
  }
}

void DepthOdometryNodelet::PublishPointClouds() const {
  source_cloud_pub_.publish(depth_odometry_wrapper_.GetPreviousPointCloudMsg());
  target_cloud_pub_.publish(depth_odometry_wrapper_.GetLatestPointCloudMsg());
  point_cloud_result_pub_.publish(depth_odometry_wrapper_.GetTransformedPreviousPointCloudMsg());
}
}  // namespace depth_odometry

PLUGINLIB_EXPORT_CLASS(depth_odometry::DepthOdometryNodelet, nodelet::Nodelet);
