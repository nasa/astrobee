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
#include <ff_util/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace depth_odometry {
namespace lc = localization_common;
// namespace mc = msg_conversions;

DepthOdometryNodelet::DepthOdometryNodelet() : ff_util::FreeFlyerNodelet(NODE_DEPTH_ODOM, true) {
  /*config_reader::ConfigReader config;
  lc::LoadDepthOdometryConfig(config);
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  LoadDepthOdometryNodeletParams(config, params_);*/
}

void DepthOdometryNodelet::Initialize(ros::NodeHandle* nh) { SubscribeAndAdvertise(nh); }

void DepthOdometryNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  const std::string depth_cloud_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                        static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                        static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX);
  depth_sub_ = nh->subscribe<sensor_msgs::PointCloud2>(depth_cloud_topic, 10, &DepthOdometryNodelet::DepthCloudCallback,
                                                       this, ros::TransportHints().tcpNoDelay());
  odom_pub_ = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(TOPIC_LOCALIZATION_DEPTH_ODOM, 10);
}

void DepthOdometryNodelet::DepthCloudCallback(const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg) {
  std::pair<lc::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> depth_cloud;
  pcl::fromROSMsg(*depth_cloud_msg, *(depth_cloud.second));
  depth_cloud.first = lc::TimeFromHeader(depth_cloud_msg->header);
  const auto relative_pose = depth_odometry_.DepthCloudCallback(depth_cloud);
  if (!relative_pose) {
    LogError("DepthCloudCallback: Failed to get relative pose.");
    return;
  }

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  lc::PoseToMsg(lc::GtPose(*relative_pose), pose_msg.pose.pose);
  // TODO(rsoussan): fill in covariance
  lc::TimeToHeader(depth_cloud.first, pose_msg.header);
  odom_pub_.publish(pose_msg);
}
}  // namespace depth_odometry

PLUGINLIB_EXPORT_CLASS(depth_odometry::DepthOdometryNodelet, nodelet::Nodelet);
