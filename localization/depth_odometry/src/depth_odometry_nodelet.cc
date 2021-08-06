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
#include <depth_odometry/utilities.h>
#include <ff_msgs/DepthCorrespondences.h>
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
  depth_sub_ = nh->subscribe<sensor_msgs::PointCloud2>(depth_cloud_topic, 10, &DepthOdometryNodelet::DepthCloudCallback,
                                                       this, ros::TransportHints().tcpNoDelay());
  odom_pub_ = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(TOPIC_LOCALIZATION_DEPTH_ODOM, 10);
  if (params_.publish_point_clouds) {
    point_cloud_a_pub_ = nh->advertise<sensor_msgs::PointCloud2>("point_cloud_a", 10);
    point_cloud_b_pub_ = nh->advertise<sensor_msgs::PointCloud2>("point_cloud_b", 10);
    point_cloud_result_pub_ = nh->advertise<sensor_msgs::PointCloud2>("point_cloud_result", 10);
    correspondences_pub_ = nh->advertise<ff_msgs::DepthCorrespondences>("depth_correspondences", 10);
  }
}

void DepthOdometryNodelet::DepthCloudCallback(const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg) {
  const lc::Time timestamp = lc::TimeFromHeader(depth_cloud_msg->header);
  std::pair<lc::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> depth_cloud{
    timestamp, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>())};
  pcl::fromROSMsg(*depth_cloud_msg, *(depth_cloud.second));
  const auto relative_pose = depth_odometry_.DepthCloudCallback(depth_cloud);
  if (!relative_pose) {
    LogError("DepthCloudCallback: Failed to get relative pose.");
    return;
  }

  // LogError("rel pose: " << std::endl << relative_pose->first.matrix());

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  mc::EigenPoseCovarianceToMsg(relative_pose->first, relative_pose->second, pose_msg);
  lc::TimeToHeader(depth_cloud.first, pose_msg.header);
  odom_pub_.publish(pose_msg);
  PublishDepthCorrespondences(depth_odometry_.correspondences(), depth_odometry_.latest_depth_cloud().first,
                              depth_odometry_.previous_depth_cloud().first);

  if (params_.publish_point_clouds) {
    PublishPointClouds(*(depth_odometry_.previous_depth_cloud().second), *(depth_odometry_.latest_depth_cloud().second),
                       relative_pose->first.matrix().cast<float>());
  }
}

void DepthOdometryNodelet::PublishDepthCorrespondences(const pcl::Correspondences& correspondences,
                                                       const lc::Time time_a, const lc::Time time_b) {
  ff_msgs::DepthCorrespondences correspondences_msg;
  lc::TimeToHeader(time_a, correspondences_msg.header);
  correspondences_msg.header.frame_id = "haz_cam";
  const ros::Time ros_time_a(time_a);
  const ros::Time ros_time_b(time_b);
  correspondences_msg.time_a.sec = ros_time_a.sec;
  correspondences_msg.time_a.nsec = ros_time_a.nsec;
  correspondences_msg.time_b.sec = ros_time_b.sec;
  correspondences_msg.time_b.nsec = ros_time_b.nsec;

  for (const auto& correspondence : correspondences) {
    ff_msgs::DepthCorrespondence correspondence_msg;
    correspondence_msg.index_a = correspondence.index_query;
    correspondence_msg.index_b = correspondence.index_match;
    correspondences_msg.correspondences.push_back(correspondence_msg);
  }
  correspondences_pub_.publish(correspondences_msg);
}

void DepthOdometryNodelet::PublishPointClouds(const pcl::PointCloud<pcl::PointXYZ>& cloud_a,
                                              const pcl::PointCloud<pcl::PointXYZ>& cloud_b,
                                              const Eigen::Matrix<float, 4, 4>& relative_transform) const {
  sensor_msgs::PointCloud2 ros_cloud_a;
  pcl::toROSMsg(cloud_a, ros_cloud_a);
  ros_cloud_a.header.stamp = ros::Time::now();
  ros_cloud_a.header.frame_id = "haz_cam";
  point_cloud_a_pub_.publish(ros_cloud_a);

  sensor_msgs::PointCloud2 ros_cloud_b;
  pcl::toROSMsg(cloud_b, ros_cloud_b);
  ros_cloud_b.header.stamp = ros::Time::now();
  ros_cloud_b.header.frame_id = "haz_cam";
  point_cloud_b_pub_.publish(ros_cloud_b);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(cloud_a, *cloud_result, relative_transform);
  sensor_msgs::PointCloud2 ros_cloud_result;
  pcl::toROSMsg(*cloud_result, ros_cloud_result);
  ros_cloud_result.header.stamp = ros::Time::now();
  ros_cloud_result.header.frame_id = "haz_cam";
  point_cloud_result_pub_.publish(ros_cloud_result);
}
}  // namespace depth_odometry

PLUGINLIB_EXPORT_CLASS(depth_odometry::DepthOdometryNodelet, nodelet::Nodelet);
