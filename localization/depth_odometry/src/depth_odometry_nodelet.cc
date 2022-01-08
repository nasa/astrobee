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
#include <ff_util/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace mc = msg_conversions;

DepthOdometryNodelet::DepthOdometryNodelet() : ff_util::FreeFlyerNodelet(NODE_DEPTH_ODOM, true), enabled_(false) {}

void DepthOdometryNodelet::Initialize(ros::NodeHandle* nh) { SubscribeAndAdvertise(nh); }

void DepthOdometryNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  const std::string point_cloud_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                        static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                        static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX);
  point_cloud_sub_ = nh->subscribe<sensor_msgs::PointCloud2>(
    point_cloud_topic, 1, &DepthOdometryNodelet::PointCloudCallback, this, ros::TransportHints().tcpNoDelay());

  image_transport::ImageTransport image_transport(*nh);
  const std::string image_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                  static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                  static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_EXTENDED) +
                                  static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_AMPLITUDE_IMAGE);
  image_sub_ = image_transport.subscribe(image_topic, 1, &DepthOdometryNodelet::ImageCallback, this);
  depth_odometry_pub_ = nh->advertise<ff_msgs::DepthOdometry>(TOPIC_LOCALIZATION_DEPTH_ODOM, 10);
  enable_srv_ = nh->advertiseService(SERVICE_LOCALIZATION_DO_ENABLE, &DepthOdometryNodelet::EnableService, this);
}

void DepthOdometryNodelet::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
  if (!enabled_) return;
  const auto depth_odometry_msgs = depth_odometry_wrapper_.PointCloudCallback(point_cloud_msg);
  for (const auto& depth_odometry_msg : depth_odometry_msgs) {
    depth_odometry_pub_.publish(depth_odometry_msg);
  }
}

void DepthOdometryNodelet::ImageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
  if (!enabled_) return;
  const auto depth_odometry_msgs = depth_odometry_wrapper_.ImageCallback(image_msg);
  for (const auto& depth_odometry_msg : depth_odometry_msgs) {
    depth_odometry_pub_.publish(depth_odometry_msg);
  }
}

bool DepthOdometryNodelet::EnableService(ff_msgs::SetBool::Request& req, ff_msgs::SetBool::Response& res) {
  enabled_ = req.enable;
  res.success = true;
  return true;
}
}  // namespace depth_odometry

PLUGINLIB_EXPORT_CLASS(depth_odometry::DepthOdometryNodelet, nodelet::Nodelet);
