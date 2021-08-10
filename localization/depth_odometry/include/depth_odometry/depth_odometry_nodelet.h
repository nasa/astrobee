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
#ifndef DEPTH_ODOMETRY_DEPTH_ODOMETRY_NODELET_H_
#define DEPTH_ODOMETRY_DEPTH_ODOMETRY_NODELET_H_

#include <depth_odometry/depth_odometry_wrapper.h>
#include <depth_odometry/depth_odometry_nodelet_params.h>
#include <ff_util/ff_nodelet.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace depth_odometry {
class DepthOdometryNodelet : public ff_util::FreeFlyerNodelet {
 public:
  DepthOdometryNodelet();

 private:
  void Initialize(ros::NodeHandle* nh) final;
  void SubscribeAndAdvertise(ros::NodeHandle* nh);
  void DepthCloudCallback(const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg);
  void DepthImageCallback(const sensor_msgs::ImageConstPtr& depth_image_msg);
  void PublishPointClouds() const;

  DepthOdometryNodeletParams params_;
  DepthOdometryWrapper depth_odometry_wrapper_;
  ros::Subscriber point_cloud_sub_;
  image_transport::Subscriber depth_image_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher source_cloud_pub_, target_cloud_pub_, point_cloud_result_pub_, correspondences_pub_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_ODOMETRY_NODELET_H_
