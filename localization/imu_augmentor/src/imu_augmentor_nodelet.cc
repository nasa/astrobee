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

#include <ff_util/ff_names.h>
#include <imu_augmentor/imu_augmentor_nodelet.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace imu_augmentor {
namespace lc = localization_common;

ImuAugmentorNodelet::ImuAugmentorNodelet() : ff_util::FreeFlyerNodelet(NODE_IMU_AUG, true) {
  imu_nh_.setCallbackQueue(&imu_queue_);
  loc_nh_.setCallbackQueue(&loc_queue_);
  heartbeat_.node = GetName();
  heartbeat_.nodelet_manager = ros::this_node::getName();
  last_time_ = ros::Time::now();
  last_heartbeat_time_ = ros::Time::now();
}

void ImuAugmentorNodelet::Initialize(ros::NodeHandle* nh) {
  // Setup the platform name
  platform_name_ = GetPlatform();
  platform_name_ = (platform_name_.empty() ? "" : platform_name_ + "/");

  SubscribeAndAdvertise(nh);
  Run();
}

void ImuAugmentorNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  state_pub_ = nh->advertise<ff_msgs::EkfState>(TOPIC_GNC_EKF, 1);
  pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_LOCALIZATION_POSE, 1);
  twist_pub_ = nh->advertise<geometry_msgs::TwistStamped>(TOPIC_LOCALIZATION_TWIST, 1);
  heartbeat_pub_ = nh->advertise<ff_msgs::Heartbeat>(TOPIC_HEARTBEAT, 5, true);

  imu_sub_ = imu_nh_.subscribe(TOPIC_HARDWARE_IMU, 100, &ImuAugmentorNodelet::ImuCallback, this,
                               ros::TransportHints().tcpNoDelay());
  // Use the imu nh so that speed mode changes arrive in order wrt IMU msgs
  flight_mode_sub_ = imu_nh_.subscribe(TOPIC_MOBILITY_FLIGHT_MODE, 10, &ImuAugmentorNodelet::FlightModeCallback, this);
  state_sub_ = loc_nh_.subscribe(TOPIC_GRAPH_LOC_STATE, 1, &ImuAugmentorNodelet::LocalizationStateCallback, this,
                                 ros::TransportHints().tcpNoDelay());
}

void ImuAugmentorNodelet::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  imu_augmentor_wrapper_.ImuCallback(*imu_msg);
}

void ImuAugmentorNodelet::FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode) {
  imu_augmentor_wrapper_.FlightModeCallback(*mode);
}

void ImuAugmentorNodelet::LocalizationStateCallback(const ff_msgs::GraphState::ConstPtr& loc_msg) {
  imu_augmentor_wrapper_.LocalizationStateCallback(*loc_msg);
}

boost::optional<ff_msgs::EkfState> ImuAugmentorNodelet::PublishLatestImuAugmentedLocalizationState() {
  const auto latest_imu_augmented_loc_msg = imu_augmentor_wrapper_.LatestImuAugmentedLocalizationMsg();
  if (!latest_imu_augmented_loc_msg) {
    LogDebugEveryN(100, "PublishLatestImuAugmentedLocalizationState: Failed to get latest imu augmented loc msg.");
    return boost::none;
  }
  state_pub_.publish(*latest_imu_augmented_loc_msg);
  return latest_imu_augmented_loc_msg;
}

void ImuAugmentorNodelet::PublishPoseAndTwistAndTransform(const ff_msgs::EkfState& loc_msg) {
  // Publish pose
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = loc_msg.header;
  pose_msg.pose = loc_msg.pose;
  pose_pub_.publish(pose_msg);

  // Publish twist
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header = loc_msg.header;
  twist_msg.twist.linear = loc_msg.velocity;
  twist_msg.twist.angular = loc_msg.omega;
  twist_pub_.publish(twist_msg);

  // Publish TF
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header = loc_msg.header;
  transform_msg.header.stamp = ros::Time::now();
  // If the rate is higher than the sim time, prevent repeated timestamps
  if (transform_msg.header.stamp == last_time_) return;
  last_time_ = transform_msg.header.stamp;

  transform_msg.child_frame_id = platform_name_ + "body";
  transform_msg.transform.translation.x = loc_msg.pose.position.x;
  transform_msg.transform.translation.y = loc_msg.pose.position.y;
  transform_msg.transform.translation.z = loc_msg.pose.position.z;
  transform_msg.transform.rotation = loc_msg.pose.orientation;
  transform_pub_.sendTransform(transform_msg);
}

void ImuAugmentorNodelet::PublishHeartbeat() {
  heartbeat_.header.stamp = ros::Time::now();
  if ((heartbeat_.header.stamp - last_heartbeat_time_).toSec() < 1.0) return;
  heartbeat_pub_.publish(heartbeat_);
  last_heartbeat_time_ = heartbeat_.header.stamp;
}

void ImuAugmentorNodelet::Run() {
  ros::Rate rate(100);
  while (ros::ok()) {
    imu_queue_.callAvailable();
    loc_queue_.callAvailable();
    const auto loc_msg = PublishLatestImuAugmentedLocalizationState();
    if (loc_msg) {
      PublishPoseAndTwistAndTransform(*loc_msg);
    }
    PublishHeartbeat();
    rate.sleep();
  }
}
}  // namespace imu_augmentor

PLUGINLIB_EXPORT_CLASS(imu_augmentor::ImuAugmentorNodelet, nodelet::Nodelet);
