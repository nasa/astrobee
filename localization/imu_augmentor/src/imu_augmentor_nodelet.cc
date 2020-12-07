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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <glog/logging.h>

namespace imu_augmentor {

ImuAugmentorNodelet::ImuAugmentorNodelet()
    : ff_util::FreeFlyerNodelet(NODE_IMU_AUG, true), platform_name_(GetPlatform()) {}

void ImuAugmentorNodelet::Initialize(ros::NodeHandle* nh) { SubscribeAndAdvertise(nh); }

void ImuAugmentorNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  state_pub_ = nh->advertise<ff_msgs::EkfState>(TOPIC_GNC_EKF, 1);
  pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_LOCALIZATION_POSE, 1);
  twist_pub_ = nh->advertise<geometry_msgs::TwistStamped>(TOPIC_LOCALIZATION_TWIST, 1);

  imu_sub_ =
    nh->subscribe(TOPIC_HARDWARE_IMU, 1, &ImuAugmentorNodelet::ImuCallback, this, ros::TransportHints().tcpNoDelay());
  state_sub_ = nh->subscribe(TOPIC_GRAPH_LOC_STATE, 1, &ImuAugmentorNodelet::LocalizationStateCallback, this,
                             ros::TransportHints().tcpNoDelay());
}

void ImuAugmentorNodelet::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  imu_augmentor_wrapper_.ImuCallback(*imu_msg);
  const auto loc_msg = PublishLatestImuAugmentedLocalizationState();
  if (loc_msg) {
    PublishPoseAndTwistAndTransform(*loc_msg);
  }
}

void ImuAugmentorNodelet::LocalizationStateCallback(const ff_msgs::EkfState::ConstPtr& loc_msg) {
  imu_augmentor_wrapper_.LocalizationStateCallback(*loc_msg);
}

boost::optional<ff_msgs::EkfState> ImuAugmentorNodelet::PublishLatestImuAugmentedLocalizationState() {
  const auto latest_imu_augmented_loc_msg = imu_augmentor_wrapper_.LatestImuAugmentedLocalizationMsg();
  if (!latest_imu_augmented_loc_msg) {
    LOG_EVERY_N(WARNING, 100)
      << "PublishLatestImuAugmentedLocalizationState: Failed to get latest imu augmented loc msg.";
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
  transform_msg.child_frame_id = platform_name_ + "body";
  transform_msg.transform.translation.x = loc_msg.pose.position.x;
  transform_msg.transform.translation.y = loc_msg.pose.position.y;
  transform_msg.transform.translation.z = loc_msg.pose.position.z;
  transform_msg.transform.rotation = loc_msg.pose.orientation;
  transform_pub_.sendTransform(transform_msg);
}
}  // namespace imu_augmentor

PLUGINLIB_EXPORT_CLASS(imu_augmentor::ImuAugmentorNodelet, nodelet::Nodelet);
