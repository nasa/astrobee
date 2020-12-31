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
#include <imu_bias_tester/imu_bias_tester_nodelet.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <geometry_msgs/PoseStamped.h>

namespace imu_bias_tester {
namespace lc = localization_common;

ImuBiasTesterNodelet::ImuBiasTesterNodelet() : ff_util::FreeFlyerNodelet(NODE_IMU_AUG, true) {
  imu_nh_.setCallbackQueue(&imu_queue_);
  loc_nh_.setCallbackQueue(&loc_queue_);
}

void ImuBiasTesterNodelet::Initialize(ros::NodeHandle* nh) {
  SubscribeAndAdvertise(nh);
  Run();
}

void ImuBiasTesterNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_IMU_BIAS_TESTER_POSE, 1);

  imu_sub_ = imu_nh_.subscribe(TOPIC_HARDWARE_IMU, 100, &ImuBiasTesterNodelet::ImuCallback, this,
                               ros::TransportHints().tcpNoDelay());
  state_sub_ = loc_nh_.subscribe(TOPIC_GRAPH_LOC_STATE, 10, &ImuBiasTesterNodelet::LocalizationStateCallback, this,
                                 ros::TransportHints().tcpNoDelay());
}

void ImuBiasTesterNodelet::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  imu_bias_tester_wrapper_.ImuCallback(*imu_msg);
}

void ImuBiasTesterNodelet::LocalizationStateCallback(const ff_msgs::EkfState::ConstPtr& loc_msg) {
  const auto imu_integration_states = imu_bias_tester_wrapper_.LocalizationStateCallback(*loc_msg);
  for (const auto& state : imu_integration_states) {
    geometry_msgs::PoseStamped pose_msg;
    lc::PoseToMsg(state.pose(), pose_msg.pose);
    lc::TimeToHeader(state.timestamp(), pose_msg.header);
    pose_pub_.publish(pose_msg);
  }
}

void ImuBiasTesterNodelet::Run() {
  ros::Rate rate(100);
  while (ros::ok()) {
    imu_queue_.callAvailable();
    loc_queue_.callAvailable();
    rate.sleep();
  }
}
}  // namespace imu_bias_tester

PLUGINLIB_EXPORT_CLASS(imu_bias_tester::ImuBiasTesterNodelet, nodelet::Nodelet);
