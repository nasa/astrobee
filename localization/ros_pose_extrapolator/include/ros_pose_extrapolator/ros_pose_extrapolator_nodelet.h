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
#ifndef ROS_POSE_EXTRAPOLATOR_ROS_POSE_EXTRAPOLATOR_NODELET_H_
#define ROS_POSE_EXTRAPOLATOR_ROS_POSE_EXTRAPOLATOR_NODELET_H_

#include <ff_msgs/EkfState.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/GraphLocState.h>
#include <ff_msgs/GraphVIOState.h>
#include <ff_msgs/Heartbeat.h>
#include <ff_util/ff_nodelet.h>
#include <ros_pose_extrapolator/ros_pose_extrapolator_wrapper.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/optional.hpp>

#include <string>

namespace ros_pose_extrapolator {
class RosPoseExtrapolatorNodelet : public ff_util::FreeFlyerNodelet {
 public:
  RosPoseExtrapolatorNodelet();

 private:
  void Initialize(ros::NodeHandle* nh) final;

  void SubscribeAndAdvertise(ros::NodeHandle* nh);

  void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

  void FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode);

  void LocalizationStateCallback(const ff_msgs::GraphLocState::ConstPtr& graph_loc_state_msg);

  void GraphVIOStateCallback(const ff_msgs::GraphVIOState::ConstPtr& graph_vio_state_msg);

  boost::optional<ff_msgs::EkfState> PublishLatestExtrapolatedLocalizationState();

  void PublishPoseAndTwistAndTransform(const ff_msgs::EkfState& loc_msg);

  void Run();

  void PublishHeartbeat();

  ros_pose_extrapolator::RosPoseExtrapolatorWrapper ros_pose_extrapolator_wrapper_;
  std::string platform_name_;
  ros::NodeHandle imu_nh_, loc_nh_, vio_nh_;
  ros::CallbackQueue imu_queue_, loc_queue_, vio_queue_;
  ros::Subscriber imu_sub_, flight_mode_sub_, loc_sub_, vio_sub_;
  ros::Publisher state_pub_, pose_pub_, twist_pub_, heartbeat_pub_;
  ff_msgs::Heartbeat heartbeat_;
  tf2_ros::TransformBroadcaster transform_pub_;
  ros::Time last_time_;
  ros::Time last_heartbeat_time_;
  boost::optional<ros::Time> last_state_msg_time_;
};
}  // namespace ros_pose_extrapolator

#endif  // ROS_POSE_EXTRAPOLATOR_ROS_POSE_EXTRAPOLATOR_NODELET_H_
