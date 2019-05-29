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


#include "dds_ros_bridge/ros_gnc_control_state.h"

namespace ff {

RosGncControlStateToRapid::RosGncControlStateToRapid(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle& nh,
                                            const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  state_supplier_.reset(new RosGncControlStateToRapid::StateSupplier(
      rapid::ext::astrobee::GNC_CONTROL_STATE_TOPIC + pub_topic,
      "",
      "AstrobeeGncControlStateProfile",
      ""));

  // start subscriber
  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosGncControlStateToRapid::MsgCallback,
                       this);

  // Initialize the state message
  rapid::RapidHelper::initHeader(state_supplier_->event().hdr);

  // Setup time for publishing the gnc control state but don't start the timer
  // since the rate is 0. The bridge will set this rate at the end of its init
  // update: Andrew changed rate to 1.0 to avoid a runtime bounds error. Should
  // not affect since autostart argument is set to false.
  gnc_timer_ = nh_.createTimer(ros::Rate(1.0),
                               &RosGncControlStateToRapid::PubGncControlState,
                               this,
                               false,
                               false);
}

void RosGncControlStateToRapid::CopyVec3D(rapid::Vec3d& vec_out,
                                        const geometry_msgs::Vector3& vec_in) {
  vec_out[0] = vec_in.x;
  vec_out[1] = vec_in.y;
  vec_out[2] = vec_in.z;
}

void RosGncControlStateToRapid::MsgCallback(
                                    const ff_msgs::ControlStateConstPtr& msg) {
  gnc_msg_ = msg;
}

void RosGncControlStateToRapid::PubGncControlState(
                                                const ros::TimerEvent& event) {
  if (gnc_msg_ == NULL) {
    return;
  }

  rapid::ext::astrobee::GncControlState &msg = state_supplier_->event();

  // Copy time
  msg.hdr.timeStamp = util::RosTime2RapidTime(gnc_msg_->when);

  msg.pose.xyz[0] = gnc_msg_->pose.position.x;
  msg.pose.xyz[1] = gnc_msg_->pose.position.y;
  msg.pose.xyz[2] = gnc_msg_->pose.position.z;

  msg.pose.rot[0] = gnc_msg_->pose.orientation.x;
  msg.pose.rot[1] = gnc_msg_->pose.orientation.y;
  msg.pose.rot[2] = gnc_msg_->pose.orientation.z;
  msg.pose.rot[3] = gnc_msg_->pose.orientation.w;

  CopyVec3D(msg.twist.linear, gnc_msg_->twist.linear);

  CopyVec3D(msg.twist.angular, gnc_msg_->twist.angular);

  CopyVec3D(msg.accel.linear, gnc_msg_->accel.linear);

  CopyVec3D(msg.accel.angular, gnc_msg_->accel.angular);

  // Send message
  state_supplier_->sendEvent();
}

void RosGncControlStateToRapid::SetGncPublishRate(float rate) {
  if (rate == 0) {
    gnc_timer_.stop();
  } else {
    gnc_timer_.setPeriod(ros::Duration(ros::Rate(rate)));
    gnc_timer_.start();
  }
}

}  // end namespace ff
