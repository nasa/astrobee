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


#include "dds_ros_bridge/ros_gnc_fam_cmd_state.h"

namespace ff {

RosGncFamCmdStateToRapid::RosGncFamCmdStateToRapid(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle& nh,
                                            const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  state_supplier_.reset(new RosGncFamCmdStateToRapid::StateSupplier(
      rapid::ext::astrobee::GNC_FAM_CMD_STATE_TOPIC + pub_topic,
      "",
      "AstrobeeGncFamCmdStateProfile",
      ""));

  // start subscriber
  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosGncFamCmdStateToRapid::MsgCallback,
                       this);

  // Initialize the state message
  rapid::RapidHelper::initHeader(state_supplier_->event().hdr);

  // Setup time for publishing the gnc fam cmd state but don't start the timer
  // since the rate is 0. The bridge will set this rate at the end of its init
  // update: Andrew changed rate to 1.0 to avoid a runtime bounds error. Should
  // not affect since autostart argument is set to false.
  gnc_timer_ = nh_.createTimer(ros::Rate(1.0),
                               &RosGncFamCmdStateToRapid::PubGncFamCmdState,
                               this,
                               false,
                               false);
}

void RosGncFamCmdStateToRapid::CopyVec3D(rapid::Vec3d& vec_out,
                                         const geometry_msgs::Vector3& vec_in) {
  vec_out[0] = vec_in.x;
  vec_out[1] = vec_in.y;
  vec_out[2] = vec_in.z;
}

void RosGncFamCmdStateToRapid::MsgCallback(
                                      const ff_msgs::FamCommandConstPtr& msg) {
  fam_msg_ = msg;
}

void RosGncFamCmdStateToRapid::PubGncFamCmdState(const ros::TimerEvent& event) {
  if (fam_msg_ == NULL) {
    return;
  }

  rapid::ext::astrobee::GncFamCmdState &msg = state_supplier_->event();

  // Copy time
  msg.hdr.timeStamp = util::RosTime2RapidTime(fam_msg_->header.stamp);

  CopyVec3D(msg.wrench.force, fam_msg_->wrench.force);

  CopyVec3D(msg.wrench.torque, fam_msg_->wrench.torque);

  CopyVec3D(msg.accel, fam_msg_->accel);

  CopyVec3D(msg.alpha, fam_msg_->alpha);

  msg.status = fam_msg_->status;

  CopyVec3D(msg.position_error, fam_msg_->position_error);

  CopyVec3D(msg.position_error_integrated, fam_msg_->position_error_integrated);

  CopyVec3D(msg.attitude_error, fam_msg_->attitude_error);

  CopyVec3D(msg.attitude_error_integrated, fam_msg_->attitude_error_integrated);

  msg.attitude_error_mag = fam_msg_->attitude_error_mag;

  msg.control_mode = fam_msg_->control_mode;

  // Send message
  state_supplier_->sendEvent();
}

void RosGncFamCmdStateToRapid::SetGncPublishRate(float rate) {
  if (rate == 0) {
    gnc_timer_.stop();
  } else {
    gnc_timer_.setPeriod(ros::Duration(ros::Rate(rate)));
    gnc_timer_.start();
  }
}

}  // end namespace ff
