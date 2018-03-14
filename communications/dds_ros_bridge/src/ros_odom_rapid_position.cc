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

#include "dds_ros_bridge/ros_odom_rapid_position.h"

namespace ff {

RosOdomRapidPosition::RosOdomRapidPosition(const std::string& subscribe_topic,
                                           const std::string& pub_topic,
                                           const ros::NodeHandle& nh,
                                           const unsigned int queue_size) :
    RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size),
    ekf_sent_(false),
    pub_ekf_(false) {
  params_.config.poseEncoding = rapid::RAPID_ROT_QUAT;
  params_.config.velocityEncoding = rapid::RAPID_ROT_XYZ;

  // Add confidence to value keys
  rapid::KeyTypeValue ktv("confidence", "INT", "0");
  params_.config.valueKeys.push_back(ktv);

  // TODO(all): confirm topic suffix has '-'
  params_.topicSuffix += pub_topic;

  // instantiate provider
  provider_.reset(
    new rapid::PositionProviderRosHelper(params_, "RosOdomRapidPosition"));

  state_supplier_.reset(
      new RosOdomRapidPosition::StateSupplier(
          rapid::ext::astrobee::EKF_STATE_TOPIC + pub_topic, "",
          "AstrobeeEkfStateProfile", ""));

  // start subscriber
  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosOdomRapidPosition::MsgCallback, this);

  // Initialize the state message
  rapid::RapidHelper::initHeader(state_supplier_->event().hdr);

  state_supplier_->event().cov_diag.length(15);
  state_supplier_->event().ml_mahal_dists.length(50);

  // Setup timers for publishing the position and ekf but don't start them since
  // the rates are 0. The bridge will set this rate at the end of its init
  // update: Andrew changed rate to 1.0 to avoid a runtime bounds error. Should
  // not affect since autostart argument is set to false.
  ekf_timer_ = nh_.createTimer(ros::Rate(1.0),
                                 &RosOdomRapidPosition::PubEkf,
                                 this,
                                 false,
                                 false);

  // update: Andrew changed rate to 1.0 to avoid a runtime bounds error. Should
  // not affect since autostart argument is set to false.
  position_timer_ = nh_.createTimer(ros::Rate(1.0),
                                      &RosOdomRapidPosition::PubPosition,
                                      this,
                                      false,
                                      false);
}

void RosOdomRapidPosition::MsgCallback(const ff_msgs::EkfStateConstPtr& msg) {
  ekf_msg_ = msg;

  // Don't find the max of_count and ml_count or copy over the ml_mahal_dists if
  // we aren't sending it to the ground
  if (pub_ekf_) {
    if (ekf_sent_) {
      // If we just sent the ekf messages to the ground, restart the search for
      // the max values
      state_supplier_->event().of_count = msg->of_count;
      state_supplier_->event().ml_count = msg->ml_count;
      ekf_sent_ = false;
    } else {
      // Brian wants the max values sent to the ground, not the most recent
      if (state_supplier_->event().of_count < msg->of_count) {
        state_supplier_->event().of_count = msg->of_count;
      }
      if (state_supplier_->event().ml_count < msg->ml_count) {
        state_supplier_->event().ml_count = msg->ml_count;
      }
    }

    // Brian wants the most recent mahalanobis distance where element 0 is a
    // number
    if (!std::isnan(msg->ml_mahal_dists[0])) {
      for (int i = 0; i < 50; i++) {
        state_supplier_->event().ml_mahal_dists[i] = msg->ml_mahal_dists[i];
      }
    }
  }
}

void RosOdomRapidPosition::CopyTransform3D(rapid::Transform3D &transform,
                                           const geometry_msgs::Pose& pose) {
  transform.xyz[0] = pose.position.x;
  transform.xyz[1] = pose.position.y;
  transform.xyz[2] = pose.position.z;

  transform.rot[0] = pose.orientation.x;
  transform.rot[1] = pose.orientation.y;
  transform.rot[2] = pose.orientation.z;
  transform.rot[3] = pose.orientation.w;
}

void RosOdomRapidPosition::CopyVec3D(rapid::Vec3d& vec_out,
                                     const geometry_msgs::Vector3& vec_in) {
  vec_out[0] = vec_in.x;
  vec_out[1] = vec_in.y;
  vec_out[2] = vec_in.z;
}

void RosOdomRapidPosition::PubEkf(const ros::TimerEvent& event) {
  ekf_sent_ = true;

  // Make sure we have received an ekf message before trying to send it
  if (ekf_msg_ == NULL) {
    return;
  }

  rapid::ext::astrobee::EkfState &msg = state_supplier_->event();

  // Copy time
  msg.hdr.timeStamp = util::RosTime2RapidTime(ekf_msg_->header.stamp);

  CopyTransform3D(msg.pose, ekf_msg_->pose);

  CopyVec3D(msg.velocity, ekf_msg_->velocity);

  CopyVec3D(msg.omega, ekf_msg_->omega);

  CopyVec3D(msg.gyro_bias, ekf_msg_->gyro_bias);

  CopyVec3D(msg.accel, ekf_msg_->accel);

  CopyVec3D(msg.accel_bias, ekf_msg_->accel_bias);

  for (int i = 0; i < 15; i++) {
    msg.cov_diag[i] = ekf_msg_->cov_diag[i];
  }

  msg.confidence = ekf_msg_->confidence;

  msg.status = ekf_msg_->status;

  // Don't copy over of_count and ml_count since the max has already been found

  CopyTransform3D(msg.hr_global_pose, ekf_msg_->hr_global_pose);

  // Don't copy over ml_mahal_dists since it was copied over when we received
  // the message

  // Send message
  state_supplier_->sendEvent();
}

void RosOdomRapidPosition::PubPosition(const ros::TimerEvent& event) {
  // Make sure we have received an ekf message before trying to send it
  if (ekf_msg_ != NULL) {
    provider_->Publish(ekf_msg_);
  }
}

void RosOdomRapidPosition::SetEkfPublishRate(float rate) {
  if (rate == 0) {
    ekf_timer_.stop();
    pub_ekf_ = false;
  } else {
    pub_ekf_ = true;
    ekf_timer_.setPeriod(ros::Duration(ros::Rate(rate)));
    ekf_timer_.start();  // Start in case it was never started
  }
}

void RosOdomRapidPosition::SetPositionPublishRate(float rate) {
  if (rate == 0) {
    position_timer_.stop();
  } else {
    position_timer_.setPeriod(ros::Duration(ros::Rate(rate)));
    position_timer_.start();  // Start in case it was never started
  }
}
}  // end namespace ff
