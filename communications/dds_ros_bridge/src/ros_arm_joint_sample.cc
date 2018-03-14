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

#include "dds_ros_bridge/ros_arm_joint_sample.h"

namespace ff {

RosArmJointSampleToRapid::RosArmJointSampleToRapid(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle &nh,
                                            unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  config_supplier_.reset(
      new ff::RosArmJointSampleToRapid::ConfigSupplier(
          rapid::JOINT_CONFIG_TOPIC + publish_topic_, "",
          "RapidJointConfigProfile", "", ""));

  sample_supplier_.reset(
      new ff::RosArmJointSampleToRapid::SampleSupplier(
          rapid::JOINT_SAMPLE_TOPIC + publish_topic_, "",
          "RapidJointSampleProfile", "", ""));

  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosArmJointSampleToRapid::Callback,
                       this);

  rapid::RapidHelper::initHeader(config_supplier_->event().hdr);
  rapid::RapidHelper::initHeader(sample_supplier_->event().hdr);

  // Initialize the serial number to be 1, it will stay 1 during execution since
  // the config won't change during run time.
  // Technically this could have been set in the initHeader function but it is
  // the last argument and I didn't care to specify any of the other arguments
  config_supplier_->event().hdr.serial = 1;
  sample_supplier_->event().hdr.serial = 1;

  // Hard code number of joints by initializing the size of the sequences in the
  // sample because it won't change
  sample_supplier_->event().anglePos.length(4);
  sample_supplier_->event().angleVel.length(4);
  sample_supplier_->event().angleAcc.length(4);
  sample_supplier_->event().current.length(4);
  sample_supplier_->event().torque.length(4);
  sample_supplier_->event().temperature.length(4);
  sample_supplier_->event().status.length(4);

  // Hard code config because it won't change
  rapid::JointConfig &msg = config_supplier_->event();
  std::strncpy(msg.jointGroupName, "AstrobeeArmJointGroup", 32);

  msg.jointDefinitions.length(4);
  std::strncpy(msg.jointDefinitions[0].frameName, "arm tilt joint", 128);
  std::strncpy(msg.jointDefinitions[0].dof, "y", 64);
  std::strncpy(msg.jointDefinitions[1].frameName, "arm pan joint", 128);
  std::strncpy(msg.jointDefinitions[1].dof, "x", 64);
  std::strncpy(msg.jointDefinitions[2].frameName, "arm gripper joint", 128);
  std::strncpy(msg.jointDefinitions[2].dof, "z", 64);
  std::strncpy(msg.jointDefinitions[3].frameName, "arm gripper joint", 128);
  std::strncpy(msg.jointDefinitions[3].dof, "z", 64);

  config_supplier_->sendEvent();
}

void RosArmJointSampleToRapid::Callback(
                            ff_msgs::JointSampleStampedConstPtr const& sample) {
  rapid::JointSample &msg = sample_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(sample->header.stamp);

  int joint_index;

  for (unsigned int i = 0; i < sample->samples.size(); i++) {
    if (sample->samples[i].name == "tilt") {
      joint_index = 0;
      msg.anglePos[joint_index] = sample->samples[i].angle_pos * M_PI/180;
    } else if (sample->samples[i].name == "pan") {
      joint_index = 1;
      msg.anglePos[joint_index] = sample->samples[i].angle_pos * M_PI/180;
    } else if (sample->samples[i].name == "gripper") {
      joint_index = 2;
      // Check to make sure that the gripper angle is valid
      if (sample->samples[i].angle_pos < 20) {
        msg.anglePos[joint_index] = 20 * M_PI/180;
      } else if (sample->samples[i].angle_pos > 45) {
        msg.anglePos[joint_index] = 45 * M_PI/180;
      } else {
        msg.anglePos[joint_index] = sample->samples[i].angle_pos * M_PI/180;
      }
    } else {
      ROS_ERROR("DDS Bridge: Arm joint named %s isn't recognized.",
                                              sample->samples[i].name.c_str());
      continue;
    }

    msg.angleVel[joint_index] = sample->samples[i].angle_vel;
    msg.angleAcc[joint_index] = sample->samples[i].angle_acc;
    msg.current[joint_index] = sample->samples[i].current;
    msg.torque[joint_index] = sample->samples[i].torque;
    msg.temperature[joint_index] = sample->samples[i].temperature;
    msg.status[joint_index] = sample->samples[i].status;
  }

  // GDS needs 2 joints for the gripper even though the gripper only has 1.
  // Copy the gripper joint we received into the second gripper joint
  msg.anglePos[3] = msg.anglePos[2];
  msg.angleVel[3] = msg.angleVel[2];
  msg.angleAcc[3] = msg.angleAcc[2];
  msg.current[3] = msg.current[2];
  msg.torque[3] = msg.torque[2];
  msg.temperature[3] = msg.temperature[2];
  msg.status[3] = msg.status[2];

  // Invert the first gripper so that the grippers are opposite each other
  msg.anglePos[2] = -1 * msg.anglePos[2];

  sample_supplier_->sendEvent();
}

}  // end namespace ff
