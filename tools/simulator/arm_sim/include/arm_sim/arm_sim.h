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


#ifndef ARM_SIM_ARM_SIM_H_
#define ARM_SIM_ARM_SIM_H_

#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <errno.h>
#include <stdint.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/statvfs.h>

#include <actionlib/server/simple_action_server.h>

#include <ff_msgs/ArmAction.h>
#include <ff_msgs/ArmGripperState.h>
#include <ff_msgs/ArmJointState.h>
#include <ff_msgs/ArmStateStamped.h>
#include <ff_msgs/JointSample.h>
#include <ff_msgs/JointSampleStamped.h>
#include <ff_util/ff_names.h>

#include <nodelet/nodelet.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace arm_sim {

class ArmSim : public nodelet::Nodelet {
 public:
  ArmSim();
  ~ArmSim();

  void GoalCallback(ff_msgs::ArmGoalConstPtr const& goal);

  bool Pan(float angle);

  bool Tilt(float angle);

  virtual void onInit();

 private:
  std::shared_ptr<actionlib::SimpleActionServer<ff_msgs::ArmAction>> sas_arm_;

  ff_msgs::ArmStateStamped arm_state_;
  ff_msgs::JointSampleStamped joint_sample_;

  ros::NodeHandle nh_;

  ros::Publisher arm_state_pub_, joint_sample_pub_;

  int pub_queue_size_;
};

}  // namespace arm_sim

#endif  // ARM_SIM_ARM_SIM_H_
