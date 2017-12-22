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


#include <arm_sim/arm_sim.h>

namespace arm_sim {
ArmSim::ArmSim() : pub_queue_size_(10) {
}

ArmSim::~ArmSim() {
}

bool ArmSim::Pan(float angle) {
  float diff = joint_sample_.samples[0].angle_pos - angle;
  float step = 1 * M_PI/180;
  int i, num_iter = std::abs(diff/step);
  ff_msgs::ArmFeedback feedback;

  // Check the direction we need to step in
  if (diff < 0) {
    step *= -1;
  }

  arm_state_.joint_state.state = ff_msgs::ArmJointState::MOVING;
  arm_state_pub_.publish(arm_state_);

  for (i = 0; i < num_iter; i++) {
    if (sas_arm_->isPreemptRequested()) {
      return false;
    }
    joint_sample_.samples[0].angle_pos -= step;
    joint_sample_pub_.publish(joint_sample_);
    ros::Duration(0.1).sleep();
    // Don't have to publish fancy feedback since the executive doesn't use it
    sas_arm_->publishFeedback(feedback);
  }

  // Make pan angle match angle received exactly so that the angle isn't
  // toggled when sending the same tilt angle again and again and again
  joint_sample_.samples[0].angle_pos = angle;
  joint_sample_pub_.publish(joint_sample_);

  if (std::abs(joint_sample_.samples[1].angle_pos - M_PI) < 0.05) {
    arm_state_.joint_state.state = ff_msgs::ArmJointState::STOWED;
  } else {
    arm_state_.joint_state.state = ff_msgs::ArmJointState::STOPPED;
  }
  arm_state_pub_.publish(arm_state_);
  return true;
}

bool ArmSim::Tilt(float angle) {
  float diff = joint_sample_.samples[1].angle_pos - angle;
  float step = 1 * M_PI/180;
  int i, num_iter = std::abs(diff/step);
  ff_msgs::ArmFeedback feedback;

  // Check the direction we need to step in
  if (diff < 0) {
    step *= -1;
  }

  if (std::abs(joint_sample_.samples[1].angle_pos - M_PI) < 0.05) {
    arm_state_.joint_state.state = ff_msgs::ArmJointState::DEPLOYING;
  } else if (std::abs(angle - M_PI) < 0.05) {
    arm_state_.joint_state.state = ff_msgs::ArmJointState::STOWING;
  } else {
    arm_state_.joint_state.state = ff_msgs::ArmJointState::MOVING;
  }
  arm_state_pub_.publish(arm_state_);

  for (i = 0; i < num_iter; i++) {
    if (sas_arm_->isPreemptRequested()) {
      return false;
    }
    joint_sample_.samples[1].angle_pos -= step;
    joint_sample_pub_.publish(joint_sample_);
    ros::Duration(0.05).sleep();
    // Don't have to publish fancy feedback since the executive doesn't use it
    sas_arm_->publishFeedback(feedback);
  }

  // Make tilt angle match angle received exactly so that the angle isn't
  // toggled when sending the same tilt angle again and again and again :)
  joint_sample_.samples[1].angle_pos = angle;
  joint_sample_pub_.publish(joint_sample_);

  if (std::abs(angle - M_PI) < 0.05) {
    arm_state_.joint_state.state = ff_msgs::ArmJointState::STOWED;
  } else {
    arm_state_.joint_state.state = ff_msgs::ArmJointState::STOPPED;
  }
  arm_state_pub_.publish(arm_state_);
  return true;
}

void ArmSim::GoalCallback(ff_msgs::ArmGoalConstPtr const& goal) {
  ff_msgs::ArmFeedback feedback;
  ff_msgs::ArmResult result;

  // Don't have to publish fancy feedback since the executive doesn't use it
  sas_arm_->publishFeedback(feedback);

  if (goal->command == ff_msgs::ArmGoal::ARM_MOVE) {
    // Do tilt first because right now we use tilt to unstow
    if (Tilt(goal->tilt)) {
      sas_arm_->publishFeedback(feedback);

      Pan(goal->pan);

      sas_arm_->publishFeedback(feedback);
    }
  } else if (goal->command == ff_msgs::ArmGoal::ARM_PAN) {
    Pan(goal->pan);

    sas_arm_->publishFeedback(feedback);
  } else if (goal->command == ff_msgs::ArmGoal::ARM_TILT) {
    Tilt(goal->tilt);

    sas_arm_->publishFeedback(feedback);
  } else if (goal->command == ff_msgs::ArmGoal::GRIPPER_OPEN) {
    sas_arm_->publishFeedback(feedback);

    joint_sample_.samples[2].angle_pos = 45 * M_PI/180;
    joint_sample_pub_.publish(joint_sample_);

    sas_arm_->publishFeedback(feedback);

    arm_state_.gripper_state.state = ff_msgs::ArmGripperState::OPEN;
    arm_state_pub_.publish(arm_state_);
  } else if (goal->command == ff_msgs::ArmGoal::GRIPPER_CLOSE) {
    sas_arm_->publishFeedback(feedback);

    joint_sample_.samples[2].angle_pos = 20 * M_PI/180;
    joint_sample_pub_.publish(joint_sample_);

    sas_arm_->publishFeedback(feedback);

    arm_state_.gripper_state.state = ff_msgs::ArmGripperState::CLOSED;
    arm_state_pub_.publish(arm_state_);
  } else {
    ROS_ERROR("Arm simulator node doesn't recognize mode %i!", goal->command);
    result.response = ff_msgs::ArmResult::INVALID_COMMAND;
    sas_arm_->setAborted(result);
  }
  result.response = ff_msgs::ArmResult::SUCCESS;
  sas_arm_->setSucceeded(result);
}

void ArmSim::onInit() {
  nh_ = getNodeHandle();

  arm_state_pub_ = nh_.advertise<ff_msgs::ArmStateStamped>(
                      TOPIC_PROCEDURES_ARM_ARM_STATE, pub_queue_size_, true);
  joint_sample_pub_ = nh_.advertise<ff_msgs::JointSampleStamped>(
              TOPIC_PROCEDURES_ARM_JOINT_SAMPLE, pub_queue_size_, true);

  arm_state_.joint_state.state = ff_msgs::ArmJointState::STOWED;
  arm_state_.gripper_state.state = ff_msgs::ArmGripperState::CLOSED;

  joint_sample_.samples.resize(3);
  for (int i = 0; i < 3; i++) {
    joint_sample_.samples[i].angle_pos = 0;
    joint_sample_.samples[i].angle_vel = 0;
    joint_sample_.samples[i].angle_acc = 0;
    joint_sample_.samples[i].current = 0;
    joint_sample_.samples[i].torque = 0;
    joint_sample_.samples[i].status = ff_msgs::JointSample::JOINT_ENABLED;
  }

  joint_sample_.samples[0].name = "pan";
  joint_sample_.samples[0].temperature = 26;

  joint_sample_.samples[1].name = "tilt";
  joint_sample_.samples[1].temperature = 23;
  joint_sample_.samples[1].angle_pos = M_PI;

  joint_sample_.samples[2].name = "gripper";
  joint_sample_.samples[2].temperature = 32;
  joint_sample_.samples[2].angle_pos = 20 * M_PI/180;

  arm_state_pub_.publish(arm_state_);
  joint_sample_pub_.publish(joint_sample_);

  sas_arm_ = std::make_shared<actionlib::SimpleActionServer<ff_msgs::ArmAction>>
                                  (nh_, ACTION_PROCEDURES_ARM,
                                  boost::bind(&ArmSim::GoalCallback, this, _1),
                                  false);
  sas_arm_->start();
}
}  // namespace arm_sim

PLUGINLIB_EXPORT_CLASS(arm_sim::ArmSim, nodelet::Nodelet)
