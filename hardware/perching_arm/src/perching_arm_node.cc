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

// Standard ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>
#include <ff_util/ff_nodelet.h>

// FSW actions, services, messages
#include <actionlib/server/simple_action_server.h>
#include <ff_msgs/ArmAction.h>
#include <ff_msgs/ArmStateStamped.h>
#include <ff_msgs/JointSampleStamped.h>

// interface class
#include <perching_arm/perching_arm.h>

/**
 * \ingroup hw
 */
namespace perching_arm {

typedef actionlib::SimpleActionServer < ff_msgs::ArmAction > PerchingArmAction;

class PerchingArmNode : public ff_util::FreeFlyerNodelet {
 public:
  PerchingArmNode() : ff_util::FreeFlyerNodelet(NODE_PERCHING_ARM) {}
  virtual ~PerchingArmNode() {}

 protected:
  // Called on flight software stack initialization
  virtual void Initialize(ros::NodeHandle *nh) {
    // Read the configuration
    config_reader::ConfigReader config_params;
    config_params.AddFile("hw/perching_arm.config");
    if (!config_params.ReadFiles())
      ROS_FATAL("Couldn't read config file");
    std::string port;
    if (!config_params.GetStr("serial_port", &port))
      FF_ERROR("Could not read the serial port from the config");
    uint32_t baud;
    if (!config_params.GetUInt("serial_baud", &baud))
      FF_ERROR("Could not read the serial baud from the config");

    // Start the arm state publisher (the state is latched because it is only updated on event occurence)
    pub_arm_state_ = nh->advertise < ff_msgs::ArmStateStamped > (TOPIC_HARDWARE_PERCHING_ARM_STATE, 1, true);
    pub_joint_sample_ = nh->advertise < ff_msgs::JointSampleStamped > (TOPIC_HARDWARE_PERCHING_ARM_JOINT_SAMPLE, 1);

    // Start the arm action action
    sas_a_ = std::shared_ptr<PerchingArmAction>(new PerchingArmAction(*nh, ACTION_HARDWARE_PERCHING_ARM, false));
    sas_a_->registerGoalCallback(boost::bind(&PerchingArmNode::GoalCallback, this));
    sas_a_->registerPreemptCallback(boost::bind(&PerchingArmNode::PreemptCallback, this));
    sas_a_->start();

    // We don't know the default states when we initialize. We have to wait for the first
    // callback from the proxy driver to populate these states.
    arm_state_.joint_state.state = ff_msgs::ArmJointState::UNKNOWN;
    arm_state_.gripper_state.state = ff_msgs::ArmGripperState::UNKNOWN;

    // Setup some callbacks
    PerchingArmSleepMsCallback cb_sleep = std::bind(&PerchingArmNode::SleepMsCallback, this,
      std::placeholders::_1);
    PerchingArmEventCallback cb_event = std::bind(&PerchingArmNode::EventCallback, this,
      std::placeholders::_1, std::placeholders::_2);
    PerchingArmFeedbackCallback cb_feedback = std::bind(&PerchingArmNode::FeedbackCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // Initialize the arm
    if (arm_.Initialize(port, baud, cb_sleep, cb_event, cb_feedback) != RESULT_SUCCESS)
      FF_ERROR("Could not initialize the arm driver");
  }

  // Complete the current action
  void Complete(uint8_t response) {
    // Action has ended, so publish the state of the perching arm
    PublishState();
    // For safety and predictability, always stop the arm
    if (arm_.Stop() != RESULT_SUCCESS)
      FF_WARN("Could not send a stop action to the perching arm");
    // Now send a response back to the callee
    ff_msgs::ArmResult result;
    result.response = response;
    if (response > 0) {                         // Success
      FF_INFO("ARM action completed successfully");
      sas_a_->setSucceeded(result);
    } else if (response < 0) {
      FF_INFO("ARM action aborted");
      sas_a_->setAborted(result);               // Failure
    } else {
      FF_INFO("ARM action was preempted");
      sas_a_->setPreempted(result);             // Preemption
    }
  }

  // ROS-aware blocking sleep, so that other nodes running in current process don't get blocked
  void SleepMsCallback(uint32_t ms) {
    ros::Duration duration(static_cast<float>(ms) / 1000.0f);
    duration.sleep();
  }

  // Asynchronous callback from the serial driver with feedback/result
  void FeedbackCallback(PerchingArmJointState joint_state, PerchingArmGripperState gripper_state,
    PerchingArmFeedback const& feedback) {
    // Transform the joint and arm states, then publish
    uint8_t backup_joint_state =  arm_state_.joint_state.state;
    switch (joint_state) {
    case JOINT_STATE_UNKNOWN:
      arm_state_.joint_state.state = ff_msgs::ArmJointState::UNKNOWN;
      break;
    case JOINT_STATE_STOPPED:
      arm_state_.joint_state.state = ff_msgs::ArmJointState::STOPPED;
      break;
    case JOINT_STATE_DEPLOYING:
      arm_state_.joint_state.state = ff_msgs::ArmJointState::DEPLOYING;
      break;
    case JOINT_STATE_PANNING:
    case JOINT_STATE_TILTING:
    case JOINT_STATE_MOVING_PANNING:
    case JOINT_STATE_MOVING_TILTING:
      arm_state_.joint_state.state = ff_msgs::ArmJointState::MOVING;
      break;
    case JOINT_STATE_STOWING_PANNING:
    case JOINT_STATE_STOWING_CLOSING:
    case JOINT_STATE_STOWING_TILTING:
      arm_state_.joint_state.state = ff_msgs::ArmJointState::STOWING;
      break;
    case JOINT_STATE_STOWED:
      arm_state_.joint_state.state = ff_msgs::ArmJointState::STOWED;
    }
    uint8_t backup_gripper_state = arm_state_.gripper_state.state;
    switch (gripper_state) {
    case GRIPPER_STATE_UNKNOWN:       arm_state_.gripper_state.state = ff_msgs::ArmGripperState::UNKNOWN;       break;
    case GRIPPER_STATE_CLOSED:        arm_state_.gripper_state.state = ff_msgs::ArmGripperState::CLOSED;        break;
    case GRIPPER_STATE_OPEN:          arm_state_.gripper_state.state = ff_msgs::ArmGripperState::OPEN;          break;
    case GRIPPER_STATE_CALIBRATING:   arm_state_.gripper_state.state = ff_msgs::ArmGripperState::CALIBRATING;   break;
    case GRIPPER_STATE_UNCALIBRATED:  arm_state_.gripper_state.state = ff_msgs::ArmGripperState::UNCALIBRATED;  break;
    default: break;
    }
    if (joint_state != backup_joint_state && gripper_state != backup_gripper_state)
      PublishState();
    // Publish the full joint state
    ff_msgs::JointSampleStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.samples.clear();
    // Tilt angle
    ff_msgs::JointSample tilt;
    tilt.name = "tilt";
    tilt.status = ff_msgs::JointSample::JOINT_ENABLED;
    tilt.angle_pos = feedback.tilt.position;
    tilt.angle_vel = feedback.tilt.velocity;
    tilt.angle_acc = 0;
    tilt.current  = feedback.tilt.load;
    tilt.torque = 0;
    tilt.temperature = 0;
    msg.samples.push_back(tilt);
    // Pan angle
    ff_msgs::JointSample pan;
    pan.name = "pan";
    pan.status = ff_msgs::JointSample::JOINT_ENABLED;
    pan.angle_pos = feedback.pan.position;
    pan.angle_vel = feedback.pan.velocity;
    pan.angle_acc = 0;
    pan.current  = feedback.pan.load;
    pan.torque = 0;
    pan.temperature = 0;
    msg.samples.push_back(pan);
    // Gripper angle
    ff_msgs::JointSample gripper;
    gripper.name = "gripper";
    gripper.status = ff_msgs::JointSample::JOINT_ENABLED;
    gripper.angle_pos = feedback.gripper.position;
    gripper.angle_vel = 0;
    gripper.angle_acc = 0;
    gripper.current  = feedback.gripper.load;
    gripper.torque = 0;
    gripper.temperature = 0;
    msg.samples.push_back(gripper);
    // Publish
    pub_joint_sample_.publish(msg);
  }

  // Lower-rate event callback
  void EventCallback(PerchingArmEvent const& event, float progress) {
    switch (event) {
    case EVENT_PROGRESS: {
        ff_msgs::ArmFeedback msg;
        msg.percentage_complete = progress;
        sas_a_->publishFeedback(msg);
      }
      return;
    // Pan, tilt, move complete events result in a stopped state
    case EVENT_PAN_COMPLETE:
    case EVENT_TILT_COMPLETE:
    case EVENT_MOVE_COMPLETE:
      arm_state_.joint_state.state = ff_msgs::ArmJointState::STOPPED;
      Complete(ff_msgs::ArmResult::SUCCESS);
      break;
    // Stow complete event results in a stowed state
    case EVENT_STOW_COMPLETE:
      arm_state_.joint_state.state = ff_msgs::ArmJointState::STOWED;
      Complete(ff_msgs::ArmResult::SUCCESS);
      break;
    // Deploy complete event results in a stopped state
    case EVENT_DEPLOY_COMPLETE:
      arm_state_.joint_state.state = ff_msgs::ArmJointState::STOPPED;
      Complete(ff_msgs::ArmResult::SUCCESS);
      break;
    // Close complete event results in a closed gripper state
    case EVENT_CLOSE_COMPLETE:
      arm_state_.gripper_state.state = ff_msgs::ArmGripperState::CLOSED;
      Complete(ff_msgs::ArmResult::SUCCESS);
      break;
    // Open complete event results in a open gripper state
    case EVENT_OPEN_COMPLETE:
      arm_state_.gripper_state.state = ff_msgs::ArmGripperState::OPEN;
      Complete(ff_msgs::ArmResult::SUCCESS);
      break;
    // Calibrate complete event results in a closed gripper state
    case EVENT_CALIBRATE_COMPLETE:
      arm_state_.gripper_state.state = ff_msgs::ArmGripperState::CLOSED;
      Complete(ff_msgs::ArmResult::SUCCESS);
      break;
    // Backdrive event results in a stopped arm event
    case EVENT_BACK_DRIVE:
      arm_state_.joint_state.state = ff_msgs::ArmJointState::STOPPED;
      Complete(ff_msgs::ArmResult::BACKDRIVE_DETECTED);
      break;
    // Close complete event results in an error state
    case EVENT_ERROR:
    default:
      Complete(ff_msgs::ArmResult::RUNTIME_ERROR);
      break;
    }
  }

  // A new arm action has been called
  void GoalCallback() {
    // Accept the new goal
    ff_msgs::ArmGoal goal = *sas_a_->acceptNewGoal();
    // Cover the case where we accept a new goal that has already been preempted
    if (sas_a_->isPreemptRequested())
      return PreemptCallback();
    // Nominal case
    switch (goal.mode) {
    // Deploy the arm
    case ff_msgs::ArmGoal::DEPLOY:
      if (arm_.Deploy() != RESULT_SUCCESS)
        return Complete(RESULT_COMMAND_REJECTED);
      arm_state_.joint_state.state = ff_msgs::ArmJointState::DEPLOYING;
      break;
    // Stow the arm
    case ff_msgs::ArmGoal::STOW:
      if (arm_.Stow() != RESULT_SUCCESS)
        return Complete(RESULT_COMMAND_REJECTED);
      arm_state_.joint_state.state = ff_msgs::ArmJointState::STOWING;
      break;
    // Open the gripper
    case ff_msgs::ArmGoal::OPEN:
      if (arm_.Open() != RESULT_SUCCESS)
        return Complete(RESULT_COMMAND_REJECTED);
      break;
    // Close the gripper
    case ff_msgs::ArmGoal::CLOSE:
      if (arm_.Close() != RESULT_SUCCESS)
        return Complete(RESULT_COMMAND_REJECTED);
      break;
    // Calibrate the gripper
    case ff_msgs::ArmGoal::CALIBRATE:
      if (arm_.Calibrate() != RESULT_SUCCESS)
        return Complete(RESULT_COMMAND_REJECTED);
      arm_state_.gripper_state.state = ff_msgs::ArmGripperState::CALIBRATING;
      break;
    // Move the arm (pan, tilt)
    case ff_msgs::ArmGoal::PAN_TILT:
      if (arm_.Move(goal.pan, goal.tilt) != RESULT_SUCCESS)
        return Complete(RESULT_COMMAND_REJECTED);
      arm_state_.joint_state.state = ff_msgs::ArmJointState::MOVING;
      break;
    // Pan the arm
    case ff_msgs::ArmGoal::PAN:
      if (arm_.Pan(goal.pan) != RESULT_SUCCESS)
        return Complete(RESULT_COMMAND_REJECTED);
      arm_state_.joint_state.state = ff_msgs::ArmJointState::MOVING;
      break;
    // Tilt the arm
    case ff_msgs::ArmGoal::TILT:
      if (arm_.Tilt(goal.tilt) != RESULT_SUCCESS)
        return Complete(RESULT_COMMAND_REJECTED);
      arm_state_.joint_state.state = ff_msgs::ArmJointState::MOVING;
      break;
    default:
      return Complete(RESULT_COMMAND_REJECTED);
    }
  }

  // A preemption without a new goal implies a cancel(), which we map to stop
  void PreemptCallback() {
    if (!sas_a_->isNewGoalAvailable())
      return Complete(ff_msgs::ArmResult::CANCELLED);
    return Complete(ff_msgs::ArmResult::PREEMPTED);
  }

  // Publish some information about the arm
  void PublishState() {
    arm_state_.header.stamp = ros::Time::now();
    pub_arm_state_.publish(arm_state_);
  }

 private:
  PerchingArm arm_;                               // Arm interface library
  ff_msgs::ArmStateStamped arm_state_;            // Arm state
  ros::Publisher pub_arm_state_;                  // Arm state publisher
  ros::Publisher pub_joint_sample_;               // Joint state publisher
  std::shared_ptr < PerchingArmAction > sas_a_;   // Action server
};

PLUGINLIB_DECLARE_CLASS(perching_arm, PerchingArmNode,
                        perching_arm::PerchingArmNode, nodelet::Nodelet);

}  // namespace perching_arm
