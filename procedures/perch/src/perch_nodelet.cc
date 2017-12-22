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

// Standard includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// TF2 support
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Shared project includes
#include <msg_conversions/msg_conversions.h>
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_service.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>

// Messages
#include <ff_msgs/ArmStateStamped.h>
#include <ff_msgs/ArmJointState.h>
#include <ff_msgs/ArmGripperState.h>

// Services
#include <ff_msgs/SetBool.h>

// Actions
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/SwitchAction.h>
#include <ff_msgs/ArmAction.h>
#include <ff_msgs/PerchAction.h>

/**
 * \ingroup proc
 */
namespace perch {

// Match the internal states and responses with the message definition
using STATE = ff_msgs::PerchFeedback;
using RESPONSE = ff_msgs::PerchResult;

class PerchNodelet : public ff_util::FreeFlyerNodelet {
 public:
  PerchNodelet() : ff_util::FreeFlyerNodelet(NODE_PERCH, true),
    state_(STATE::INITIALIZING) {}
  virtual ~PerchNodelet() {}

 protected:
  // All possible events that can occur
  enum PercherEvent {
    READY,              // System is initialized
    PERCH,              // Start perching
    UNPERCH,            // Stop perching
    STATE_UNPERCHED,    // We know that we are unperched
    STATE_ACTIVE,       // We know that the arm is active
    SWITCH_SUCCESS,     // Localization manager switch result
    SWITCH_PROBLEM,     // Localization manager switch failed
    MOVE_SUCCESS,       // Mobility move success
    MOVE_PROBLEM,       // Mobility move problem
    ARM_SUCCESS,        // Arm success
    ARM_PROBLEM         // Arm problem
  };

  // Poses that are useful in the perch procedure
  enum PercherPose {
    APPROACH_POSE,      // The starting poses of perching
    COMPLETE_POSE,      // The completion pose of perching
    ATTACHED_POSE       // A small delta pose for checking attachment
  };

  // Arm events for the percher
  enum PercherArm {
    ARM_DEPLOY,         // Deploy the arm
    ARM_CALIBRATE,      // Calibrate the gripper
    ARM_OPEN,           // Open the gripprt
    ARM_CLOSE,          // Close the gripper
    ARM_RESET,          // Reset the pan/tilt to (0,0)
    ARM_STOW            // Stow the arm
  };

  // Called to initialize this nodelet
  void Initialize(ros::NodeHandle *nh) {
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.Initialize(GetPrivateHandle(), "proc/perch.config");
    if (!cfg_.Listen(boost::bind(
      &PerchNodelet::ReconfigureCallback, this, _1)))
      return AssertFault("INITIALIZATION_FAULT", "Could not load config");

    // The handrail frame is always named the same
    frame_ = FRAME_NAME_HANDRAIL;
    if (!GetPlatform().empty())
      frame_ = GetPlatform() + std::string("/") + frame_;

    // Create a transform buffer to listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(tf_buffer_));

    // Subscribe to be notified when the dock state changes
    sub_s_ = nh->subscribe(TOPIC_PROCEDURES_ARM_STATE, 5,
      &PerchNodelet::ArmStateCallback, this);

    // Enable/disable control (propulsion)
    client_e_.SetConnectedTimeout(cfg_.Get<double>("timeout_enable_connected"));
    client_e_.SetConnectedCallback(std::bind(
      &PerchNodelet::ConnectedCallback, this));
    client_e_.SetTimeoutCallback(std::bind(
      &PerchNodelet::EnableTimeoutCallback, this));
    client_e_.Create(nh, SERVICE_GNC_CTL_ENABLE);

    // Setup arm action client
    client_a_.SetConnectedTimeout(cfg_.Get<double>("timeout_arm_connected"));
    client_a_.SetActiveTimeout(cfg_.Get<double>("timeout_arm_active"));
    client_a_.SetResponseTimeout(cfg_.Get<double>("timeout_arm_response"));
    client_a_.SetFeedbackCallback(std::bind(&PerchNodelet::AFeedbackCallback,
      this, std::placeholders::_1));
    client_a_.SetResultCallback(std::bind(&PerchNodelet::AResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_a_.SetConnectedCallback(std::bind(
      &PerchNodelet::ConnectedCallback, this));
    client_a_.Create(nh, ACTION_PROCEDURES_ARM);

    // Setup teleop action client
    client_t_.SetConnectedTimeout(cfg_.Get<double>("timeout_motion_connected"));
    client_t_.SetActiveTimeout(cfg_.Get<double>("timeout_motion_active"));
    client_t_.SetResponseTimeout(cfg_.Get<double>("timeout_motion_response"));
    client_t_.SetFeedbackCallback(std::bind(&PerchNodelet::MFeedbackCallback,
      this, std::placeholders::_1));
    client_t_.SetResultCallback(std::bind(&PerchNodelet::MResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_t_.SetConnectedCallback(std::bind(
      &PerchNodelet::ConnectedCallback, this));
    client_t_.Create(nh, ACTION_MOBILITY_MOTION);

    // Setup switch client action
    client_s_.SetConnectedTimeout(cfg_.Get<double>("timeout_switch_connected"));
    client_s_.SetActiveTimeout(cfg_.Get<double>("timeout_switch_active"));
    client_s_.SetResponseTimeout(cfg_.Get<double>("timeout_switch_response"));
    client_s_.SetDeadlineTimeout(cfg_.Get<double>("timeout_switch_deadline"));
    client_s_.SetFeedbackCallback(std::bind(
      &PerchNodelet::SFeedbackCallback, this, std::placeholders::_1));
    client_s_.SetResultCallback(std::bind(&PerchNodelet::SResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_s_.SetConnectedCallback(std::bind(&PerchNodelet::ConnectedCallback,
      this));
    client_s_.Create(nh, ACTION_LOCALIZATION_MANAGER_SWITCH);

    // Setup the execute action
    server_.SetGoalCallback(std::bind(
      &PerchNodelet::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &PerchNodelet::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &PerchNodelet::CancelCallback, this));
    server_.Create(nh, ACTION_PROCEDURES_PERCH);

    // Read the approach and complete poses
    config_reader::ConfigReader* cfg = cfg_.GetConfigReader();
    if (!msg_conversions::config_read_transform(cfg, "approach_pose", &tf_a_))
      return AssertFault("INITIALIZATION_FAULT", "Cannot read approach pose");
    if (!msg_conversions::config_read_transform(cfg, "complete_pose", &tf_c_))
      return AssertFault("INITIALIZATION_FAULT", "Cannot read complete pose");
    if (!msg_conversions::config_read_transform(cfg, "attached_pose", &tf_t_))
      return AssertFault("INITIALIZATION_FAULT", "Cannot read attached pose");
  }

  // Timeout on a trajectory generation request
  void EnableTimeoutCallback(void) {
    return AssertFault("INITIALIZATION_FAULT", "Could not find enable service");
  }

  // Ensure all clients are connected
  void ConnectedCallback() {
    NODELET_DEBUG_STREAM("ConnectedCallback()");
    if (!client_e_.IsConnected()) return;       // Enable service
    if (!client_s_.IsConnected()) return;       // Switch action
    if (!client_t_.IsConnected()) return;       // Motion action
    if (!client_a_.IsConnected()) return;       // Arm action
    if (state_ == STATE::INITIALIZING)          // Prevent multiple calls
      FiniteStateMachine(READY);
  }

  // Current dock state as published by the dock hardware driver
  void ArmStateCallback(ff_msgs::ArmStateStamped::ConstPtr const& msg) {
    // If the joint state is stowed then we know we are not perched
    if (msg->joint_state.state == ff_msgs::ArmJointState::STOWED)
      return FiniteStateMachine(STATE_UNPERCHED);
    // If the gripper state is open then we know we are not perched
    if (msg->gripper_state.state == ff_msgs::ArmGripperState::OPEN)
      return FiniteStateMachine(STATE_UNPERCHED);
    // In all other cases we actually don't know our perch state, and the only
    // way we can determine the state is by testing attachment
    return FiniteStateMachine(STATE_ACTIVE);
  }

  // Complete the current dock or undock action
  void Complete(int32_t response) {
    switch (state_) {
    // Goal-tracking states
    case STATE::PERCHING_SWITCHING_TO_HR_LOC:
    case STATE::PERCHING_MOVING_TO_APPROACH_POSE:
    case STATE::PERCHING_DEPLOYING_ARM:
    case STATE::PERCHING_OPENING_GRIPPER:
    case STATE::PERCHING_CALIBRATING_GRIPPER:
    case STATE::PERCHING_INGRESSING:
    case STATE::PERCHING_CLOSING_GRIPPER:
    case STATE::PERCHING_CHECKING_ATTACHED:
    case STATE::PERCHING_SWITCHING_TO_PL_LOC:
    case STATE::UNPERCHING_RESETTING_PAN_TILT:
    case STATE::UNPERCHING_SWITCHING_TO_HR_LOC:
    case STATE::UNPERCHING_OPENING_GRIPPER:
    case STATE::UNPERCHING_CALIBRATING_GRIPPER:
    case STATE::UNPERCHING_EGRESSING:
    case STATE::UNPERCHING_CLOSING_GRIPPER:
    case STATE::UNPERCHING_STOWING_ARM:
    case STATE::UNPERCHING_SWITCHING_TO_ML_LOC: {
      ff_msgs::PerchResult result;
      result.response = response;
      if (response > 0)
        server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
      else if (response < 0)
        server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
      else
        server_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
      break;
    }
    // Goal-free states
    case STATE::UNKNOWN:
    case STATE::INITIALIZING:
    case STATE::TESTING_SWITCHING_TO_HR_LOC:
    case STATE::TESTING_SWITCHING_TO_PL_LOC:
    case STATE::UNPERCHED:
    case STATE::PERCHED:
      NODELET_INFO("Complete() should not be called in a non-action state");
      break;
    }
  }

  // Update the docker state -- make sure that if the state changes, then we
  // convert the deep arm state into the abstract joint and gripper states
  // and send a message to the ROS backbone indicating the state change.
  void UpdateState(uint8_t state) {
    // Debug state changes
    std::string str = "UNKNOWN";
    switch (state) {
    case STATE::INITIALIZING:
      str = "INITIALIZING";                     break;
    case STATE::UNKNOWN:
      str = "UNKNOWN";                          break;
    case STATE::TESTING_SWITCHING_TO_HR_LOC:
      str = "TESTING_SWITCHING_TO_HR_LOC";      break;
    case STATE::TESTING_SWITCHING_TO_PL_LOC:
      str = "TESTING_SWITCHING_TO_PL_LOC";      break;
    case STATE::UNPERCHED:
      str = "UNPERCHED";                        break;
    case STATE::PERCHING_SWITCHING_TO_HR_LOC:
      str = "PERCHING_SWITCHING_TO_HR_LOC";     break;
    case STATE::PERCHING_MOVING_TO_APPROACH_POSE:
      str = "PERCHING_MOVING_TO_APPROACH_POSE"; break;
    case STATE::PERCHING_DEPLOYING_ARM:
      str = "PERCHING_DEPLOYING_ARM";           break;
    case STATE::PERCHING_OPENING_GRIPPER:
      str = "PERCHING_OPENING_GRIPPER";         break;
    case STATE::PERCHING_CALIBRATING_GRIPPER:
      str = "PERCHING_CALIBRATING_GRIPPER";     break;
    case STATE::PERCHING_INGRESSING:
      str = "PERCHING_INGRESSING";              break;
    case STATE::PERCHING_CLOSING_GRIPPER:
      str = "PERCHING_CLOSING_GRIPPER";         break;
    case STATE::PERCHING_CHECKING_ATTACHED:
      str = "PERCHING_CHECKING_ATTACHED";       break;
    case STATE::PERCHING_SWITCHING_TO_PL_LOC:
      str = "PERCHING_SWITCHING_TO_PL_LOC";     break;
    case STATE::PERCHED:
      str = "PERCHED";                          break;
    case STATE::UNPERCHING_RESETTING_PAN_TILT:
      str = "UNPERCHING_RESETTING_PAN_TILT";    break;
    case STATE::UNPERCHING_SWITCHING_TO_HR_LOC:
      str = "UNPERCHING_SWITCHING_TO_HR_LOC";   break;
    case STATE::UNPERCHING_OPENING_GRIPPER:
      str = "UNPERCHING_OPENING_GRIPPER";       break;
    case STATE::UNPERCHING_CALIBRATING_GRIPPER:
      str = "UNPERCHING_CALIBRATING_GRIPPER";   break;
    case STATE::UNPERCHING_EGRESSING:
      str = "UNPERCHING_EGRESSING";             break;
    case STATE::UNPERCHING_CLOSING_GRIPPER:
      str = "UNPERCHING_CLOSING_GRIPPER";       break;
    case STATE::UNPERCHING_STOWING_ARM:
      str = "UNPERCHING_STOWING_ARM";           break;
    case STATE::UNPERCHING_SWITCHING_TO_ML_LOC:
      str = "UNPERCHING_SWITCHING_TO_ML_LOC";   break;
    }
    NODELET_DEBUG_STREAM("State changed to " << str);
    // Set the state
    state_ = state;
    // Send feedback if needed
    switch (state_) {
    case STATE::PERCHING_SWITCHING_TO_HR_LOC:
    case STATE::PERCHING_MOVING_TO_APPROACH_POSE:
    case STATE::PERCHING_DEPLOYING_ARM:
    case STATE::PERCHING_OPENING_GRIPPER:
    case STATE::PERCHING_CALIBRATING_GRIPPER:
    case STATE::PERCHING_INGRESSING:
    case STATE::PERCHING_CLOSING_GRIPPER:
    case STATE::PERCHING_CHECKING_ATTACHED:
    case STATE::PERCHING_SWITCHING_TO_PL_LOC:
    case STATE::UNPERCHING_RESETTING_PAN_TILT:
    case STATE::UNPERCHING_SWITCHING_TO_HR_LOC:
    case STATE::UNPERCHING_OPENING_GRIPPER:
    case STATE::UNPERCHING_CALIBRATING_GRIPPER:
    case STATE::UNPERCHING_EGRESSING:
    case STATE::UNPERCHING_CLOSING_GRIPPER:
    case STATE::UNPERCHING_STOWING_ARM:
    case STATE::UNPERCHING_SWITCHING_TO_ML_LOC: {
      ff_msgs::PerchFeedback feedback;
      feedback.state = state_;
      server_.SendFeedback(feedback);
      break;
    }
    case STATE::UNKNOWN:
    case STATE::INITIALIZING:
    case STATE::TESTING_SWITCHING_TO_HR_LOC:
    case STATE::TESTING_SWITCHING_TO_PL_LOC:
    case STATE::UNPERCHED:
    case STATE::PERCHED:
    default:
      break;
    }
  }

  // The state machine uses an event to move the state forward
  void FiniteStateMachine(PercherEvent event) {
    // Debug events
    std::string str = "UNKNOWN";
    switch (event) {
    case READY:           str = "READY";            break;
    case PERCH:           str = "PERCH";            break;
    case UNPERCH:         str = "UNPERCH";          break;
    case STATE_UNPERCHED: str = "STATE_UNPERCHED";  break;
    case STATE_ACTIVE:    str = "STATE_ACTIVE";     break;
    case SWITCH_SUCCESS:  str = "SWITCH_SUCCESS";   break;
    case SWITCH_PROBLEM:  str = "SWITCH_PROBLEM";   break;
    case MOVE_SUCCESS:    str = "MOVE_SUCCESS";     break;
    case MOVE_PROBLEM:    str = "MOVE_PROBLEM";     break;
    case ARM_SUCCESS:     str = "ARM_SUCCESS";      break;
    case ARM_PROBLEM:     str = "ARM_PROBLEM";      break;
    }
    NODELET_DEBUG_STREAM("Received event " << str);

    // Now do something based on the state
    switch (state_) {
    // We have not yet connected to our client services, so we are unable to
    // offser perch support until this has completed.
    case STATE::INITIALIZING:
      switch (event) {
      case READY:
        return UpdateState(STATE::UNKNOWN);
      default:
        break;
      }
      break;

    // We don't know if we are perched or not. We need to wait for the arm
    // node to send state information so that we can determine if we are
    // unperched or in an active state before we can proceed
    case STATE::UNKNOWN:
      switch (event) {
      // We know the arm is NOT stowed. To determine if we are, in fact,
      // attached to the handrail we will turn on handrail localization. If it
      // fails, then we are not PERCHED. If it suceeds, we must check agreement
      // between the joint angles and the location of the handrail. It would
      // have been so much easier if we had gripper feedback...
      case STATE_ACTIVE:
        if (!SendSwitch("hr", STATE::TESTING_SWITCHING_TO_HR_LOC))
          return Complete(RESPONSE::SWITCH_FAILED);
      case STATE_UNPERCHED:
        return UpdateState(STATE::UNPERCHED);
      default:
        break;
      }
      break;

    // We are busy switching to hardrail localization to test if a handrail
    // is in view. This gives us a clue as to whether we are perched or not.
    case STATE::TESTING_SWITCHING_TO_HR_LOC:
      switch (event) {
      // If we can switch to handrail localization a handrail is in view
      case SWITCH_SUCCESS:
        // If our joint angles agree with the handrail's position, it means
        // that we are very likely perched on the hadrail. The gripper state
        // will remain unknown until we calibrate as part of the unperch.
        if (CheckPerched()) {
          if (!SendSwitch("pl", STATE::PERCHING_SWITCHING_TO_PL_LOC))
            return AssertFault("INITIALIZATION_FAULT", "Cannot switch to pl");
        } else {
          return UpdateState(STATE::UNPERCHED);
        }
      // If we cannot switch to handrail localizaiton it means a handrail is
      // probably not in view. This means we are not perched.
      case SWITCH_PROBLEM:
        return UpdateState(STATE::UNPERCHED);
      default:
        break;
      }
      break;

    // We are perched, so we should force the localization mode to perch
    // localization, so that the user can pan and tilt immediately. However,
    // we don't want to reset the pose in case it has been manually set.
    case STATE::TESTING_SWITCHING_TO_PL_LOC:
      switch (event) {
      case SWITCH_SUCCESS:
        return UpdateState(STATE::PERCHED);
      case SWITCH_PROBLEM:
        return AssertFault("INITIALIZATION_FAULT", "Cannot switch to pl");
      default:
        break;
      }
      break;

    // We are in an unperched state.
    case STATE::UNPERCHED:
      switch (event) {
      case UNPERCH:
        return Complete(RESPONSE::ALREADY_UNPERCHED);
      case PERCH:
        if (!SendSwitch("hr", STATE::PERCHING_SWITCHING_TO_HR_LOC))
          return Complete(RESPONSE::SWITCH_FAILED);
      default:
        break;
      }
      break;

    // We are busy switching to handrail localization
    case STATE::PERCHING_SWITCHING_TO_HR_LOC:
      switch (event) {
      case SWITCH_SUCCESS:
        if (!SendMove(APPROACH_POSE, STATE::PERCHING_MOVING_TO_APPROACH_POSE))
          return Complete(RESPONSE::MOTION_FAILED);
      case SWITCH_PROBLEM:
        return Complete(RESPONSE::SWITCH_TO_HR_LOC_FAILED);
      default:
        break;
      }
      break;

    // We are busy moving to the approach pose
    case STATE::PERCHING_MOVING_TO_APPROACH_POSE:
      switch (event)
      case MOVE_SUCCESS: {
        if (!SendArm(ARM_DEPLOY, STATE::PERCHING_DEPLOYING_ARM))
          return Complete(RESPONSE::ARM_FAILED);
      case MOVE_PROBLEM:
        return Complete(RESPONSE::APPROACH_FAILED);
      default:
        break;
      }
      break;

    // We are waiting for the arm to deploy
    case STATE::PERCHING_DEPLOYING_ARM:
      switch (event)
      case ARM_SUCCESS: {
        if (!SendArm(ARM_OPEN, STATE::PERCHING_OPENING_GRIPPER))
          return Complete(RESPONSE::ARM_FAILED);
      case ARM_PROBLEM:
        return Complete(RESPONSE::DEPLOY_FAILED);
      default:
        break;
      }
      break;

    // We are waiting for the gripper to open
    case STATE::PERCHING_OPENING_GRIPPER:
      switch (event) {
      case ARM_SUCCESS:
        if (!SendMove(COMPLETE_POSE, STATE::PERCHING_INGRESSING))
          return Complete(RESPONSE::MOTION_FAILED);
      case ARM_PROBLEM:
        if (!SendArm(ARM_CALIBRATE, STATE::PERCHING_CALIBRATING_GRIPPER))
          return Complete(RESPONSE::ARM_FAILED);
      default:
        break;
      }
      break;

    // We are waiting for the gripper to complete the calibration phase
    case STATE::PERCHING_CALIBRATING_GRIPPER:
      switch (event)
      case ARM_SUCCESS: {
        if (!SendArm(ARM_OPEN, STATE::PERCHING_OPENING_GRIPPER))
          return Complete(RESPONSE::ARM_FAILED);
      case ARM_PROBLEM:
        return Complete(RESPONSE::CALIBRATE_FAILED);
      default:
        break;
      }
      break;


    // We are ingressing towards the handrail
    case STATE::PERCHING_INGRESSING:
      switch (event) {
      case MOVE_SUCCESS:
        if (!SendArm(ARM_CLOSE, STATE::PERCHING_CLOSING_GRIPPER))
          return Complete(RESPONSE::ARM_FAILED);
      case MOVE_PROBLEM:
        return Complete(RESPONSE::INGRESS_FAILED);
      default:
        break;
      }
      break;

    // We are closing the gripper
    case STATE::PERCHING_CLOSING_GRIPPER:
      switch (event) {
      case ARM_SUCCESS:
        if (!SendMove(ATTACHED_POSE, STATE::PERCHING_CHECKING_ATTACHED))
          return Complete(RESPONSE::MOTION_FAILED);
      case ARM_PROBLEM:
        return Complete(RESPONSE::CLOSE_FAILED);
      default:
        break;
      }
      break;

    // We are verifying attachment
    case STATE::PERCHING_CHECKING_ATTACHED:
      switch (event)
      case MOVE_PROBLEM: {
        if (!SendSwitch("pl", STATE::PERCHING_SWITCHING_TO_PL_LOC))
          return Complete(RESPONSE::SWITCH_FAILED);
      case MOVE_SUCCESS:
        return Complete(RESPONSE::ATTACH_FAILED);
      default:
        break;
      }
      break;

    // We are switching to perch localization
    case STATE::PERCHING_SWITCHING_TO_PL_LOC:
      switch (event) {
      case SWITCH_SUCCESS:
        // Try and disable propulsion
        if (!TogglePropulsion(false))
          return Complete(RESPONSE::TOGGLE_FAILED);
        // Switch to a docked state
        UpdateState(STATE::PERCHED);
        // Success!
        return Complete(RESPONSE::PERCHED);
      case SWITCH_PROBLEM:
        return Complete(RESPONSE::SWITCH_TO_PL_LOC_FAILED);
      default:
        break;
      }
      break;

    // We are perched
    case STATE::PERCHED:
      switch (event) {
      case PERCH:
        return Complete(RESPONSE::ALREADY_PERCHED);
      case UNPERCH:
        if (!TogglePropulsion(true))
          return Complete(RESPONSE::TOGGLE_FAILED);
        if (!SendArm(ARM_RESET, STATE::UNPERCHING_RESETTING_PAN_TILT))
          return Complete(RESPONSE::ARM_FAILED);
      case STATE_UNPERCHED:
        return UpdateState(STATE::UNPERCHED);
      default:
        break;
      }
      break;

    // We are returning to our attached pose, which
    case STATE::UNPERCHING_RESETTING_PAN_TILT:
      switch (event) {
      case ARM_SUCCESS:
        if (!SendSwitch("hr", STATE::UNPERCHING_SWITCHING_TO_HR_LOC))
          return Complete(RESPONSE::SWITCH_FAILED);
      case ARM_PROBLEM:
        return Complete(RESPONSE::RESET_FAILED);
      default:
        break;
      }
      break;

    // We are switching to handrail localization
    case STATE::UNPERCHING_SWITCHING_TO_HR_LOC:
      switch (event) {
      case SWITCH_SUCCESS:
        if (!SendArm(ARM_OPEN, STATE::UNPERCHING_OPENING_GRIPPER))
          return Complete(RESPONSE::ARM_FAILED);
      case SWITCH_PROBLEM:
        return Complete(RESPONSE::SWITCH_TO_HR_LOC_FAILED);
      default:
        break;
      }
      break;

    // We are moving to the attached pose (just in case we )
    case STATE::UNPERCHING_OPENING_GRIPPER:
      switch (event) {
      case ARM_SUCCESS:
        if (!SendMove(APPROACH_POSE, STATE::UNPERCHING_EGRESSING))
          return Complete(RESPONSE::ARM_FAILED);
      case ARM_PROBLEM:
        if (!SendArm(ARM_OPEN, STATE::UNPERCHING_CALIBRATING_GRIPPER))
          return Complete(RESPONSE::ARM_FAILED);
      default:
        break;
      }
      break;

    // We are moving to the attached pose (just in case we )
    case STATE::UNPERCHING_CALIBRATING_GRIPPER:
      switch (event) {
      case ARM_SUCCESS:
        if (!SendArm(ARM_OPEN, STATE::UNPERCHING_OPENING_GRIPPER))
          return Complete(RESPONSE::ARM_FAILED);
      case ARM_PROBLEM:
        return Complete(RESPONSE::CALIBRATE_FAILED);
      default:
        break;
      }
      break;

    // We are egressing away from the handrail
    case STATE::UNPERCHING_EGRESSING:
      switch (event) {
      case MOVE_SUCCESS:
        if (!SendArm(ARM_CLOSE, STATE::PERCHING_CLOSING_GRIPPER))
          return Complete(RESPONSE::ARM_FAILED);
      case MOVE_PROBLEM:
        return Complete(RESPONSE::EGRESS_FAILED);
      default:
        break;
      }
      break;

    // We are closing the gripper
    case STATE::UNPERCHING_CLOSING_GRIPPER:
      switch (event) {
      case ARM_SUCCESS:
        if (!SendArm(ARM_STOW, STATE::UNPERCHING_STOWING_ARM))
          return Complete(RESPONSE::ARM_FAILED);
      case ARM_PROBLEM:
        return Complete(RESPONSE::RESET_FAILED);
      default:
        break;
      }
      break;

    // We are stowing the arm
    case STATE::UNPERCHING_STOWING_ARM:
      switch (event) {
      case ARM_SUCCESS:
        if (!SendSwitch("ml", STATE::UNPERCHING_SWITCHING_TO_ML_LOC))
          return Complete(RESPONSE::SWITCH_FAILED);
      case ARM_PROBLEM:
        return Complete(RESPONSE::ARM_FAILED);
      default:
        break;
      }
      break;

    // We are switching to mapped landmark navigation
    case STATE::UNPERCHING_SWITCHING_TO_ML_LOC:
      switch (event) {
      case SWITCH_SUCCESS:
        // We have now undocked
        UpdateState(STATE::UNPERCHED);
        // Send success tot he calling node
        return Complete(RESPONSE::UNPERCHED);
      case SWITCH_PROBLEM:
        return Complete(RESPONSE::SWITCH_TO_ML_LOC_FAILED);
      default:
        break;
      }
      break;

    // Ignore states that simply do not exist
    default:
      NODELET_ERROR("System in bad state");
      break;
    }

    // State machine debugging
    NODELET_DEBUG("No action taken.");
  }

  // CHECK IF WE ARE PERCHED

  // We can only implement this when perch localization is complete...
  bool CheckPerched() {
    return true;
  }

  // ENABLE PROPULSION

  bool TogglePropulsion(bool enable) {
    ff_msgs::SetBool msg;
    msg.request.enable = enable;
    if (!client_e_.Call(msg))
      return false;
    return true;
  }

  // SWITCH action

  // Helper function for localization switching
  bool SendSwitch(std::string const& pipeline, uint8_t new_state) {
    // Send the switch goal
    ff_msgs::SwitchGoal goal;
    goal.pipeline = pipeline;
    if (!client_s_.SendGoal(goal))
      return false;
    // Switch to the new state
    UpdateState(new_state);
    return true;
  }

  // Ignore the switch feedback for now
  void SFeedbackCallback(ff_msgs::SwitchFeedbackConstPtr const& feedback) {}

  // Do something with the switch result
  void SResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::SwitchResultConstPtr const& result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return FiniteStateMachine(SWITCH_SUCCESS);
    default:
      return FiniteStateMachine(SWITCH_PROBLEM);
    }
  }

  // MOVE ACTION CLIENT

  // Helper function for moving
  bool SendMove(PercherPose type, uint8_t new_state) {
    // Find the required pose type in the world frame
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_;
    switch (type) {
    case COMPLETE_POSE:
      msg.pose.position.x = tf_c_.translation.x;
      msg.pose.position.y = tf_c_.translation.y;
      msg.pose.position.z = tf_c_.translation.z;
      msg.pose.orientation = tf_c_.rotation;
      break;
    case APPROACH_POSE:
      msg.pose.position.x = tf_a_.translation.x;
      msg.pose.position.y = tf_a_.translation.y;
      msg.pose.position.z = tf_a_.translation.z;
      msg.pose.orientation = tf_a_.rotation;
      break;
    case ATTACHED_POSE:
      msg.pose.position.x = tf_t_.translation.x;
      msg.pose.position.y = tf_t_.translation.y;
      msg.pose.position.z = tf_t_.translation.z;
      msg.pose.orientation = tf_t_.rotation;
      break;
    default:
      return false;
    }

    // Get the dock -> world transform
    try {
      // Look up the world -> dock transform
      geometry_msgs::TransformStamped dock2world = tf_buffer_.lookupTransform(
        "world", msg.header.frame_id, ros::Time(0));
      // Transform the message into the world frame
      tf2::doTransform(msg, msg, dock2world);
    } catch (tf2::TransformException &ex) {
      NODELET_WARN_STREAM("Transform failed" << ex.what());
      return false;
    }

    // Reconfigure the choreographer
    ff_util::ConfigClient cfg(GetPlatformHandle(), NODE_CHOREOGRAPHER);
    cfg.Set<bool>("enable_collision_checking", false);
    cfg.Set<bool>("enable_validation", false);
    cfg.Set<bool>("enable_bootstrapping", true);
    cfg.Set<bool>("enable_immediate", true);
    cfg.Set<bool>("enable_faceforward", false);
    cfg.Set<bool>("enable_control", true);
    cfg.Set<double>("desired_vel",   cfg_.Get<double>("vel"));
    cfg.Set<double>("desired_accel", cfg_.Get<double>("accel"));
    cfg.Set<double>("desired_omega", cfg_.Get<double>("omega"));
    cfg.Set<double>("desired_alpha", cfg_.Get<double>("alpha"));
    cfg.Set<std::string>("planner", "trapezoidal");
    if (!cfg.Reconfigure())
      return false;

    // Send the goal to the mobility subsystem
    ff_msgs::MotionGoal goal;
    goal.command = ff_msgs::MotionGoal::MOVE;
    goal.flight_mode = "perching";
    goal.states.push_back(msg);
    if (!client_t_.SendGoal(goal))
      return false;

    // Switch to the new state and return success
    UpdateState(new_state);
    return true;
  }

  // Feedback from a teleop action
  void MFeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback) {}

  // Result of a teleop action
  void MResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::MotionResultConstPtr const& result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return FiniteStateMachine(MOVE_SUCCESS);
    default:
      return FiniteStateMachine(MOVE_PROBLEM);
    }
  }

  // ARM ACTION CLIENT

  // Helper function for moving
  bool SendArm(PercherArm command, uint8_t new_state) {
    ff_msgs::ArmGoal goal;
    switch (command) {
    default: return false;
    case ARM_DEPLOY:
      goal.command = ff_msgs::ArmGoal::ARM_DEPLOY;
      break;
    case ARM_CALIBRATE:
      goal.command = ff_msgs::ArmGoal::GRIPPER_CALIBRATE;
      break;
    case ARM_OPEN:
      goal.command = ff_msgs::ArmGoal::GRIPPER_OPEN;
      break;
    case ARM_CLOSE:
      goal.command = ff_msgs::ArmGoal::GRIPPER_CLOSE;
      break;
    case ARM_STOW:
      goal.command = ff_msgs::ArmGoal::ARM_STOW;
      break;
    case ARM_RESET:
      goal.command = ff_msgs::ArmGoal::ARM_MOVE;
      goal.pan  = 0;
      goal.tilt = 0;
      break;
    }
    // Try and send the goal
    if (!client_a_.SendGoal(goal))
      return false;
    // Switch to the new state and return success
    UpdateState(new_state);
    return true;
  }

  // Ignore the move feedback, for now
  void AFeedbackCallback(ff_msgs::ArmFeedbackConstPtr const& feedback) {}

  // Result of a move action
  void AResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::ArmResultConstPtr const& result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return FiniteStateMachine(ARM_SUCCESS);
    default:
      return FiniteStateMachine(ARM_PROBLEM);
    }
  }

  // PERCH ACTION SERVER

  // A new arm action has been called
  void GoalCallback(ff_msgs::PerchGoalConstPtr const& goal) {
    switch (goal->command) {
    default:
      return Complete(RESPONSE::INVALID_COMMAND);
    case ff_msgs::PerchGoal::PERCH:
      return FiniteStateMachine(PERCH);
    case ff_msgs::PerchGoal::UNPERCH:
      return FiniteStateMachine(UNPERCH);
    }
  }

  // Preempt the current action with a new action
  void PreemptCallback() {
    return Complete(RESPONSE::PREEMPTED);
  }

  // A Cancellation request arrives
  void CancelCallback() {
    return Complete(RESPONSE::CANCELLED);
  }

  // RECONFIGURE REQUESTS

  // When a new reconfigure request comes in, deal with that request
  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    if (state_ == STATE::UNPERCHED || state_ == STATE::PERCHED)
      return cfg_.Reconfigure(config);
    return false;
  }

 protected:
  uint8_t state_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  ff_util::FreeFlyerActionClient<ff_msgs::MotionAction> client_t_;
  ff_util::FreeFlyerActionClient<ff_msgs::SwitchAction> client_s_;
  ff_util::FreeFlyerActionClient<ff_msgs::ArmAction> client_a_;
  ff_util::FreeFlyerServiceClient<ff_msgs::SetBool> client_e_;
  ff_util::FreeFlyerActionServer<ff_msgs::PerchAction> server_;
  ff_util::ConfigServer cfg_;
  geometry_msgs::Transform tf_a_, tf_c_, tf_t_;
  ros::Subscriber sub_s_;
  std::string frame_;
};

PLUGINLIB_DECLARE_CLASS(perch, PerchNodelet,
                        perch::PerchNodelet, nodelet::Nodelet);

}  // namespace perch
