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
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// Shared project includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>

// Messages
#include <ff_msgs/MobilityResult.h>

// Actions
#include <ff_msgs/ControlAction.h>
#include <ff_msgs/MoveAction.h>
#include <ff_msgs/SwitchAction.h>
#include <ff_msgs/ArmAction.h>
#include <ff_msgs/PerchAction.h>
#include <ff_msgs/UnperchAction.h>

/**
 * \ingroup mobility
 */
namespace percher {

// Sometimes we want to fail
enum PercherMoveResult {
  SUCCESS_DESIRED,
  FAILURE_DESIRED
};

// Positions that we might need to move to
enum PercherPose {
  APPROACH_POSE,
  PERCHED_POSE
};

// All possible states that we can be in
enum PercherState {
  IDLE,                                           // Percher is doing nothing
  PERCHING_SWITCHING_TO_HANDRAIL_LOC,             // Switching to handrail localizations
  PERCHING_MOVING_TO_APPROACH_POINT,              // Moving to the initial approach point
  PERCHING_DEPLOYING_ARM,                         // Deploying the perching arm
  PERCHING_OPENING_GRIPPER,                       // Opening the gripper
  PERCHING_INGRESSING,                            // Moving towards the handrail
  PERCHING_CLOSING_GRIPPER,                       // Closing the gripper
  PERCHING_CHECKING_ATTACHMENT,                   // Checking whether we are attached
  PERCHING_SWITCHING_TO_NONE_LOC,                 // Switching to no localization / control off
  PERCHING_RESETTING_PAN_TILT_POSE,               // Resetting pan tilt pose
  PERCHED,                                        // We are perched!
  UNPERCHING_RESETTING_PAN_TILT_POSE,             // Resetting pan tilt pose
  UNPERCHING_SWITCHING_TO_HANDRAIL_LOC,           // Switching to perch localization
  UNPERCHING_CHECKING_ATTACHMENT,                 // Checking we are attached
  UNPERCHING_OPENING_GRIPPER,                     // Opening the gripper
  UNPERCHING_EGRESSING,                           // Egressing from the handrail
  UNPERCHING_CLOSING_GRIPPER,                     // Closing the gripper
  UNPERCHING_STOWING_ARM,                         // Stowing the perching arm
  UNPERCHING_SWITCHING_TO_MAPPED_LANDMARKS_LOC,   // Switching back to sparse mapping
};

// All possible events that can occur
enum PercherEvent {
  START_PERCH,                                    // Start perching
  START_UNPERCH,                                  // Stop perching
  SUCCESS,                                        // Localization manager switch result
  PROBLEM,                                        // Hardware arm result
  PREEMPT,                                        // Preempt with a new action
  CANCEL                                          // Cancel whatever action is underway
};

class Percher : public ff_util::FreeFlyerNodelet {
 public:
  Percher() : ff_util::FreeFlyerNodelet(NODE_PERCHER, true), state_(IDLE) {}
  virtual ~Percher() {}

 protected:
  // Called to initialize this nodelet
  virtual void Initialize(ros::NodeHandle *nh) {
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.Initialize(GetPrivateHandle(), "mobility/percher.config");
    cfg_.Listen(boost::bind(&Percher::ReconfigureCallback, this, _1));

    // Setup a timer to forward diagnostics
    if (cfg_.Get<double>("diagnostics_rate") > 0)
      timer_d_ = nh->createTimer(ros::Duration(ros::Rate(DEFAULT_DIAGNOSTICS_RATE)),
        &Percher::DiagnosticsCallback, this, false, true);

    /////////////////////////////////////////////////////////////////////////////////////
    // ACTION CLIENTS ///////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////

    // move action
    sac_a_ = std::shared_ptr<actionlib::SimpleActionClient<ff_msgs::ArmAction>>(
               new actionlib::SimpleActionClient<ff_msgs::ArmAction>(
                  *nh, ACTION_HARDWARE_ARM, true));
    if (!sac_a_->waitForServer(ros::Duration(DEFAULT_ACTION_WAIT_TIME)))
      FF_ERROR("Failed to find the ARM action within the specified timeout");

    // move action
    sac_m_ = std::shared_ptr<actionlib::SimpleActionClient<ff_msgs::MoveAction>>(
               new actionlib::SimpleActionClient<ff_msgs::MoveAction>(
                  *nh, ACTION_MOBILITY_MOVE, true));
    if (!sac_m_->waitForServer(ros::Duration(DEFAULT_ACTION_WAIT_TIME)))
      FF_ERROR("Failed to find the MOVE action within the specified timeout");

    // switch action
    sac_s_ = std::shared_ptr<actionlib::SimpleActionClient<ff_msgs::SwitchAction>>(
               new actionlib::SimpleActionClient<ff_msgs::SwitchAction>(
                  *nh, ACTION_LOCALIZATION_MANAGER_SWITCH, true));
    if (!sac_s_->waitForServer(ros::Duration(DEFAULT_ACTION_WAIT_TIME)))
      FF_ERROR("Failed to find the SWITCH action within specified timeout");

    /////////////////////////////////////////////////////////////////////////////////////
    // ACTION SERVERS ///////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////

    // Start perch action
    sas_p_ = std::shared_ptr<actionlib::SimpleActionServer<ff_msgs::PerchAction> >(
               new actionlib::SimpleActionServer<ff_msgs::PerchAction>(*nh, ACTION_MOBILITY_PERCH, false));
    sas_p_->registerGoalCallback(boost::bind(&Percher::PerchGoalCallback, this));
    sas_p_->registerPreemptCallback(boost::bind(&Percher::PreemptCallback, this));
    sas_p_->start();

    // Start unperch action
    sas_u_ = std::shared_ptr<actionlib::SimpleActionServer<ff_msgs::UnperchAction> >(
               new actionlib::SimpleActionServer<ff_msgs::UnperchAction>(*nh, ACTION_MOBILITY_UNPERCH, false));
    sas_u_->registerGoalCallback(boost::bind(&Percher::UnperchGoalCallback, this));
    sas_u_->registerPreemptCallback(boost::bind(&Percher::PreemptCallback, this));
    sas_u_->start();

    // When the system boots we don't really know if we are perched or not. The idea being
    // that the astronaut might have manually perched the device then turned it on.
  }

  // The state machine uses an event to move the state forward
  void StateMachine(PercherEvent event) {
    /////////////////////////////////////////////////////////////////////////////////////
    // NOMINAL BEHAVIOUR ////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////
    switch (state_) {
    // Percher is doing nothing /////////////////////////////////////////////////////////
    case IDLE:
      if (event == START_PERCH) {
        FF_INFO("- Switching to handrail localization");
        state_ = PERCHING_SWITCHING_TO_HANDRAIL_LOC;
        return Switch(MODE_LOCALIZATION_HANDRAIL);
      }
      break;
    // Switching to handrail localization ///////////////////////////////////////////////
    case PERCHING_SWITCHING_TO_HANDRAIL_LOC:
      if (event == SUCCESS) {
        FF_INFO("- Moving to approach point");
        state_ = PERCHING_MOVING_TO_APPROACH_POINT;
        return Move("nominal", APPROACH_POSE, SUCCESS_DESIRED);
      }
      break;
    // Moving to the initial approach point /////////////////////////////////////////////
    case PERCHING_MOVING_TO_APPROACH_POINT:
      if (event == SUCCESS) {
        FF_INFO("- Deploying arm");
        state_ = PERCHING_DEPLOYING_ARM;
        return Arm(ff_msgs::ArmGoal::ACTION_ARM_DEPLOY);
      }
      break;
    // Deploying the perching arm ///////////////////////////////////////////////////////
    case PERCHING_DEPLOYING_ARM:
      if (event == SUCCESS) {
        FF_INFO("- Opening gripper");
        state_ = PERCHING_OPENING_GRIPPER;
        return Arm(ff_msgs::ArmGoal::ACTION_GRIPPER_OPEN);
      }
      break;
    // Opening the gripper //////////////////////////////////////////////////////////////
    case PERCHING_OPENING_GRIPPER:
      if (event == SUCCESS) {
        FF_INFO("- Ingressing");
        state_ = PERCHING_INGRESSING;
        return Move("perching", PERCHED_POSE, SUCCESS_DESIRED);
      }
      break;
    // Moving towards the handrail //////////////////////////////////////////////////////
    case PERCHING_INGRESSING:
      if (event == SUCCESS) {
        FF_INFO("- Closing gripper");
        state_ = PERCHING_CLOSING_GRIPPER;
        return Arm(ff_msgs::ArmGoal::ACTION_GRIPPER_CLOSE);
      }
      break;
    // Closing the gripper //////////////////////////////////////////////////////////////
    case PERCHING_CLOSING_GRIPPER:
      if (event == SUCCESS) {
        FF_INFO("- Checking attachment");
        state_ = PERCHING_CHECKING_ATTACHMENT;
        return Move("perching", APPROACH_POSE, FAILURE_DESIRED);
      }
      break;
    // Checking whether we are attached /////////////////////////////////////////////////
    case PERCHING_CHECKING_ATTACHMENT:
      if (event == SUCCESS) {
        FF_INFO("- Turning off localization");
        state_ = PERCHING_SWITCHING_TO_NONE_LOC;
        return Switch(MODE_LOCALIZATION_NONE);
      }
      break;
    // Switching to no localization / control off ///////////////////////////////////////
    case PERCHING_SWITCHING_TO_NONE_LOC:
      if (event == SUCCESS) {
        FF_INFO("- Resetting pan and tilt");
        state_ = PERCHING_RESETTING_PAN_TILT_POSE;
        return Arm(ff_msgs::ArmGoal::ACTION_ARM_PAN_TILT, 0.0f, 0.0f);
      }
      break;
    // Resetting pan tilt pose //////////////////////////////////////////////////////////
    case PERCHING_RESETTING_PAN_TILT_POSE:
      if (event == SUCCESS) {
        FF_INFO("Perch successful");
        state_ = PERCHED;
        return;
      }
      break;
    // We are perched! //////////////////////////////////////////////////////////////////
    case PERCHED:
      if (event == START_UNPERCH) {
        FF_INFO("- Resetting pan and tilt");
        state_ = UNPERCHING_RESETTING_PAN_TILT_POSE;
        return Arm(ff_msgs::ArmGoal::ACTION_ARM_PAN_TILT, 0.0f, 0.0f);
      }
      break;
    // Resetting pan tilt pose //////////////////////////////////////////////////////////
    case UNPERCHING_RESETTING_PAN_TILT_POSE:
      if (event == SUCCESS) {
        FF_INFO("- Switching to handrail localization");
        state_ = UNPERCHING_SWITCHING_TO_HANDRAIL_LOC;
        return Switch(MODE_LOCALIZATION_HANDRAIL);
      }
      break;
    // Switching to perch localization //////////////////////////////////////////////////
    case UNPERCHING_SWITCHING_TO_HANDRAIL_LOC:
      if (event == SUCCESS) {
        FF_INFO("- Checking attachment");
        state_ = UNPERCHING_CHECKING_ATTACHMENT;
        return Move("perching", APPROACH_POSE, FAILURE_DESIRED);
      }
      break;
    // Checking we are attached /////////////////////////////////////////////////////////
    case UNPERCHING_CHECKING_ATTACHMENT:
      if (event == SUCCESS) {
        FF_INFO("- Opening gripper");
        state_ = UNPERCHING_OPENING_GRIPPER;
        return Arm(ff_msgs::ArmGoal::ACTION_GRIPPER_OPEN);
      }
      break;
    // Opening the gripper //////////////////////////////////////////////////////////////
    case UNPERCHING_OPENING_GRIPPER:
      if (event == SUCCESS) {
        FF_INFO("- Egressing");
        state_ = UNPERCHING_EGRESSING;
        return  Move("perching", APPROACH_POSE, SUCCESS_DESIRED);
      }
      break;
    // Egressing from the handrail //////////////////////////////////////////////////////
    case UNPERCHING_EGRESSING:
      if (event == SUCCESS) {
        FF_INFO("- Closing gripper");
        state_ = UNPERCHING_CLOSING_GRIPPER;
        return Arm(ff_msgs::ArmGoal::ACTION_GRIPPER_CLOSE);
      }
      break;
    // Closing the gripper //////////////////////////////////////////////////////////////
    case UNPERCHING_CLOSING_GRIPPER:
      if (event == SUCCESS) {
        FF_INFO("- Stowing arm");
        state_ = UNPERCHING_STOWING_ARM;
        return Arm(ff_msgs::ArmGoal::ACTION_ARM_STOW);
      }
      break;
    // Retracting the perching arm //////////////////////////////////////////////////////
    case UNPERCHING_STOWING_ARM:
      if (event == SUCCESS) {
        FF_INFO("- Switching back to mapped landmark localization");
        state_ = UNPERCHING_SWITCHING_TO_MAPPED_LANDMARKS_LOC;
        return Switch(MODE_LOCALIZATION_MAPPED_LANDMARKS);
      }
      break;
    // Switching back to sparse mapping /////////////////////////////////////////////////
    case UNPERCHING_SWITCHING_TO_MAPPED_LANDMARKS_LOC:
      if (event == SUCCESS) {
        FF_INFO("- Unperch successful");
        state_ = IDLE;
        return;
      }
      break;
    default:
      FF_ERROR("NOMINAL: Unknown event (developer error)" << event);
      break;
    }
    /////////////////////////////////////////////////////////////////////////////////////
    // OFF-NOMINAL BEHAVIOUR ////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////
    // If we get to this point, it means that we did not transition as per planned. We //
    // need to work out what to do based on what was asked and our current state...    //
    /////////////////////////////////////////////////////////////////////////////////////
    FF_ERROR("Could not complete desired request");
  }

  // Helper function for localization switching

  void Switch(std::string const& localization_mode) {
    ff_msgs::SwitchGoal goal;
    goal.mode = localization_mode;
    sac_s_->sendGoal(goal, boost::bind(&Percher::SwitchResultCallback, this, _1, _2));
  }

  void SwitchResultCallback(actionlib::SimpleClientGoalState const& state,
    ff_msgs::SwitchResultConstPtr const& result) {
    if (result->response > 0)
      return StateMachine(SUCCESS);
    return StateMachine(PROBLEM);
  }

  // Helper function for moving

  void Move(std::string const& flight_mode, PercherPose perch_pose, PercherMoveResult type) {
    // In order to work out the perch pose, we first need to know the pose of the
    // handrail in space. This is not a static value, but is solved for as part of
    // handrail localization. TODO(Get this from localization)
    geometry_msgs::PoseStamped pose;
    switch (perch_pose) {
    case APPROACH_POSE:
      /* Do something */
      break;
    case PERCHED_POSE:
      /* Do something */
      break;
    }
    // Configure the choreographer
    ff_util::ConfigClient cfg_choreographer(GetPlatformHandle(), NODE_CHOREOGRAPHER);
    cfg_choreographer.Set<bool>("enable_control", true);            // Actually move!
    cfg_choreographer.Set<bool>("enable_move_to_start", true);      // Allow moving to start
    cfg_choreographer.Set<bool>("enable_immediate", true);          // Ignore timestamps
    if (!cfg_choreographer.Reconfigure())
      return StateMachine(PROBLEM);
    // Configure the planner
    ff_util::ConfigClient cfg_planner(GetPlatformHandle(), NODE_PLANNER);
    cfg_planner.Set<double>("limits_lin_vel", 0);                   // No constraint
    cfg_planner.Set<double>("limits_lin_acc", 0);                   // No constraint
    cfg_planner.Set<double>("limits_ang_vel", 0);                   // No constraint
    cfg_planner.Set<double>("limits_ang_acc", 0);                   // No constraint
    cfg_planner.Set<bool>("enable_faceforward", false);             // Holonomic mode
    cfg_planner.Set<bool>("enable_obstacles", false);               // Ignore obstacles
    cfg_planner.Set<bool>("enable_keepouts", true);                 // Respect keepouts
    cfg_planner.Reconfigure();
    if (!cfg_planner.Reconfigure())
      return StateMachine(PROBLEM);
    // Send the goal
    ff_msgs::MoveGoal goal;
    goal.flight_mode = flight_mode;
    goal.states.push_back(pose);
    switch (type) {
    case SUCCESS_DESIRED:
      sac_m_->sendGoal(goal, boost::bind(&Percher::MoveSuccessResultCallback, this, _1, _2));
      break;
    case FAILURE_DESIRED:
      sac_m_->sendGoal(goal, boost::bind(&Percher::MoveFailureResultCallback, this, _1, _2));
      break;
    }
  }

  void MoveSuccessResultCallback(actionlib::SimpleClientGoalState const& state,
    ff_msgs::MoveResultConstPtr const& result) {
    if (result->result.response == ff_msgs::MobilityResult::SUCCESS)
      return StateMachine(SUCCESS);
    return StateMachine(PROBLEM);
  }

  void MoveFailureResultCallback(actionlib::SimpleClientGoalState const& state,
    ff_msgs::MoveResultConstPtr const& result) {
    if (result->result.response == ff_msgs::MobilityResult::ERROR_CONTROL && (
             result->result.error == ff_msgs::ControlResult::TOLERANCE_VIOLATION_POSITION
          || result->result.error == ff_msgs::ControlResult::TOLERANCE_VIOLATION_VELOCITY
          || result->result.error == ff_msgs::ControlResult::TOLERANCE_VIOLATION_ATTITUDE
          || result->result.error == ff_msgs::ControlResult::TOLERANCE_VIOLATION_OMEGA))
      return StateMachine(SUCCESS);
    return StateMachine(PROBLEM);
  }

  // Helper function for interacting with the perching arm

  void Arm(uint8_t action, float pan = 0, float tilt = 0) {
    ff_msgs::ArmGoal goal;
    goal.action = action;
    goal.pan    = pan;
    goal.tilt   = tilt;
    sac_a_->sendGoal(goal, boost::bind(&Percher::ArmResultCallback, this, _1, _2));
  }

  void ArmResultCallback(actionlib::SimpleClientGoalState const& state,
    ff_msgs::ArmResultConstPtr const& result) {
    if (result->response == ff_msgs::ArmResult::SUCCESS)
      return StateMachine(SUCCESS);
    return StateMachine(PROBLEM);
  }

 private:
  // A new stop action has been called
  void PerchGoalCallback() {
    PreemptCallback();  // Force preemption of all other actions
    FF_INFO("A new PERCH command was just received");
    sas_p_->acceptNewGoal();
    if (sas_p_->isPreemptRequested())
      return PreemptCallback();
    // Switch state
  }

  // A new idle action has been called
  void UnperchGoalCallback() {
    PreemptCallback();  // Force preemption of all other actions
    FF_INFO("A new UNPERCH command was just received");
    sas_u_->acceptNewGoal();
    if (sas_u_->isPreemptRequested())
      return PreemptCallback();
    // Switch state
  }

  // Stop has been preempted
  void PreemptCallback() {
    if (sas_p_->isNewGoalAvailable())
      return StateMachine(PREEMPT);
    if (sas_u_->isNewGoalAvailable())
      return StateMachine(PREEMPT);
    return StateMachine(CANCEL);
  }

  // When a new reconfigure request comes in
  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    if (state_ != IDLE)
      return false;
    cfg_.Reconfigure(config);
    return true;
  }

  // When diagnostics must be sent
  void DiagnosticsCallback(const ros::TimerEvent & event) {
    SendDiagnostics(cfg_.Dump());
  }

 protected:
  PercherState state_;
  ff_util::ConfigServer cfg_;
  ros::Timer timer_d_;
  // Action servers
  std::shared_ptr<actionlib::SimpleActionServer<ff_msgs::PerchAction> > sas_p_;
  std::shared_ptr<actionlib::SimpleActionServer<ff_msgs::UnperchAction> > sas_u_;
  // Action clients and activation flags
  std::shared_ptr<actionlib::SimpleActionClient<ff_msgs::ArmAction>> sac_a_;
  std::shared_ptr<actionlib::SimpleActionClient<ff_msgs::MoveAction>> sac_m_;
  std::shared_ptr<actionlib::SimpleActionClient<ff_msgs::SwitchAction>> sac_s_;
};

PLUGINLIB_DECLARE_CLASS(percher, Percher,
                        percher::Percher, nodelet::Nodelet);

}  // namespace percher
