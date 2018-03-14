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
#include <ff_util/ff_fsm.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>

// Hardware messages
#include <ff_hw_msgs/EpsDockStateStamped.h>

// Software messages
#include <ff_msgs/DockState.h>

// Services
#include <ff_hw_msgs/Undock.h>
#include <ff_msgs/SetState.h>

// Actions
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/SwitchAction.h>
#include <ff_msgs/DockAction.h>

/**
 * \ingroup beh
 */
namespace dock {

// Match the internal states and responses with the message definition
using FSM = ff_util::FSM;
using STATE = ff_msgs::DockState;
using RESPONSE = ff_msgs::DockResult;

/*
  This class provides the high-level logic that allows the freeflyer to
  perform both docking and undocking from its dock. Please see the accom-
  panying state diagram for a high level overview of this logic. Note that
  there is an auto-recovery mechanism to avoid a motion tolerance error from
  causing the freeflyer to drift uncontrollably into / away from the dock.

  Here are the rules of auto-recovery in the context of docking:

    * If the callee cancels the task the system stops immediately. Nothing
      further is carried out, under the key assumption that a cancellation
      implies intervention due to something having gone quite wrong.

    * If something during the docking or undocking procedure goes wrong,
      the server aborts the task, then the system moved to an auto-recovery
      sequence. The callee is only notified of the error AFTER the auto-
      recovery completes, to avoid dock and executive contention.

    * If a dock or undock procedure is preempted, the system must abort
      the current task immediately, so that the new goal can be accepted,
      regardless of the progress (or lack thereof). The end state will
      either be docked (if still attached) or undocked (if in flight).
*/
class DockNodelet : public ff_util::FreeFlyerNodelet {
 public:
  // All possible events that can occur
  enum : FSM::Event {
    READY           = (1<<0),     // System is initialized
    GOAL_DOCK       = (1<<1),     // Start docking
    GOAL_UNDOCK     = (1<<2),     // Stop undocking
    GOAL_CANCEL     = (1<<3),     // Cancel an existing goal
    GOAL_PREEMPT    = (1<<4),     // Cancel an existing goal
    EPS_DOCKED      = (1<<5),     // State is currently docked
    EPS_UNDOCKED    = (1<<6),     // State is currently undocked
    EPS_TIMEOUT     = (1<<7),     // EPS timeout
    SWITCH_SUCCESS  = (1<<8),     // Localization manager switch result
    SWITCH_FAILED   = (1<<9),     // Localization manager switch failed
    MOTION_SUCCESS  = (1<<10),    // Mobility motion action success
    MOTION_FAILED   = (1<<11)     // Mobility motion action problem
  };

  // Positions that we might need to move to
  enum DockPose {
    APPROACH_POSE,      // The starting poses of docking
    COMPLETE_POSE       // The completion pose of docking
  };

  // Constructor boostraps freeflyer nodelet and sets initial FSM state
  DockNodelet() : ff_util::FreeFlyerNodelet(NODE_DOCK, true),
    fsm_(STATE::INITIALIZING, std::bind(&DockNodelet::UpdateCallback,
      this, std::placeholders::_1, std::placeholders::_2)) {
    // Add the berths -> frame associations
    berths_[ff_msgs::DockGoal::BERTH_1] = FRAME_NAME_DOCK_BERTH_1;
    berths_[ff_msgs::DockGoal::BERTH_2] = FRAME_NAME_DOCK_BERTH_2;
    // Add the state transition lambda functions - refer to the FSM diagram
    // [0]
    fsm_.Add(STATE::INITIALIZING,
      READY, [this](FSM::Event const& event) -> FSM::State {
        return STATE::UNKNOWN;
      });
    // [1]
    fsm_.Add(STATE::UNKNOWN,
      EPS_UNDOCKED, [this](FSM::Event const& event) -> FSM::State {
        return STATE::UNDOCKED;
      });
    // [2]
    fsm_.Add(STATE::UNKNOWN,
      EPS_DOCKED, [this](FSM::Event const& event) -> FSM::State {
        return STATE::DOCKED;
      });
    // [3]
    fsm_.Add(STATE::UNDOCKED,
      GOAL_DOCK, [this](FSM::Event const& event) -> FSM::State {
        Switch(LOCALIZATION_MAPPED_LANDMARKS);
        return STATE::DOCKING_SWITCHING_TO_ML_LOC;
      });
    // [4]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_ML_LOC,
      SWITCH_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Move(APPROACH_POSE, ff_msgs::MotionGoal::NOMINAL);
        return STATE::DOCKING_MOVING_TO_APPROACH_POSE;
      });
    // [5]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_ML_LOC,
      SWITCH_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::SWITCH_TO_ML_FAILED);
        return STATE::UNDOCKED;
      });
    // [6]
    fsm_.Add(STATE::DOCKING_MOVING_TO_APPROACH_POSE,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Switch(LOCALIZATION_AR_TAGS);
        return STATE::DOCKING_SWITCHING_TO_AR_LOC;
      });
    // [7]
    fsm_.Add(STATE::DOCKING_MOVING_TO_APPROACH_POSE,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::MOTION_APPROACH_FAILED;
        Switch(LOCALIZATION_MAPPED_LANDMARKS);
        return STATE::RECOVERY_SWITCHING_TO_ML_LOC;
      });
    // [31]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_AR_LOC,
      SWITCH_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Move(COMPLETE_POSE, ff_msgs::MotionGoal::DOCKING);
        return STATE::DOCKING_MOVING_TO_COMPLETE_POSE;
      });
    // [32]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_AR_LOC, SWITCH_FAILED,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::SWITCH_TO_AR_FAILED);
        return STATE::UNDOCKED;
      });
    // [8]
    fsm_.Add(STATE::DOCKING_MOVING_TO_COMPLETE_POSE,
      EPS_DOCKED, [this](FSM::Event const& event) -> FSM::State {
        Move(APPROACH_POSE, ff_msgs::MotionGoal::DOCKING);
        return STATE::DOCKING_CHECKING_ATTACHED;
      });
    // [9]
    fsm_.Add(STATE::DOCKING_MOVING_TO_COMPLETE_POSE,
      MOTION_SUCCESS | MOTION_FAILED,
      [this](FSM::Event const& event) -> FSM::State {
        if (event == MOTION_SUCCESS)
          err_ = RESPONSE::EPS_DOCK_FAILED;
        if (event == MOTION_FAILED)
          err_ = RESPONSE::MOTION_COMPLETE_FAILED;
        Move(APPROACH_POSE, ff_msgs::MotionGoal::DOCKING);
        return STATE::RECOVERY_MOVING_TO_APPROACH_POSE;
      });
    // [10]
    fsm_.Add(STATE::DOCKING_CHECKING_ATTACHED,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Prep(ff_msgs::MotionGoal::OFF);
        return STATE::DOCKING_WAITING_FOR_SPIN_DOWN;
      });
    // [11]
    fsm_.Add(STATE::DOCKING_CHECKING_ATTACHED,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::MOTION_ATTACHED_FAILED;
        Move(APPROACH_POSE, ff_msgs::MotionGoal::DOCKING);
        return STATE::RECOVERY_MOVING_TO_APPROACH_POSE;
      });
    // [12]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_NO_LOC,
      SWITCH_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::DOCKED);
        return STATE::DOCKED;
      });
    // [13]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_NO_LOC,
      SWITCH_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::SWITCH_TO_NO_FAILED);
        return STATE::DOCKED;
      });
    // [14]
    fsm_.Add(STATE::DOCKED,
      GOAL_UNDOCK, [this](FSM::Event const& event) -> FSM::State {
        Switch(LOCALIZATION_MAPPED_LANDMARKS);
        return STATE::UNDOCKING_SWITCHING_TO_ML_LOC;
      });
    // [15]
    fsm_.Add(STATE::UNDOCKING_SWITCHING_TO_ML_LOC,
      SWITCH_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        Prep(ff_msgs::MotionGoal::NOMINAL);
        return STATE::UNDOCKING_WAITING_FOR_SPIN_UP;
      });
    // [16]
    fsm_.Add(STATE::UNDOCKING_SWITCHING_TO_ML_LOC,
      SWITCH_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::SWITCH_TO_ML_FAILED;
        Switch(LOCALIZATION_NONE);
        return STATE::RECOVERY_SWITCHING_TO_NO_LOC;
      });
    // [19]
    fsm_.Add(STATE::UNDOCKING_MOVING_TO_APPROACH_POSE,
      MOTION_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::UNDOCKED);
        return STATE::UNDOCKED;
      });
    // [20]
    fsm_.Add(STATE::UNDOCKING_MOVING_TO_APPROACH_POSE,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::MOTION_APPROACH_FAILED);
        return STATE::UNDOCKED;
      });
    // [21]
    fsm_.Add(STATE::RECOVERY_MOVING_TO_APPROACH_POSE,
      MOTION_SUCCESS | MOTION_FAILED,
      [this](FSM::Event const& event) -> FSM::State {
        Switch(LOCALIZATION_MAPPED_LANDMARKS);
        return STATE::RECOVERY_SWITCHING_TO_ML_LOC;
      });
    // [22]
    fsm_.Add(STATE::RECOVERY_SWITCHING_TO_ML_LOC,
      SWITCH_SUCCESS | SWITCH_FAILED,
      [this](FSM::Event const& event) -> FSM::State {
        Result(err_);
        return STATE::UNDOCKED;
      });
    // [23]
    fsm_.Add(STATE::RECOVERY_SWITCHING_TO_NO_LOC,
      SWITCH_SUCCESS | SWITCH_FAILED,
      [this](FSM::Event const& event) -> FSM::State {
        Result(err_);
        return STATE::DOCKED;
      });
    // [24]
    fsm_.Add(STATE::DOCKED,
      EPS_UNDOCKED, [this](FSM::Event const& event) -> FSM::State {
        return STATE::UNDOCKED;
      });
    // [25]
    fsm_.Add(STATE::UNDOCKED,
      EPS_DOCKED, [this](FSM::Event const& event) -> FSM::State {
        return STATE::DOCKED;
      });
    // [26]
    fsm_.Add(STATE::UNDOCKING_WAITING_FOR_SPIN_UP,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Undock();
        Move(APPROACH_POSE, ff_msgs::MotionGoal::NOMINAL);
        return STATE::UNDOCKING_MOVING_TO_APPROACH_POSE;
      });
    // [27]
    fsm_.Add(STATE::DOCKING_WAITING_FOR_SPIN_DOWN,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Switch(LOCALIZATION_NONE);
        return STATE::DOCKING_SWITCHING_TO_NO_LOC;
      });
    // [28]
    fsm_.Add(STATE::DOCKING_WAITING_FOR_SPIN_DOWN,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::PREP_DISABLE_FAILED);
        Switch(LOCALIZATION_NONE);
        return STATE::DOCKED;
      });
    // [29]
    fsm_.Add(STATE::UNDOCKED,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::PREP_ENABLE_FAILED;
        Prep(ff_msgs::MotionGoal::OFF);
        return STATE::RECOVERY_WAITING_FOR_SPIN_DOWN;
      });
    // [30]
    fsm_.Add(STATE::RECOVERY_WAITING_FOR_SPIN_DOWN,
      MOTION_FAILED | MOTION_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        Switch(LOCALIZATION_NONE);
        return STATE::RECOVERY_SWITCHING_TO_NO_LOC;
      });
    //////////////////////////////////////////////
    // CATCH-ALL FOR CANCELLATIONS / PREEMPTION //
    //////////////////////////////////////////////
    fsm_.Add(GOAL_CANCEL | GOAL_PREEMPT,
      [this](FSM::State const& state, FSM::Event const& event) -> FSM::State {
        switch (state) {
        // Values we certaintly don't want to mess with
        default:
        case STATE::INITIALIZING:
        case STATE::UNKNOWN:
        case STATE::DOCKED:
        case STATE::UNDOCKED:
          break;
        // Undocked and switch in progress
        case STATE::DOCKING_SWITCHING_TO_ML_LOC:
        case STATE::DOCKING_SWITCHING_TO_AR_LOC:
        case STATE::RECOVERY_SWITCHING_TO_ML_LOC:
          client_s_.CancelGoal();
          return STATE::UNDOCKED;
        // Undocked and motion in progress
        case STATE::DOCKING_MOVING_TO_APPROACH_POSE:
        case STATE::DOCKING_MOVING_TO_COMPLETE_POSE:
        case STATE::UNDOCKING_MOVING_TO_APPROACH_POSE:
        case STATE::RECOVERY_MOVING_TO_APPROACH_POSE:
          client_m_.CancelGoal();
          return STATE::UNDOCKED;
        // Docked and switch in progress
        case STATE::DOCKING_SWITCHING_TO_NO_LOC:
        case STATE::UNDOCKING_SWITCHING_TO_ML_LOC:
        case STATE::RECOVERY_SWITCHING_TO_NO_LOC:
          client_s_.CancelGoal();
          return STATE::DOCKED;
        // Docked and motion in progress
        case STATE::UNDOCKING_WAITING_FOR_SPIN_UP:
        case STATE::DOCKING_CHECKING_ATTACHED:
        case STATE::DOCKING_WAITING_FOR_SPIN_DOWN:
          client_m_.CancelGoal();
          return STATE::DOCKED;
        }
        // Send a response stating that we were canceled
        switch (event) {
        case GOAL_CANCEL:
          Result(RESPONSE::CANCELLED);
          break;
        case GOAL_PREEMPT:
          Result(RESPONSE::PREEMPTED);
          break;
        }
        // Default case is to preserve the stata
        return state;
      });
  }
  virtual ~DockNodelet() {}

 protected:
  // Called to initialize this nodelet
  void Initialize(ros::NodeHandle *nh) {
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.Initialize(GetPrivateHandle(), "behaviors/dock.config");
    if (!cfg_.Listen(boost::bind(
      &DockNodelet::ReconfigureCallback, this, _1)))
      return AssertFault("INITIALIZATION_FAULT", "Could not load config");
    // One shot timer to check if we undock with a timeout
    timer_eps_ = nh->createTimer(
      ros::Duration(cfg_.Get<double>("timeout_eps_response")),
      &DockNodelet::DockTimerCallback, this, true, false);

    // Create a transform buffer to listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(tf_buffer_));

    // Publish the docking state as a latched topic
    pub_ = nh->advertise<ff_msgs::DockState>(
      TOPIC_BEHAVIORS_DOCKING_STATE, 1, true);

    // Subscribe to be notified when the dock state changes
    sub_s_ = nh->subscribe(TOPIC_HARDWARE_EPS_DOCK_STATE, 5,
      &DockNodelet::DockStateCallback, this);

    // Allow the state to be manually set
    server_set_state_ = nh->advertiseService(SERVICE_BEHAVIORS_DOCK_SET_STATE,
      &DockNodelet::SetStateCallback, this);

    // Contact EPS service for undocking
    client_u_.SetConnectedTimeout(cfg_.Get<double>("timeout_undock_connected"));
    client_u_.SetConnectedCallback(std::bind(
      &DockNodelet::ConnectedCallback, this));
    client_u_.SetTimeoutCallback(std::bind(
      &DockNodelet::UndockTimeoutCallback, this));
    client_u_.Create(nh, SERVICE_HARDWARE_EPS_UNDOCK);

    // Setup move client action
    client_m_.SetConnectedTimeout(cfg_.Get<double>("timeout_motion_connected"));
    client_m_.SetActiveTimeout(cfg_.Get<double>("timeout_motion_active"));
    client_m_.SetResponseTimeout(cfg_.Get<double>("timeout_motion_response"));
    client_m_.SetFeedbackCallback(std::bind(&DockNodelet::MFeedbackCallback,
      this, std::placeholders::_1));
    client_m_.SetResultCallback(std::bind(&DockNodelet::MResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_m_.SetConnectedCallback(std::bind(
      &DockNodelet::ConnectedCallback, this));
    client_m_.Create(nh, ACTION_MOBILITY_MOTION);

    // Setup switch client action
    client_s_.SetConnectedTimeout(cfg_.Get<double>("timeout_switch_connected"));
    client_s_.SetActiveTimeout(cfg_.Get<double>("timeout_switch_active"));
    client_s_.SetResponseTimeout(cfg_.Get<double>("timeout_switch_response"));
    client_s_.SetDeadlineTimeout(cfg_.Get<double>("timeout_switch_deadline"));
    client_s_.SetFeedbackCallback(std::bind(
      &DockNodelet::SFeedbackCallback, this, std::placeholders::_1));
    client_s_.SetResultCallback(std::bind(&DockNodelet::SResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_s_.SetConnectedCallback(std::bind(&DockNodelet::ConnectedCallback,
      this));
    client_s_.Create(nh, ACTION_LOCALIZATION_MANAGER_SWITCH);

    // Setup the execute action
    server_.SetGoalCallback(std::bind(
      &DockNodelet::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &DockNodelet::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &DockNodelet::CancelCallback, this));
    server_.Create(nh, ACTION_BEHAVIORS_DOCK);
  }

  // Timeout on a trajectory generation request
  void EnableTimeoutCallback(void) {
    return AssertFault("INITIALIZATION_FAULT", "Could not find enable service");
  }

  // Timeout on a trajectory generation request
  void UndockTimeoutCallback(void) {
    return AssertFault("INITIALIZATION_FAULT", "Could not find undock service");
  }

  // Ensure all clients are connected
  void ConnectedCallback() {
    NODELET_DEBUG_STREAM("ConnectedCallback()");
    if (!client_u_.IsConnected()) return;       // Undock service
    if (!client_s_.IsConnected()) return;       // Switch action
    if (!client_m_.IsConnected()) return;       // Move action
    fsm_.Update(READY);                         // Ready!
  }

  // Called on registration of aplanner
  bool SetStateCallback(ff_msgs::SetState::Request& req,
                        ff_msgs::SetState::Response& res) {
    fsm_.SetState(req.state);
    res.success = true;
    return true;
  }

  // Complete the current dock or undock action
  void Result(int32_t response) {
    // Send the feedback if needed
    switch (fsm_.GetState()) {
    case STATE::INITIALIZING:
    case STATE::UNKNOWN:
    case STATE::DOCKED:
    case STATE::UNDOCKED:
      return;
    default:
      break;
    }
    // Package up the feedback
    ff_msgs::DockResult result;
    result.response = response;
    if (response > 0)
      server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    else if (response < 0)
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
    else
      server_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
  }

  // When the FSM state changes we get a callback here, so that we
  // can choose to do various things.
  void UpdateCallback(FSM::State const& state, FSM::Event const& event) {
    // Debug events
    std::string str = "UNKNOWN";
    switch (event) {
    case READY:          str = "READY";          break;
    case GOAL_DOCK:      str = "GOAL_DOCK";      break;
    case GOAL_UNDOCK:    str = "GOAL_UNDOCK";    break;
    case GOAL_CANCEL:    str = "GOAL_CANCEL";    break;
    case EPS_DOCKED:     str = "EPS_DOCKED";     break;
    case EPS_UNDOCKED:   str = "EPS_UNDOCKED";   break;
    case EPS_TIMEOUT:    str = "EPS_TIMEOUT";    break;
    case SWITCH_SUCCESS: str = "SWITCH_SUCCESS"; break;
    case SWITCH_FAILED:  str = "SWITCH_FAILED";  break;
    case MOTION_SUCCESS: str = "MOTION_SUCCESS"; break;
    case MOTION_FAILED:  str = "MOTION_FAILED";  break;
    }
    NODELET_DEBUG_STREAM("Received event " << str);
    // Debug state changes
    switch (state) {
    case STATE::INITIALIZING:
      str = "INITIALIZING";                      break;
    case STATE::UNKNOWN:
      str = "UNKNOWN";                           break;
    case STATE::UNDOCKED:
      str = "UNDOCKED";                          break;
    case STATE::DOCKING_SWITCHING_TO_ML_LOC:
      str = "DOCKING_SWITCHING_TO_ML_LOC";       break;
    case STATE::DOCKING_MOVING_TO_APPROACH_POSE:
      str = "DOCKING_MOVING_TO_APPROACH_POSE";   break;
    case STATE::DOCKING_SWITCHING_TO_AR_LOC:
      str = "DOCKING_SWITCHING_TO_AR_LOC";       break;
    case STATE::DOCKING_MOVING_TO_COMPLETE_POSE:
      str = "DOCKING_MOVING_TO_COMPLETE_POSE";   break;
    case STATE::DOCKING_CHECKING_ATTACHED:
      str = "DOCKING_CHECKING_ATTACHED";         break;
    case STATE::DOCKING_WAITING_FOR_SPIN_DOWN:
      str = "DOCKING_WAITING_FOR_SPIN_DOWN";     break;
    case STATE::DOCKING_SWITCHING_TO_NO_LOC:
      str = "DOCKING_SWITCHING_TO_NO_LOC";       break;
    case STATE::DOCKED:
      str = "DOCKED";                            break;
    case STATE::UNDOCKING_SWITCHING_TO_ML_LOC:
      str = "UNDOCKING_SWITCHING_TO_ML_LOC";     break;
    case STATE::UNDOCKING_WAITING_FOR_SPIN_UP:
      str = "UNDOCKING_WAITING_FOR_SPIN_UP";     break;
    case STATE::UNDOCKING_MOVING_TO_APPROACH_POSE:
      str = "UNDOCKING_MOVING_TO_APPROACH_POSE"; break;
    case STATE::RECOVERY_SWITCHING_TO_NO_LOC:
      str = "RECOVERY_SWITCHING_TO_NO_LOC";         break;
    case STATE::RECOVERY_MOVING_TO_APPROACH_POSE:
      str = "RECOVERY_MOVING_TO_APPROACH_POSE";  break;
    case STATE::RECOVERY_SWITCHING_TO_ML_LOC:
      str = "RECOVERY_SWITCHING_TO_ML_LOC";         break;
    }
    NODELET_DEBUG_STREAM("State changed to " << str);
    // Send the feedback if needed
    switch (state) {
    case STATE::INITIALIZING:
    case STATE::UNKNOWN:
    case STATE::DOCKED:
    case STATE::UNDOCKED:
      break;
    default:
      {
        ff_msgs::DockFeedback feedback;
        feedback.state.state = state;
        server_.SendFeedback(feedback);
      }
    }
    // Broadcast the docking state
    ff_msgs::DockState msg;
    msg.header.frame_id = GetPlatform();
    msg.header.stamp = ros::Time::now();
    msg.state = state;
    pub_.publish(msg);
  }

  // Check that we are close enough to the approach pose
  bool CloseEnoughToApproach(uint32_t berth) {
    try {
      // Look up the body frame in the berth frame
      geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
        berths_[berth] + "/approach", FRAME_NAME_BODY, ros::Time(0));
      // Copy the transform
      double d = tf.transform.translation.x * tf.transform.translation.x
               + tf.transform.translation.y * tf.transform.translation.y
               + tf.transform.translation.z * tf.transform.translation.z;
      // We need to be within the initial tolerance to start docking
      if (sqrt(d) > cfg_.Get<double>("initial_tolerance"))
        return false;
    } catch (tf2::TransformException &ex) {
      return false;
    }
    return true;
  }

  // EPS (DOCKING)

  bool Undock() {
    // Any error finding the berth transform or calling the service will result
    // in an EPS_TIMEOUT event, which the FSM will use to recover.
    timer_eps_.start();
    // Look for the berth
    std::map<uint8_t, std::string>::iterator it;
    for (it = berths_.begin(); it != berths_.end(); it++) {
      try {
        // Look up the body frame in the berth frame
        geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
          it->second, FRAME_NAME_BODY, ros::Time(0));
        // Copy the transform
        double d = tf.transform.translation.x * tf.transform.translation.x
                 + tf.transform.translation.y * tf.transform.translation.y
                 + tf.transform.translation.z * tf.transform.translation.z;
        // If we are within some delta of the origin, then we are at this berth
        if (sqrt(d) < cfg_.Get<double>("detection_tolerance"))
          break;
      } catch (tf2::TransformException &ex) {}
    }
    // Let the use know what's happening
    if (it == berths_.end()) {
      NODELET_ERROR_STREAM("Could not detect berth from current pose");
      return false;
    } else {
      NODELET_DEBUG_STREAM("Berth frame detected: " << it->second);
    }
    // At this point we should have good AR or ML localization, so we can
    // determine our pose to within a couple centimeters.
    frame_ = it->second;
    if (!GetPlatform().empty())
      frame_ = GetPlatform() + std::string("/") + frame_;
    // Call the undock service
    ff_hw_msgs::Undock msg;
    return client_u_.Call(msg);
  }

  void DockStateCallback(ff_hw_msgs::EpsDockStateStamped::ConstPtr const& msg) {
    switch (msg->state) {
    // We don't worry about a timeout on docking, because we'll get a motion
    // failure if dockign doesn't succeed.
    case ff_hw_msgs::EpsDockStateStamped::CONNECTING:
    case ff_hw_msgs::EpsDockStateStamped::DOCKED:
      return fsm_.Update(EPS_DOCKED);
    // If the dock state is reported as undocked
    case ff_hw_msgs::EpsDockStateStamped::UNDOCKED:
      timer_eps_.stop();
      return fsm_.Update(EPS_UNDOCKED);
    default:
      break;
    }
  }

  void DockTimerCallback(const ros::TimerEvent&) {
    return fsm_.Update(EPS_TIMEOUT);
  }

  // SWITCH action

  // Helper function for localization switching
  bool Switch(std::string const& pipeline) {
    // Send the switch goal
    ff_msgs::SwitchGoal goal;
    goal.pipeline = pipeline;
    return client_s_.SendGoal(goal);
  }

  // Ignore the switch feedback for now
  void SFeedbackCallback(ff_msgs::SwitchFeedbackConstPtr const& feedback) {}

  // Do something with the switch result
  void SResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::SwitchResultConstPtr const& result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return fsm_.Update(SWITCH_SUCCESS);
    default:
      return fsm_.Update(SWITCH_FAILED);
    }
  }

  // MOTION ACTION CLIENT

  // Prepare for a motion
  bool Prep(std::string const& flight_mode) {
    static ff_msgs::MotionGoal goal;
    goal.command = ff_msgs::MotionGoal::PREP;
    goal.flight_mode = flight_mode;
    return client_m_.SendGoal(goal);
  }

  // Send a move command
  bool Move(DockPose type, std::string const& mode) {
    // Package up the desired end pose
    static geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    switch (type) {
    case COMPLETE_POSE: msg.header.frame_id = frame_ + "/complete"; break;
    case APPROACH_POSE: msg.header.frame_id = frame_ + "/approach"; break;
    default:
      return false;
    }

    // Get the dock -> world transform
    try {
      // Look up the world -> berth transform
      geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
        "world", msg.header.frame_id, ros::Time(0));
      // Copy the transform
      msg.pose.position.x = tf.transform.translation.x;
      msg.pose.position.y = tf.transform.translation.y;
      msg.pose.position.z = tf.transform.translation.z;
      msg.pose.orientation = tf.transform.rotation;
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
    cfg.Set<bool>("enable_timesync", false);
    cfg.Set<bool>("enable_faceforward", false);
    cfg.Set<double>("desired_vel", -1.0);
    cfg.Set<double>("desired_accel", -1.0);
    cfg.Set<double>("desired_omega", -1.0);
    cfg.Set<double>("desired_alpha", -1.0);
    cfg.Set<std::string>("planner", "trapezoidal");
    if (!cfg.Reconfigure())
      return false;

    // Send the goal to the mobility subsystem
    ff_msgs::MotionGoal goal;
    goal.command = ff_msgs::MotionGoal::MOVE;
    goal.flight_mode = mode;
    goal.states.push_back(msg);
    return client_m_.SendGoal(goal);
  }

  // Ignore the move feedback, for now
  void MFeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback) {}

  // Result of a move action
  void MResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::MotionResultConstPtr const& result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return fsm_.Update(MOTION_SUCCESS);
    default:
      return fsm_.Update(MOTION_FAILED);
    }
  }

  // DOCK ACTION SERVER

  // A new arm action has been called
  void GoalCallback(ff_msgs::DockGoalConstPtr const& goal) {
    ff_msgs::DockResult result;
    switch (goal->command) {
    case ff_msgs::DockGoal::DOCK:
      // We are undocked
      if (fsm_.GetState() == STATE::UNDOCKED) {
        // Do we know about the specified berth?
        if (berths_.find(goal->berth) == berths_.end()) {
          result.response = RESPONSE::INVALID_BERTH;
          server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
          return;
        }
        // Check that we are close enough to the approach pose
        if (!CloseEnoughToApproach(goal->berth)) {
          result.response = RESPONSE::TOO_FAR_AWAY_FROM_APPROACH;
          server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
          return;
        }
        // If we do know about it, add a frame id prefix
        frame_ = berths_[goal->berth];
        if (!GetPlatform().empty())
          frame_ = GetPlatform() + std::string("/") + frame_;
        // Start docking
        return fsm_.Update(GOAL_DOCK);
      // We are already docked
      } else if (fsm_.GetState() == STATE::DOCKED) {
        result.response = RESPONSE::ALREADY_DOCKED;
        server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
        return;
      // We are not in  a position to dock
      } else {
        result.response = RESPONSE::NOT_IN_UNDOCKED_STATE;
        server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
        return;
      }
      break;
    // Undock command
    case ff_msgs::DockGoal::UNDOCK:
      // We are docked
      if (fsm_.GetState() == STATE::DOCKED) {
        return fsm_.Update(GOAL_UNDOCK);
      // We are already undocked
      } else if (fsm_.GetState() == STATE::UNDOCKED) {
        result.response = RESPONSE::ALREADY_UNDOCKED;
        server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
        return;
      // We are not in a position to undock
      } else {
        result.response = RESPONSE::NOT_IN_DOCKED_STATE;
      }
      break;
    // Invalid command
    default:
      result.response = RESPONSE::INVALID_COMMAND;
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
      break;
    }
  }

  // Preempt the current action with a new action
  void PreemptCallback() {
    return fsm_.Update(GOAL_PREEMPT);
  }

  // A Cancellation request arrives
  void CancelCallback() {
    return fsm_.Update(GOAL_CANCEL);
  }

  // RECONFIGURE REQUESTS

  // When a new reconfigure request comes in, deal with that request
  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    if ( fsm_.GetState() == STATE::UNDOCKED
      || fsm_.GetState() == STATE::DOCKED)
      return cfg_.Reconfigure(config);
    return false;
  }

 protected:
  std::map<uint8_t, std::string> berths_;
  ff_util::FSM fsm_;
  ff_util::FreeFlyerActionClient<ff_msgs::MotionAction> client_m_;
  ff_util::FreeFlyerActionClient<ff_msgs::SwitchAction> client_s_;
  ff_util::FreeFlyerServiceClient<ff_hw_msgs::Undock> client_u_;
  ff_util::FreeFlyerActionServer<ff_msgs::DockAction> server_;
  ff_util::ConfigServer cfg_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  ros::Publisher pub_;
  ros::Subscriber sub_s_;
  ros::Subscriber sub_p_;
  ros::ServiceServer server_set_state_;
  ros::Timer timer_eps_;
  ros::Timer timer_pmc_;
  std::string frame_;
  int32_t err_;
};

PLUGINLIB_DECLARE_CLASS(dock, DockNodelet,
                        dock::DockNodelet, nodelet::Nodelet);

}  // namespace dock
