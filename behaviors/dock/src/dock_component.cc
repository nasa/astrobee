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
#include <rclcpp/rclcpp.hpp>

// TF2 support
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Shared project includes
#include <msg_conversions/msg_conversions.h>
#include <ff_util/ff_component.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_service.h>
#include <ff_util/ff_fsm.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>

// FSW actions, services, messages
#include <ff_hw_msgs/msg/eps_dock_state_stamped.hpp>
#include <ff_hw_msgs/srv/undock.hpp>
namespace ff_hw_msgs {
  typedef msg::EpsDockStateStamped EpsDockStateStamped;
  typedef srv::Undock Undock;
}  // namespace ff_hw_msgs

#include <ff_msgs/msg/dock_state.hpp>
#include <ff_msgs/srv/set_state.hpp>
#include <ff_msgs/srv/get_pipelines.hpp>
#include <ff_msgs/action/motion.hpp>
#include <ff_msgs/action/localization.hpp>
#include <ff_msgs/action/dock.hpp>
namespace ff_msgs {
  typedef msg::DockState DockState;
  typedef srv::SetState SetState;
  typedef srv::GetPipelines GetPipelines;
  typedef action::Motion Motion;
  typedef action::Localization Localization;
  typedef action::Dock Dock;
}  // namespace ff_msgs

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
namespace geometry_msgs {
  typedef msg::PoseStamped PoseStamped;
  typedef msg::TransformStamped TransformStamped;
}


/**
 * \ingroup beh
 */
namespace dock {

FF_DEFINE_LOGGER("dock");

// Match the internal states and responses with the message definition
using FSM = ff_util::FSM;
using STATE = ff_msgs::DockState;
using RESPONSE = ff_msgs::Dock::Result;

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
class DockComponent : public ff_util::FreeFlyerComponent {
 public:
  // All possible events that can occur
  enum : FSM::Event {
    READY            = (1<<0),     // System is initialized
    GOAL_DOCK        = (1<<1),     // Start docking
    GOAL_UNDOCK      = (1<<2),     // Stop undocking
    GOAL_CANCEL      = (1<<3),     // Cancel an existing goal
    GOAL_PREEMPT     = (1<<4),     // Cancel an existing goal
    EPS_DOCKED       = (1<<5),     // State is currently docked
    EPS_UNDOCKED     = (1<<6),     // State is currently undocked
    EPS_TIMEOUT      = (1<<7),     // EPS timeout
    SWITCH_SUCCESS   = (1<<8),     // Localization manager switch result
    SWITCH_FAILED    = (1<<9),     // Localization manager switch failed
    MOTION_SUCCESS   = (1<<10),    // Mobility motion action success
    MOTION_FAILED    = (1<<11),    // Mobility motion action problem
    MANUAL_STATE_SET = (1<<12)     // Setting the state manually with service
  };

  // Positions that we might need to move to
  enum DockPose {
    APPROACH_POSE,      // The starting poses of docking
    BERTHING_POSE       // The completion pose of docking
  };

  // Constructor boostraps freeflyer nodelet and sets initial FSM state
  explicit DockComponent(const rclcpp::NodeOptions& options) : ff_util::FreeFlyerComponent(options, NODE_DOCK, true),
    fsm_(STATE::INITIALIZING, std::bind(&DockComponent::UpdateCallback,
      this, std::placeholders::_1, std::placeholders::_2)) {
    // Add the berths -> frame associations
    berths_[ff_msgs::Dock::Goal::BERTH_1] = FRAME_NAME_DOCK_BERTH_1;
    berths_[ff_msgs::Dock::Goal::BERTH_2] = FRAME_NAME_DOCK_BERTH_2;
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
        ff_msgs::GetPipelines::Request req;
        auto response = std::make_shared<ff_msgs::GetPipelines::Response>();
        if (client_l_.call(req, response)) {
          if (!response->pipelines.empty() && response->pipelines[0].id == "ar") {
            // Move to the approach pose using AR localization
            Move(APPROACH_POSE, ff_msgs::Motion::Goal::NOMINAL);
            return STATE::DOCKING_MOVING_TO_APPROACH_POSE;
          }
        }
        Switch(LOCALIZATION_MAPPED_LANDMARKS);
        return STATE::DOCKING_SWITCHING_TO_ML_LOC;
      });
    // [4]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_ML_LOC,
      SWITCH_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Move(APPROACH_POSE, ff_msgs::Motion::Goal::NOMINAL);
        return STATE::DOCKING_MOVING_TO_APPROACH_POSE;
      });
    // [5]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_ML_LOC,
      SWITCH_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::SWITCH_TO_ML_FAILED,
          "Could not switch to mapped landmark localization");
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
        err_msg_ = "Failed while moving to the approach pose";
        Switch(LOCALIZATION_MAPPED_LANDMARKS);
        return STATE::RECOVERY_SWITCHING_TO_ML_LOC;
      });
    // [31]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_AR_LOC,
      SWITCH_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Move(BERTHING_POSE, ff_msgs::Motion::Goal::PRECISION);
        return STATE::DOCKING_MOVING_TO_COMPLETE_POSE;
      });
    // [32]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_AR_LOC, SWITCH_FAILED,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::SWITCH_TO_AR_FAILED,
          "Could not switch to marker tracking localization");
        return STATE::UNDOCKED;
      });
    // [8]
    fsm_.Add(STATE::DOCKING_MOVING_TO_COMPLETE_POSE,
      EPS_DOCKED, [this](FSM::Event const& event) -> FSM::State {
        Move(APPROACH_POSE, ff_msgs::Motion::Goal::PRECISION);
        return STATE::DOCKING_CHECKING_ATTACHED;
      });
    // [9]
    fsm_.Add(STATE::DOCKING_MOVING_TO_COMPLETE_POSE,
      MOTION_SUCCESS | MOTION_FAILED,
      [this](FSM::Event const& event) -> FSM::State {
        if (event == MOTION_SUCCESS) {
          err_ = RESPONSE::EPS_DOCK_FAILED;
          err_msg_ = "Motion success but EPS did not trigger";
        }
        if (event == MOTION_FAILED) {
          err_ = RESPONSE::MOTION_COMPLETE_FAILED;
          err_msg_ = "Motion failed trying to go to complete pose";
        }
        Move(APPROACH_POSE, ff_msgs::Motion::Goal::PRECISION);
        return STATE::RECOVERY_MOVING_TO_APPROACH_POSE;
      });
    // [10]
    fsm_.Add(STATE::DOCKING_CHECKING_ATTACHED,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Prep(ff_msgs::Motion::Goal::OFF);
        return STATE::DOCKING_WAITING_FOR_SPIN_DOWN;
      });
    // [11]
    fsm_.Add(STATE::DOCKING_CHECKING_ATTACHED,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::MOTION_ATTACHED_FAILED;
        err_msg_ = "Check attached failed, magnets didn't stop robot's movement";
        Move(APPROACH_POSE, ff_msgs::Motion::Goal::PRECISION);
        return STATE::RECOVERY_MOVING_TO_APPROACH_POSE;
      });
    // [12]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_NO_LOC,
      SWITCH_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::DOCKED, "Dock completed successfully");
        return STATE::DOCKED;
      });
    // [13]
    fsm_.Add(STATE::DOCKING_SWITCHING_TO_NO_LOC,
      SWITCH_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::SWITCH_TO_NO_FAILED,
          "Could not turn localization off");
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
        if (!CheckBerth()) {
          err_ = RESPONSE::SWITCH_TO_ML_FAILED;
          err_msg_ = "Could not detect berth from current pose, check localization";
          Switch(LOCALIZATION_NONE);
          return STATE::RECOVERY_SWITCHING_TO_NO_LOC;
        }
        Prep(ff_msgs::Motion::Goal::NOMINAL);
        return STATE::UNDOCKING_WAITING_FOR_SPIN_UP;
      });
    // [16]
    fsm_.Add(STATE::UNDOCKING_SWITCHING_TO_ML_LOC,
      SWITCH_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::SWITCH_TO_ML_FAILED;
        err_msg_ = "Switch to Mapped Landmarks Failed";
        Switch(LOCALIZATION_NONE);
        return STATE::RECOVERY_SWITCHING_TO_NO_LOC;
      });
    // [19]
    fsm_.Add(STATE::UNDOCKING_MOVING_TO_APPROACH_POSE,
      MOTION_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::UNDOCKED, "Undock completed successfully");
        return STATE::UNDOCKED;
      });
    // [20]
    fsm_.Add(STATE::UNDOCKING_MOVING_TO_APPROACH_POSE,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::MOTION_APPROACH_FAILED,
          "Could not move to approach point");
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
        Result(err_, "Operation failed with recovery: " + err_msg_);
        return STATE::UNDOCKED;
      });
    // [23]
    fsm_.Add(STATE::RECOVERY_SWITCHING_TO_NO_LOC,
      SWITCH_SUCCESS | SWITCH_FAILED,
      [this](FSM::Event const& event) -> FSM::State {
        Result(err_, "Operation failed with recovery: " + err_msg_);
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
        // Problem case
        if (!Undock()) {
          Prep(ff_msgs::Motion::Goal::OFF);
          err_ = RESPONSE::EPS_UNDOCK_FAILED;
          err_msg_ = "There was a problem calling the eps undock service";
          return STATE::RECOVERY_WAITING_FOR_SPIN_DOWN;
        }
        // Nominal case
        Move(APPROACH_POSE, ff_msgs::Motion::Goal::NOMINAL);
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
        Result(RESPONSE::PREP_DISABLE_FAILED, "Could not spin down motors");
        Switch(LOCALIZATION_NONE);
        return STATE::DOCKED;
      });
    // [29]
    fsm_.Add(STATE::UNDOCKING_WAITING_FOR_SPIN_UP,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::PREP_ENABLE_FAILED;
        err_msg_ = "Spin up was not successful";
        Prep(ff_msgs::Motion::Goal::OFF);
        return STATE::RECOVERY_WAITING_FOR_SPIN_DOWN;
      });
    // [30]
    fsm_.Add(STATE::RECOVERY_WAITING_FOR_SPIN_DOWN,
      MOTION_FAILED | MOTION_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        Switch(LOCALIZATION_NONE);
        return STATE::RECOVERY_SWITCHING_TO_NO_LOC;
      });
    ///////////////////////////////////////////////////
    // [-1] CATCH-ALL FOR CANCELLATIONS / PREEMPTION //
    ///////////////////////////////////////////////////
    fsm_.Add(GOAL_CANCEL | GOAL_PREEMPT,
      [this](FSM::State const& state, FSM::Event const& event) -> FSM::State {
        switch (state) {
        // Values we certainly don't want to mess with
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
          Result(RESPONSE::CANCELLED, "User cancelled the operation");
          break;
        case GOAL_PREEMPT:
          Result(RESPONSE::PREEMPTED, "Third party preempted operation");
          break;
        }
        // Default case is to preserve the state
        return state;
      });
  }
  ~DockComponent() {}

 protected:
  // Called to initialize this component
  void Initialize(NodeHandle &nh) {
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.AddFile("behaviors/dock.config");
    if (!cfg_.Initialize(nh))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                        "Could not start config server", GetTimeNow());

    // One shot timer to check if we undock with a timeout
    timer_eps_.createTimer(cfg_.Get<double>("timeout_eps_response"),
      std::bind(&DockComponent::DockTimerCallback, this), nh, true, false);

    // Set buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh->get_clock());

    // Create a transform buffer to listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(*tf_buffer_));

    // Publish the docking state as a latched topic
    pub_ = FF_CREATE_PUBLISHER(nh, ff_msgs::DockState, TOPIC_BEHAVIORS_DOCKING_STATE, 1);

    // Subscribe to be notified when the dock state changes
    sub_s_ = FF_CREATE_SUBSCRIBER(nh, ff_hw_msgs::EpsDockStateStamped, TOPIC_HARDWARE_EPS_DOCK_STATE, 5,
      std::bind(&DockComponent::DockStateCallback, this, std::placeholders::_1));

    // Allow the state to be manually set
    server_set_state_ = FF_CREATE_SERVICE(nh, ff_msgs::SetState, SERVICE_BEHAVIORS_DOCK_SET_STATE,
      std::bind(&DockComponent::SetStateCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Query the current localization pipeline
    client_l_.Create(nh, SERVICE_LOCALIZATION_MANAGER_GET_CURR_PIPELINE);

    // Contact EPS service for undocking
    client_u_.SetConnectedTimeout(cfg_.Get<double>("timeout_undock_connected"));
    client_u_.SetConnectedCallback(std::bind(
      &DockComponent::ConnectedCallback, this));
    client_u_.SetTimeoutCallback(std::bind(
      &DockComponent::UndockTimeoutCallback, this));
    client_u_.Create(nh, SERVICE_HARDWARE_EPS_UNDOCK);

    // Setup move client action
    client_m_.SetConnectedTimeout(cfg_.Get<double>("timeout_motion_connected"));
    client_m_.SetActiveTimeout(cfg_.Get<double>("timeout_motion_active"));
    client_m_.SetResponseTimeout(cfg_.Get<double>("timeout_motion_response"));
    client_m_.SetFeedbackCallback(std::bind(&DockComponent::MFeedbackCallback,
      this, std::placeholders::_1));
    client_m_.SetResultCallback(std::bind(&DockComponent::MResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_m_.SetConnectedCallback(std::bind(
      &DockComponent::ConnectedCallback, this));
    client_m_.Create(nh, ACTION_MOBILITY_MOTION);

    // Setup switch client action
    client_s_.SetConnectedTimeout(cfg_.Get<double>("timeout_switch_connected"));
    client_s_.SetActiveTimeout(cfg_.Get<double>("timeout_switch_active"));
    client_s_.SetResponseTimeout(cfg_.Get<double>("timeout_switch_response"));
    client_s_.SetDeadlineTimeout(cfg_.Get<double>("timeout_switch_deadline"));
    client_s_.SetFeedbackCallback(std::bind(
      &DockComponent::SFeedbackCallback, this, std::placeholders::_1));
    client_s_.SetResultCallback(std::bind(&DockComponent::SResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_s_.SetConnectedCallback(std::bind(&DockComponent::ConnectedCallback,
      this));
    client_s_.Create(nh, ACTION_LOCALIZATION_MANAGER_LOCALIZATION);

    // Setup the execute action
    server_.SetGoalCallback(std::bind(
      &DockComponent::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &DockComponent::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &DockComponent::CancelCallback, this));
    server_.Create(nh, ACTION_BEHAVIORS_DOCK);
  }

  // Timeout on a trajectory generation request
  void EnableTimeoutCallback(void) {
    return AssertFault(ff_util::INITIALIZATION_FAILED,
                       "Could not find enable service", GetTimeNow());
  }

  // Timeout on a trajectory generation request
  void UndockTimeoutCallback(void) {
    return AssertFault(ff_util::INITIALIZATION_FAILED,
                       "Could not find undock service", GetTimeNow());
  }

  // Ensure all clients are connected
  void ConnectedCallback() {
    FF_DEBUG_STREAM("ConnectedCallback()");
    if (!client_u_.IsConnected()) return;       // Undock service
    if (!client_s_.IsConnected()) return;       // Switch action
    if (!client_m_.IsConnected()) return;       // Move action
    fsm_.Update(READY);                         // Ready!
  }

  // Called on registration of a planner
  bool SetStateCallback(const std::shared_ptr<ff_msgs::SetState::Request> req,
                        std::shared_ptr<ff_msgs::SetState::Response> res) {
    fsm_.SetState(req->state);
    res->success = true;
    UpdateCallback(fsm_.GetState(), MANUAL_STATE_SET);
    return true;
  }

  // Complete the current dock or undock action
  void Result(int32_t response, std::string const& msg = "") {
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
    auto result = std::make_shared<ff_msgs::Dock::Result>();
    result->fsm_result = msg;
    result->response = response;

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
    ff_msgs::DockState msg;
    msg.header.frame_id = GetPlatform();
    msg.header.stamp = GetTimeNow();
    msg.state = state;
    // Debug events
    switch (event) {
    case READY:            msg.fsm_event = "READY";            break;
    case GOAL_DOCK:        msg.fsm_event = "GOAL_DOCK";        break;
    case GOAL_UNDOCK:      msg.fsm_event = "GOAL_UNDOCK";      break;
    case GOAL_CANCEL:      msg.fsm_event = "GOAL_CANCEL";      break;
    case EPS_DOCKED:       msg.fsm_event = "EPS_DOCKED";       break;
    case EPS_UNDOCKED:     msg.fsm_event = "EPS_UNDOCKED";     break;
    case EPS_TIMEOUT:      msg.fsm_event = "EPS_TIMEOUT";      break;
    case SWITCH_SUCCESS:   msg.fsm_event = "SWITCH_SUCCESS";   break;
    case SWITCH_FAILED:    msg.fsm_event = "SWITCH_FAILED";    break;
    case MOTION_SUCCESS:   msg.fsm_event = "MOTION_SUCCESS";   break;
    case MOTION_FAILED:    msg.fsm_event = "MOTION_FAILED";    break;
    case MANUAL_STATE_SET: msg.fsm_event = "MANUAL_STATE_SET"; break;
    }
    FF_DEBUG_STREAM("Received event " << msg.fsm_event);
    // Debug state changes
    switch (state) {
    case STATE::INITIALIZING:
      msg.fsm_state = "INITIALIZING";                      break;
    case STATE::UNKNOWN:
      msg.fsm_state = "UNKNOWN";                           break;
    case STATE::UNDOCKED:
      msg.fsm_state = "UNDOCKED";                          break;
    case STATE::DOCKING_SWITCHING_TO_ML_LOC:
      msg.fsm_state = "DOCKING_SWITCHING_TO_ML_LOC";       break;
    case STATE::DOCKING_MOVING_TO_APPROACH_POSE:
      msg.fsm_state = "DOCKING_MOVING_TO_APPROACH_POSE";   break;
    case STATE::DOCKING_SWITCHING_TO_AR_LOC:
      msg.fsm_state = "DOCKING_SWITCHING_TO_AR_LOC";       break;
    case STATE::DOCKING_MOVING_TO_COMPLETE_POSE:
      msg.fsm_state = "DOCKING_MOVING_TO_COMPLETE_POSE";   break;
    case STATE::DOCKING_CHECKING_ATTACHED:
      msg.fsm_state = "DOCKING_CHECKING_ATTACHED";         break;
    case STATE::DOCKING_WAITING_FOR_SPIN_DOWN:
      msg.fsm_state = "DOCKING_WAITING_FOR_SPIN_DOWN";     break;
    case STATE::DOCKING_SWITCHING_TO_NO_LOC:
      msg.fsm_state = "DOCKING_SWITCHING_TO_NO_LOC";       break;
    case STATE::DOCKED:
      msg.fsm_state = "DOCKED";                            break;
    case STATE::UNDOCKING_SWITCHING_TO_ML_LOC:
      msg.fsm_state = "UNDOCKING_SWITCHING_TO_ML_LOC";     break;
    case STATE::UNDOCKING_WAITING_FOR_SPIN_UP:
      msg.fsm_state = "UNDOCKING_WAITING_FOR_SPIN_UP";     break;
    case STATE::UNDOCKING_MOVING_TO_APPROACH_POSE:
      msg.fsm_state = "UNDOCKING_MOVING_TO_APPROACH_POSE"; break;
    case STATE::RECOVERY_SWITCHING_TO_NO_LOC:
      msg.fsm_state = "RECOVERY_SWITCHING_TO_NO_LOC";      break;
    case STATE::RECOVERY_MOVING_TO_APPROACH_POSE:
      msg.fsm_state = "RECOVERY_MOVING_TO_APPROACH_POSE";  break;
    case STATE::RECOVERY_SWITCHING_TO_ML_LOC:
      msg.fsm_state = "RECOVERY_SWITCHING_TO_ML_LOC";      break;
    }
    FF_DEBUG_STREAM("State changed to " << msg.fsm_state);
    // Broadcast the docking state
    pub_->publish(msg);
    // Send the feedback if needed
    switch (state) {
    case STATE::INITIALIZING:
    case STATE::UNKNOWN:
    case STATE::DOCKED:
    case STATE::UNDOCKED:
      break;
    default:
      {
        auto feedback = std::make_shared<ff_msgs::Dock::Feedback>();
        feedback->state = msg;
        server_.SendFeedback(feedback);
      }
    }
  }

  // Check that we are close enough to the approach pose
  bool CloseEnoughToApproach(std::string berth) {
    try {
      // Look up the body frame in the berth frame
      geometry_msgs::TransformStamped tf = tf_buffer_->lookupTransform(
        berth + "/approach", GetTransform(FRAME_NAME_BODY),
          tf2::TimePointZero);
      // Copy the transform
      double d = tf.transform.translation.x * tf.transform.translation.x
               + tf.transform.translation.y * tf.transform.translation.y
               + tf.transform.translation.z * tf.transform.translation.z;
      // We need to be within the initial tolerance to start docking
      if (sqrt(d) > cfg_.Get<double>("initial_tolerance"))
        return false;
    } catch (const tf2::TransformException &ex) {
      return false;
    }
    return true;
  }

  bool CheckBerth() {
    // Look for the berth and confirm localization is working
    std::map<uint8_t, std::string>::iterator it;
    rclcpp::Time begin = GetTimeNow();
    // Some number bigger than the tolerance
    double d = cfg_.Get<double>("detection_tolerance") * 2;
    // Give localization time to stabilize if berth not detected within detection_timeout
    while (((GetTimeNow() - begin).seconds() < cfg_.Get<double>("detection_timeout"))
          && sqrt(d) > cfg_.Get<double>("detection_tolerance")) {
      for (it = berths_.begin(); it != berths_.end(); it++) {
        try {
          // Look up the body frame in the berth frame
          geometry_msgs::TransformStamped tf = tf_buffer_->lookupTransform(
            it->second + "/complete", GetTransform(FRAME_NAME_BODY), tf2::TimePointZero);
          // Copy the transform
          d = tf.transform.translation.x * tf.transform.translation.x
            + tf.transform.translation.y * tf.transform.translation.y
            + tf.transform.translation.z * tf.transform.translation.z;
          // If we are within some delta of the origin, then we are at this berth
          if (sqrt(d) < cfg_.Get<double>("detection_tolerance"))
            break;
          else
            rclcpp::sleep_for(std::chrono::milliseconds(500));  // sleep for half a second
        } catch (const tf2::TransformException &ex) {}
      }
    }
    // Let the use know what's happening
    if (it == berths_.end()) {
      FF_ERROR_STREAM("Could not detect berth from current pose");
      return false;
    } else {
      FF_DEBUG_STREAM("Berth frame detected: " << it->second);
      // At this point we should have good AR or ML localization, so we can
      // determine our pose to within a couple centimeters.
      frame_ = it->second;
    }
    return true;
  }
  // EPS (DOCKING)

  bool Undock() {
    // Any error finding the berth transform or calling the service will result
    // in an EPS_TIMEOUT event, which the FSM will use to recover.
    timer_eps_.start();
    // Call the undock service
    ff_hw_msgs::Undock::Request request;
    auto response = std::make_shared<ff_hw_msgs::Undock::Response>();
    if (!client_u_.Call(request, response))
      return false;
    // Check that we actually called EPS undock() successfully
    switch (response->value) {
    case ff_hw_msgs::Undock::Response::SUCCESS:
      FF_DEBUG_STREAM("Undocking called successfully");
      return true;
    case ff_hw_msgs::Undock::Response::UNDOCK_FAILED:
    default:
      break;
    }
    FF_DEBUG_STREAM("There was a problem calling the undock service");
    return false;
  }

  void DockStateCallback(const std::shared_ptr<ff_hw_msgs::EpsDockStateStamped> msg) {
    switch (msg->state) {
    // We don't worry about a timeout on docking, because we'll get a motion
    // failure if docking doesn't succeed.
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

  void DockTimerCallback() {
    return fsm_.Update(EPS_TIMEOUT);
  }

  // SWITCH action

  // Helper function for localization switching
  bool Switch(std::string const& pipeline) {
    // Send the switch goal
    ff_msgs::Localization::Goal goal;
    goal.command = ff_msgs::Localization::Goal::COMMAND_SWITCH_PIPELINE;
    goal.pipeline = pipeline;
    return client_s_.SendGoal(goal);
  }

  // Ignore the switch feedback for now
  void SFeedbackCallback(
    const std::shared_ptr<const ff_msgs::Localization::Feedback> feedback) {}

  // Do something with the switch result
  void SResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    std::shared_ptr<const ff_msgs::Localization::Result> result) {
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
    static ff_msgs::Motion::Goal goal;
    goal.command = ff_msgs::Motion::Goal::PREP;
    goal.flight_mode = flight_mode;
    return client_m_.SendGoal(goal);
  }

  // Send a move command
  bool Move(DockPose type, std::string const& mode) {
    // Create a new motion foal
    ff_msgs::Motion::Goal goal;
    goal.command = ff_msgs::Motion::Goal::MOVE;
    goal.flight_mode = mode;

    // Package up the desired end pose
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = GetTimeNow();
    ff_util::ConfigClient cfg(node_, NODE_CHOREOGRAPHER);
    // Set parameters for the choreographer
    cfg.Set<bool>("enable_collision_checking", false);
    cfg.Set<bool>("enable_validation", false);
    cfg.Set<bool>("enable_bootstrapping", true);
    cfg.Set<bool>("enable_immediate", true);
    cfg.Set<bool>("enable_faceforward", false);
    cfg.Set<double>("desired_vel", -1.0);
    cfg.Set<double>("desired_accel", -1.0);
    cfg.Set<double>("desired_omega", -1.0);
    cfg.Set<double>("desired_alpha", -1.0);
    cfg.Set<std::string>("planner", "trapezoidal");

    switch (type) {
    case BERTHING_POSE:
      msg.header.frame_id = frame_ + "/approach";
      goal.states.push_back(msg);
      msg.header.frame_id = frame_;
      goal.states.push_back(msg);
      break;
    case APPROACH_POSE:
    {
      msg.header.frame_id = frame_ + "/approach";
      goal.states.push_back(msg);

      // if docking and return to dock enabled
      if (!CloseEnoughToApproach(frame_) &&
          fsm_.GetState() == STATE::DOCKING_SWITCHING_TO_ML_LOC) {
        // Load parameters from config file
        std::string planner = cfg_.Get<std::string>("planner");
        bool coll_check = cfg_.Get<bool>("enable_collision_checking");
        bool face_forward = cfg_.Get<bool>("enable_faceforward");
        bool replanning = cfg_.Get<bool>("enable_replanning");
        int replanning_attempts = cfg_.Get<int>("max_replanning_attempts");
        bool validation = cfg_.Get<bool>("enable_validation");
        bool boostrapping = cfg_.Get<bool>("enable_bootstrapping");
        bool immediate = cfg_.Get<bool>("enable_immediate");
        double desired_vel = cfg_.Get<double>("desired_vel");
        double desired_accel = cfg_.Get<double>("desired_accel");
        double desired_omega = cfg_.Get<double>("desired_omega");
        double desired_alpha = cfg_.Get<double>("desired_alpha");
        // Set parameters for the choreographer
        cfg.Set<std::string>("planner", planner);
        cfg.Set<bool>("enable_collision_checking", coll_check);
        cfg.Set<bool>("enable_faceforward", face_forward);
        cfg.Set<bool>("enable_replanning", replanning);
        cfg.Set<int>("max_replanning_attempts", replanning_attempts);
        cfg.Set<bool>("enable_validation", validation);
        cfg.Set<bool>("enable_bootstrapping", boostrapping);
        cfg.Set<bool>("enable_immediate", immediate);
        cfg.Set<double>("desired_vel", desired_vel);
        cfg.Set<double>("desired_accel", desired_accel);
        cfg.Set<double>("desired_omega", desired_omega);
        cfg.Set<double>("desired_alpha", desired_alpha);
      }
      break;
    }
    default:
      return false;
    }

    // Iterate over all poses in action, finding the location of each
    for (auto & pose : goal.states) {
      try {
        // Look up the world -> berth transform
        geometry_msgs::TransformStamped tf = tf_buffer_->lookupTransform(
          "world", pose.header.frame_id, tf2::TimePointZero);
        // Copy the transform
        pose.pose = msg_conversions::ros_transform_to_ros_pose(tf.transform);
      } catch (const tf2::TransformException &ex) {
        FF_WARN_STREAM("Transform failed" << ex.what());
        return false;
      }
    }
    // Reconfigure the choreographer
    if (!cfg.Reconfigure()) {
      FF_ERROR_STREAM("Failed to reconfigure choreographer");
      return false;
    }
    // Send the goal to the mobility subsystem
    return client_m_.SendGoal(goal);
  }

  // Ignore the move feedback, for now
  void MFeedbackCallback(const std::shared_ptr<const ff_msgs::Motion::Feedback> feedback) {}

  // Result of a move action
  void MResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    std::shared_ptr<const ff_msgs::Motion::Result> result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return fsm_.Update(MOTION_SUCCESS);
    default:
      return fsm_.Update(MOTION_FAILED);
    }
  }

  // DOCK ACTION SERVER

  // A new arm action has been called
  void GoalCallback(std::shared_ptr<const ff_msgs::Dock::Goal> goal) {
    auto result = std::make_shared<ff_msgs::Dock::Result>();
    switch (goal->command) {
    case ff_msgs::Dock::Goal::DOCK:
      // We are undocked
      if (fsm_.GetState() == STATE::UNDOCKED) {
        // Do we know about the specified berth?
        if (berths_.find(goal->berth) == berths_.end()) {
          result->fsm_result = "Invalid berth specified";
          result->response = RESPONSE::INVALID_BERTH;
          server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
          return;
        }
        // Check that we are close enough to the approach pose
        if (!CloseEnoughToApproach(berths_[goal->berth]) && !goal->return_dock) {
          result->fsm_result = "Too far from dock";
          result->response = RESPONSE::TOO_FAR_AWAY_FROM_APPROACH;
          server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
          return;
        }
        // If we do know about it, add a frame id prefix
        frame_ = berths_[goal->berth];
        // Start docking
        return fsm_.Update(GOAL_DOCK);
      // We are already docked
      } else if (fsm_.GetState() == STATE::DOCKED) {
        result->fsm_result = "The robot is already docked";
        result->response = RESPONSE::ALREADY_DOCKED;
        server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
        return;
      // We are not in  a position to dock
      } else {
        result->fsm_result = "Docking only possible if undocked";
        result->response = RESPONSE::NOT_IN_UNDOCKED_STATE;
        server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
        return;
      }
      break;
    // Undock command
    case ff_msgs::Dock::Goal::UNDOCK:
      // We are docked
      if (fsm_.GetState() == STATE::DOCKED) {
        return fsm_.Update(GOAL_UNDOCK);
      // We are already undocked
      } else if (fsm_.GetState() == STATE::UNDOCKED) {
        result->fsm_result = "The robot is already undocked";
        result->response = RESPONSE::ALREADY_UNDOCKED;
        server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
        return;
      // We are not in a position to undock
      } else {
        result->fsm_result = "Undocking only possible if docked";
        result->response = RESPONSE::NOT_IN_DOCKED_STATE;
      }
      break;
    // Invalid command
    default:
      result->fsm_result = "Invalid command in request";
      result->response = RESPONSE::INVALID_COMMAND;
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

 protected:
  std::map<uint8_t, std::string> berths_;
  ff_util::FSM fsm_;
  ff_util::FreeFlyerActionClient<ff_msgs::Motion> client_m_;
  ff_util::FreeFlyerActionClient<ff_msgs::Localization> client_s_;
  ff_util::FreeFlyerServiceClient<ff_msgs::GetPipelines>  client_l_;
  ff_util::FreeFlyerServiceClient<ff_hw_msgs::Undock> client_u_;
  ff_util::FreeFlyerActionServer<ff_msgs::Dock> server_;
  ff_util::ConfigServer cfg_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<ff_msgs::DockState>::SharedPtr pub_;
  rclcpp::Subscription<ff_hw_msgs::EpsDockStateStamped>::SharedPtr sub_s_;
  rclcpp::Service<ff_msgs::SetState>::SharedPtr server_set_state_;
  ff_util::FreeFlyerTimer timer_eps_;
  ff_util::FreeFlyerTimer timer_pmc_;
  std::string frame_;
  int32_t err_;
  std::string err_msg_;
};

}  // namespace dock

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(dock::DockComponent)
