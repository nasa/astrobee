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

// Standard messages
#include <std_srvs/srv/empty.hpp>
namespace std_srvs {
  typedef srv::Empty Empty;
}  // namespace std_srvs

#include <geometry_msgs/msg/pose_stamped.hpp>
namespace geometry_msgs {
  typedef msg::PoseStamped PoseStamped;
}  // namespace geometry_msgs

// FSW messages, services and actions
#include <ff_msgs/msg/arm_state_stamped.hpp>
#include <ff_msgs/msg/arm_joint_state.hpp>
#include <ff_msgs/msg/perch_state.hpp>
#include <ff_msgs/srv/set_state.hpp>
#include <ff_msgs/srv/set_bool.hpp>
#include <ff_msgs/action/motion.hpp>
#include <ff_msgs/action/localization.hpp>
#include <ff_msgs/action/arm.hpp>
#include <ff_msgs/action/perch.hpp>
namespace ff_msgs {
  typedef msg::ArmStateStamped ArmStateStamped;
  typedef msg::ArmJointState ArmJointState;
  typedef msg::PerchState PerchState;
  typedef srv::SetState SetState;
  typedef srv::SetBool SetBool;
  typedef action::Motion Motion;
  typedef action::Localization Localization;
  typedef action::Arm Arm;
  typedef action::Perch Perch;
}  // namespace ff_msgs

#include <ff_hw_msgs/srv/set_enabled.hpp>
namespace ff_hw_msgs {
  typedef srv::SetEnabled SetEnabled;
}  // namespace ff_hw_msgs

/**
 * \ingroup beh
 */
namespace perch {

FF_DEFINE_LOGGER("perch");

// Match the internal states and responses with the message definition
using FSM = ff_util::FSM;
using STATE = ff_msgs::PerchState;
using RESPONSE = ff_msgs::Perch::Result;

/*
  This class provides the high-level logic that allows the freeflyer to
  perform both perching and unperching from a handrail. Please see the accom-
  panying state diagram for a high level overview of this logic. Note that
  there is an auto-recovery mechanism to avoid a motion tolerance error from
  causing the freeflyer to drift uncontrollably into / away from the perch.

  Here are the rules of auto-recovery in the context of perching:

    * If the callee cancels the task the system stops immediately. Nothing
      further is carried out, under the key assumption that a cancellation
      implies intervention due to something having gone quite wrong.

    * If something during the perching or unperching procedure goes wrong,
      the server aborts the task, then the system moves to an auto-recovery
      sequence. The callee is only notified of the error AFTER the auto-
      recovery completes, to avoid perch and executive contention.

    * If a perch or unperch procedure is preempted, the system must abort
      the current task immediately, so that the new goal can be accepted,
      regardless of the progress (or lack thereof). The end state will
      either be perched (if still attached) or unperched (if in flight).
*/
class PerchComponent : public ff_util::FreeFlyerComponent {
 public:
  // All possible events that can occur
  enum : FSM::Event {
    READY             = (1<<0),     // All services connected
    GOAL_PERCH        = (1<<1),     // Start perching
    GOAL_UNPERCH      = (1<<2),     // Start unperching
    GOAL_CANCEL       = (1<<3),     // Cancel an existing goal
    GOAL_PREEMPT      = (1<<4),     // Preempt an existing goal
    ARM_DEPLOYED      = (1<<5),     // Arm is deployed
    ARM_STOWED        = (1<<6),     // Arm is stowed
    ARM_SUCCESS       = (1<<7),     // Arm action succeeded
    ARM_FAILED        = (1<<8),     // Arm action failed
    SWITCH_SUCCESS    = (1<<9),     // Localization manager switch result
    SWITCH_FAILED     = (1<<10),    // Localization manager switch failed
    MOTION_SUCCESS    = (1<<11),    // Mobility motion action success
    MOTION_FAILED     = (1<<12),    // Mobility motion action problem
    MANUAL_STATE_SET  = (1<<13),    // Setting the state manually with service
  };

  // Positions that we might need to move to
  enum PerchPose {
    APPROACH_POSE,      // The starting poses of perching
    COMPLETE_POSE,      // The completion pose of perching
    RECOVERY_POSE,      // The pose to safely stow the arm
    UNPERCHED_POSE,     // The pose when asking for unperching
    PERCHED_POSE        // The pose when we are perched
  };

  // Perching arm servos to enable / disable
  enum ServoID {
    PROXIMAL,           // Proximal joint servo
    DISTAL,             // Distal joint servo
    GRIPPER,            // Gripper joint servo
    ALL_SERVOS          // Proximal, Distal and Gripper servos
  };

  // Constructor bootstraps freeflyer component and sets initial FSM state
  explicit PerchComponent(const rclcpp::NodeOptions& options) : ff_util::FreeFlyerComponent(options, NODE_PERCH, true),
    fsm_(STATE::INITIALIZING, std::bind(&PerchComponent::UpdateCallback,
      this, std::placeholders::_1, std::placeholders::_2)) {
    ////////////////////////////////
    // NON-BLOCKING STATE UPDATES //
    ////////////////////////////////
    // [0] - The system has just started. Wait until all services are
    // connected and then move to the ready state.
    fsm_.Add(STATE::INITIALIZING,
      READY, [this](FSM::Event const& event) -> FSM::State {
        return STATE::UNKNOWN;
      });
    // [1] - If the state is unknown, if the arm reports as being
    // stowed then we will assume that we are in an unperched state.
    fsm_.Add(STATE::UNKNOWN,
      ARM_STOWED, [this](FSM::Event const& event) -> FSM::State {
        return STATE::UNPERCHED;
      });
    // [2] - If the state is unknown, if the arm reports as being
    // deployed then we will assume that we are in a perched state.
    fsm_.Add(STATE::UNKNOWN,
      ARM_DEPLOYED, [this](FSM::Event const& event) -> FSM::State {
        return STATE::PERCHED;
      });
    ////////////////////////////
    // NOMINAL PERCH SEQUENCE //
    ////////////////////////////
    // [3] - If unperched and a perch goal is received, try switching
    // to mapped landmark localization.
    fsm_.Add(STATE::UNPERCHED,
      GOAL_PERCH, [this](FSM::Event const& event) -> FSM::State {
        Switch(LOCALIZATION_HANDRAIL);
        return STATE::PERCHING_SWITCHING_TO_HR_LOC;
      });
    // [4] - If we successfuly switched to handrail navigation
    // then we can try moving to the approach pose in nominal mode.
    fsm_.Add(STATE::PERCHING_SWITCHING_TO_HR_LOC,
      SWITCH_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        SaveApproachPose();
        Move(APPROACH_POSE, ff_msgs::Motion::Goal::NOMINAL);
        return STATE::PERCHING_MOVING_TO_APPROACH_POSE;
      });
    // [5] - If the move to approach pose was successful, we save the approach pose
    // as a landmark and ask for arm deployment.
    fsm_.Add(STATE::PERCHING_MOVING_TO_APPROACH_POSE,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        SaveApproachPose();
        Arm(ff_msgs::Arm::Goal::ARM_DEPLOY);
        return STATE::PERCHING_DEPLOYING_ARM;
      });
    // [6] - If we successfully deployed the arm, then we open its gripper.
    fsm_.Add(STATE::PERCHING_DEPLOYING_ARM,
      ARM_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Arm(ff_msgs::Arm::Goal::GRIPPER_OPEN);
        return STATE::PERCHING_OPENING_GRIPPER;
      });
    // [7] - If the gripper is opened, we move to complete pose.
    fsm_.Add(STATE::PERCHING_OPENING_GRIPPER,
      ARM_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Move(COMPLETE_POSE, ff_msgs::Motion::Goal::PRECISION);
        return STATE::PERCHING_MOVING_TO_COMPLETE_POSE;
      });
    // [8] - If the complete pose is reached, we close the gripper.
    fsm_.Add(STATE::PERCHING_MOVING_TO_COMPLETE_POSE,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Arm(ff_msgs::Arm::Goal::GRIPPER_CLOSE);
        return STATE::PERCHING_CLOSING_GRIPPER;
      });
    // [9] - If the gripper is closed, we try moving away from the handrail.
    fsm_.Add(STATE::PERCHING_CLOSING_GRIPPER,
      ARM_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Move(APPROACH_POSE, ff_msgs::Motion::Goal::NOMINAL);
        return STATE::PERCHING_CHECKING_ATTACHED;
      });
    // [10] - If the motion is failed, we are grasped and ask to stop movement.
    fsm_.Add(STATE::PERCHING_CHECKING_ATTACHED,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Prep(ff_msgs::Motion::Goal::OFF);
        return STATE::PERCHING_WAITING_FOR_SPIN_DOWN;
      });
    // [11] - If we successfully stopped, we switch to perch localization (future work)
    fsm_.Add(STATE::PERCHING_WAITING_FOR_SPIN_DOWN,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Switch(LOCALIZATION_MAPPED_LANDMARKS);
        return STATE::PERCHING_SWITCHING_TO_PL_LOC;
      });
    // [12] - With all steps done, we conclude we are perched.
    fsm_.Add(STATE::PERCHING_SWITCHING_TO_PL_LOC,
      SWITCH_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::PERCHED, "Successfully perched");
        return STATE::PERCHED;
      });
    //////////////////////////////
    // NOMINAL UNPERCH SEQUENCE //
    //////////////////////////////
    // [13] - Switch to handrail localization.
    fsm_.Add(STATE::PERCHED,
      GOAL_UNPERCH, [this](FSM::Event const& event) -> FSM::State {
        Switch(LOCALIZATION_MAPPED_LANDMARKS);
        return STATE::UNPERCHING_SWITCHING_TO_ML_LOC;
      });
    // [14] -If the switch is successful,
    // spin up the propellers to enable motion commands.
    fsm_.Add(STATE::UNPERCHING_SWITCHING_TO_ML_LOC,
      SWITCH_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Prep(ff_msgs::Motion::Goal::NOMINAL);
        return STATE::UNPERCHING_WAITING_FOR_SPIN_UP;
      });
    // [15] - If the propellers are spinning, open the gripper.
    fsm_.Add(STATE::UNPERCHING_WAITING_FOR_SPIN_UP,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Arm(ff_msgs::Arm::Goal::GRIPPER_OPEN);
        return STATE::UNPERCHING_OPENING_GRIPPER;
      });
    // [16] - If the gripper opened successfully, move to the approach pose.
    fsm_.Add(STATE::UNPERCHING_OPENING_GRIPPER,
      ARM_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Move(APPROACH_POSE, ff_msgs::Motion::Goal::NOMINAL);
        return STATE::UNPERCHING_MOVING_TO_APPROACH_POSE;
      });
    // [17] - Once at the approach pose, stow the arm.
    fsm_.Add(STATE::UNPERCHING_MOVING_TO_APPROACH_POSE,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Arm(ff_msgs::Arm::Goal::ARM_STOW);
        return STATE::UNPERCHING_STOWING_ARM;
      });
    // [18] - With all steps dones, conclude we are unperched.
    fsm_.Add(STATE::UNPERCHING_STOWING_ARM,
      ARM_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::UNPERCHED, "Successfully unperched");
        return STATE::UNPERCHED;
      });

    //////////////////////////////
    // MONIAL-RECOVERY BRANCHES //
    //////////////////////////////
    // --------------------------perching
    // [19]
    fsm_.Add(STATE::PERCHING_SWITCHING_TO_HR_LOC,
      SWITCH_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::SWITCH_FAILED,
          "Switch to handrail localization failed - UNPERCHED");
        return STATE::UNPERCHED;
      });
    // [20]
    fsm_.Add(STATE::PERCHING_MOVING_TO_APPROACH_POSE,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::MOTION_FAILED;
        err_msg_ = "Failed while moving to approach pose";
        Switch(LOCALIZATION_MAPPED_LANDMARKS);
        return STATE::RECOVERY_SWITCHING_TO_ML_LOC;
      });
    // [21]
    fsm_.Add(STATE::PERCHING_DEPLOYING_ARM,
      ARM_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::ARM_FAILED;
        err_msg_ = "Failed while deploying the arm";
        Switch(LOCALIZATION_MAPPED_LANDMARKS);
        return STATE::RECOVERY_SWITCHING_TO_ML_LOC;
      });
    // [22]
    fsm_.Add(STATE::PERCHING_OPENING_GRIPPER,
      ARM_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::ARM_FAILED;
        err_msg_ = "Failed while opening the gripper";
        Arm(ff_msgs::Arm::Goal::ARM_STOW);
        return STATE::RECOVERY_STOWING_ARM;
      });
    // [23]
    fsm_.Add(STATE::PERCHING_MOVING_TO_COMPLETE_POSE,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::MOTION_FAILED;
        err_msg_ = "Failed while moving to complete pose";
        Arm(ff_msgs::Arm::Goal::GRIPPER_OPEN);
        return STATE::RECOVERY_OPENING_GRIPPER;
      });
    // [24]
    fsm_.Add(STATE::PERCHING_CLOSING_GRIPPER,
      ARM_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::ARM_FAILED;
        err_msg_ = "Failed while closing the gripper";
        Arm(ff_msgs::Arm::Goal::GRIPPER_OPEN);
        return STATE::RECOVERY_OPENING_GRIPPER;
      });
    // [25]
    fsm_.Add(STATE::PERCHING_CHECKING_ATTACHED,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::MOTION_FAILED;
        err_msg_ = "Failed while checking the attachment";
        Arm(ff_msgs::Arm::Goal::GRIPPER_OPEN);
        return STATE::RECOVERY_OPENING_GRIPPER;
      });
    // [26]
    fsm_.Add(STATE::PERCHING_WAITING_FOR_SPIN_DOWN,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::MOTION_FAILED,
          "Spinning down propulsion failed");
        return STATE::PERCHED;
      });
    // [27]
    fsm_.Add(STATE::PERCHING_SWITCHING_TO_PL_LOC,
      SWITCH_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::SWITCH_FAILED,
          "Switch to perch localization failed");
        return STATE::PERCHED;
      });

    // --------------------------unperching
    // [28]
    fsm_.Add(STATE::UNPERCHING_SWITCHING_TO_ML_LOC,
      SWITCH_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::SWITCH_FAILED,
          "Switch to mapped landmarks localization failed");
        return STATE::PERCHED;
      });
    // [29]
    fsm_.Add(STATE::UNPERCHING_WAITING_FOR_SPIN_UP,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::MOTION_FAILED,
          "Spinning up propulsion failed");
        return STATE::PERCHED;
      });
    // [30]
    fsm_.Add(STATE::UNPERCHING_OPENING_GRIPPER,
      ARM_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::ARM_FAILED,
          "Gripper open failed");
        return STATE::PERCHED;
      });
    // [31]
    fsm_.Add(STATE::UNPERCHING_MOVING_TO_APPROACH_POSE,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::MOTION_FAILED;
        err_msg_ = "Failed while moving to approach pose";
        Move(RECOVERY_POSE, ff_msgs::Motion::Goal::NOMINAL);
        return STATE::RECOVERY_MOVING_TO_RECOVERY_POSE;
      });
    // [32]
    fsm_.Add(STATE::UNPERCHING_STOWING_ARM,
      ARM_FAILED, [this](FSM::Event const& event) -> FSM::State {
        err_ = RESPONSE::ARM_FAILED;
        err_msg_ = "Failed while stowing the arm";
        Arm(ff_msgs::Arm::Goal::ARM_STOW);
        return STATE::RECOVERY_STOWING_ARM;
      });

    ///////////////////////
    // RECOVERY SEQUENCE //
    ///////////////////////
    // [33] - Opening gripper for recovery attempt: Success and Fail options
    fsm_.Add(STATE::RECOVERY_OPENING_GRIPPER,
      ARM_SUCCESS | ARM_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Switch(LOCALIZATION_MAPPED_LANDMARKS);
        return STATE::RECOVERY_SWITCHING_TO_ML_LOC;
      });
    // [34] - Switching to ML loc in recovery: Success and Fail options
    fsm_.Add(STATE::RECOVERY_SWITCHING_TO_ML_LOC,
      SWITCH_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Move(RECOVERY_POSE, ff_msgs::Motion::Goal::NOMINAL);
        return STATE::RECOVERY_MOVING_TO_RECOVERY_POSE;
      });
    fsm_.Add(STATE::RECOVERY_SWITCHING_TO_ML_LOC,
      SWITCH_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(err_, "Recovery switch to mapped landmarks localization failed: " + err_msg_);
        return STATE::UNPERCHED;
      });
    // [35] - Motion to recovery pose: Success and Fail options
    fsm_.Add(STATE::RECOVERY_MOVING_TO_RECOVERY_POSE,
      MOTION_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Arm(ff_msgs::Arm::Goal::ARM_STOW);
        return STATE::RECOVERY_STOWING_ARM;
      });
    fsm_.Add(STATE::RECOVERY_MOVING_TO_RECOVERY_POSE,
      MOTION_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(err_, "Recovery motion to recovery pose failed: " + err_msg_);
        return STATE::UNPERCHED;
      });
    // [36] - Stowing arm in recovery: Success and Fail options
    fsm_.Add(STATE::RECOVERY_STOWING_ARM,
      ARM_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Result(err_, "Successful Recovery: " + err_msg_);
        return STATE::UNPERCHED;
      });
    fsm_.Add(STATE::RECOVERY_STOWING_ARM,
      ARM_FAILED, [this](FSM::Event const& event) -> FSM::State {
        Result(err_, "Recovery Stowing arm failed: " + err_msg_);
        return STATE::UNPERCHED;
      });

    //////////////////////////////////////////////
    // CATCH-ALL FOR CANCELLATIONS / PREEMPTION //
    //////////////////////////////////////////////
    // [A] - If a GOAL is CANCELLED or preempted, we should immediately.
    fsm_.Add(GOAL_CANCEL | GOAL_PREEMPT,
      [this](FSM::State const& state, FSM::Event const& event) -> FSM::State {
        // Send a response stating that we were canceled
        if (event == GOAL_CANCEL)
          Result(RESPONSE::CANCELLED, "Cancelled by callee");
        else
          Result(RESPONSE::PREEMPTED, "Preempted by third party");

        // Cancel any leftover actions
        switch (state) {
        default:
        case STATE::INITIALIZING:
        case STATE::UNKNOWN:
        case STATE::PERCHED:
        case STATE::UNPERCHED:
          return state;
        // Switch goals
        case STATE::RECOVERY_SWITCHING_TO_ML_LOC:
        case STATE::PERCHING_SWITCHING_TO_HR_LOC:
        case STATE::PERCHING_SWITCHING_TO_PL_LOC:
        case STATE::UNPERCHING_SWITCHING_TO_HR_LOC:
        case STATE::UNPERCHING_SWITCHING_TO_ML_LOC:
          client_s_.CancelGoal();
          return STATE::UNKNOWN;
          break;
        // Motion goals
        case STATE::RECOVERY_MOVING_TO_RECOVERY_POSE:
        case STATE::PERCHING_MOVING_TO_APPROACH_POSE:
        case STATE::PERCHING_ENSURING_APPROACH_POSE:
        case STATE::PERCHING_MOVING_TO_COMPLETE_POSE:
        case STATE::PERCHING_CHECKING_ATTACHED:
        case STATE::PERCHING_WAITING_FOR_SPIN_DOWN:
        case STATE::UNPERCHING_WAITING_FOR_SPIN_UP:
        case STATE::UNPERCHING_MOVING_TO_APPROACH_POSE:
          client_m_.CancelGoal();
          return STATE::UNKNOWN;
          break;
        // Arm goals
        case STATE::RECOVERY_STOWING_ARM:
        case STATE::RECOVERY_OPENING_GRIPPER:
        case STATE::PERCHING_DEPLOYING_ARM:
        case STATE::PERCHING_OPENING_GRIPPER:
        case STATE::PERCHING_CLOSING_GRIPPER:
        case STATE::UNPERCHING_OPENING_GRIPPER:
        case STATE::UNPERCHING_STOWING_ARM:
          client_a_.CancelGoal();
          return STATE::UNKNOWN;
          break;
        }
      });
  }
  ~PerchComponent() {}

 protected:
  // Called to initialize this component
  void Initialize(NodeHandle &nh) {
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.AddFile("behaviors/perch.config");
    if (!cfg_.Initialize(nh))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                         "Could not load config", GetTimeNow());

    // Setup the platform name
    platform_name_ = GetPlatform();
    platform_name_ = (platform_name_.empty() ? "" : platform_name_ + "/");

    // Set buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh->get_clock());

    // Create a transform buffer to listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(*tf_buffer_));

    // Publish the perching state as a latched topic
    pub_ = FF_CREATE_PUBLISHER(nh, ff_msgs::PerchState,
      TOPIC_BEHAVIORS_PERCHING_STATE, 1);

    // Subscribe to be notified when the perch state changes
    sub_ = FF_CREATE_SUBSCRIBER(nh, ff_msgs::ArmStateStamped, TOPIC_BEHAVIORS_ARM_ARM_STATE, 1,
      std::bind(&PerchComponent::ArmStateStampedCallback, this, std::placeholders::_1));

    // Allow the state to be manually set
    server_set_state_ = FF_CREATE_SERVICE(nh, ff_msgs::SetState, SERVICE_BEHAVIORS_PERCH_SET_STATE,
      std::bind(&PerchComponent::SetStateCallback, this, std::placeholders::_1, std::placeholders::_2));

    client_service_of_enable_.Create(nh, SERVICE_LOCALIZATION_OF_ENABLE);

    client_service_hr_reset_.Create(nh, SERVICE_GNC_EKF_RESET_HR);

    // Setup move client action
    client_m_.SetConnectedTimeout(cfg_.Get<double>("timeout_motion_connected"));
    client_m_.SetActiveTimeout(cfg_.Get<double>("timeout_motion_active"));
    client_m_.SetResponseTimeout(cfg_.Get<double>("timeout_motion_response"));
    client_m_.SetFeedbackCallback(std::bind(&PerchComponent::MFeedbackCallback,
      this, std::placeholders::_1));
    client_m_.SetResultCallback(std::bind(&PerchComponent::MResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_m_.SetConnectedCallback(std::bind(
      &PerchComponent::ConnectedCallback, this));
    client_m_.Create(nh, ACTION_MOBILITY_MOTION);

    // Setup switch client action
    client_s_.SetConnectedTimeout(cfg_.Get<double>("timeout_switch_connected"));
    client_s_.SetActiveTimeout(cfg_.Get<double>("timeout_switch_active"));
    client_s_.SetResponseTimeout(cfg_.Get<double>("timeout_switch_response"));
    // client_s_.SetDeadlineTimeout(cfg_.Get<double>("timeout_switch_deadline"));
    client_s_.SetFeedbackCallback(std::bind(
      &PerchComponent::SFeedbackCallback, this, std::placeholders::_1));
    client_s_.SetResultCallback(std::bind(&PerchComponent::SResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_s_.SetConnectedCallback(std::bind(&PerchComponent::ConnectedCallback,
      this));
    client_s_.Create(nh, ACTION_LOCALIZATION_MANAGER_LOCALIZATION);

    // Setup arm client action
    client_a_.SetConnectedTimeout(cfg_.Get<double>("timeout_arm_connected"));
    client_a_.SetActiveTimeout(cfg_.Get<double>("timeout_arm_active"));
    client_a_.SetResponseTimeout(cfg_.Get<double>("timeout_arm_response"));
    client_a_.SetFeedbackCallback(std::bind(
      &PerchComponent::AFeedbackCallback, this, std::placeholders::_1));
    client_a_.SetResultCallback(std::bind(&PerchComponent::AResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_a_.SetConnectedCallback(std::bind(&PerchComponent::ConnectedCallback,
      this));
    client_a_.Create(nh, ACTION_BEHAVIORS_ARM);

    // Setup the execute action
    server_.SetGoalCallback(std::bind(
      &PerchComponent::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &PerchComponent::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &PerchComponent::CancelCallback, this));
    server_.Create(nh, ACTION_BEHAVIORS_PERCH);
  }

  // Ensure all clients are connected
  void ConnectedCallback() {
    FF_DEBUG_STREAM("ConnectedCallback()");
    if (!client_a_.IsConnected()) return;       // Arm service
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

  // Complete the current perch or unperch action
  FSM::State Result(int32_t response, std::string const& msg = "") {
    // Send the feedback if needed
    switch (fsm_.GetState()) {
    case STATE::INITIALIZING:
    case STATE::UNKNOWN:
    case STATE::PERCHED:
    case STATE::UNPERCHED:
      return fsm_.GetState();
    default:
      break;
    }
    // Package up the feedback
    auto result = std::make_shared<ff_msgs::Perch::Result>();
    result->fsm_result = msg;
    result->response = response;
    if (response > 0)
      server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    else if (response < 0)
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
    else
      server_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
    // Always move to the unknown state and let arm feedback set it for you
    return STATE::UNKNOWN;
  }

  // When the FSM state changes we get a callback here, so that we
  // can choose to do various things.
  void UpdateCallback(FSM::State const& state, FSM::Event const& event) {
    // Debug events
    ff_msgs::PerchState msg;
    msg.header.frame_id = GetPlatform();
    msg.header.stamp = GetTimeNow();
    msg.state = state;
    // Debug events
    switch (event) {
    case READY:            msg.fsm_event = "READY";             break;
    case GOAL_PERCH:       msg.fsm_event = "GOAL_PERCH";        break;
    case GOAL_UNPERCH:     msg.fsm_event = "GOAL_UNPERCH";      break;
    case GOAL_CANCEL:      msg.fsm_event = "GOAL_CANCEL";       break;
    case GOAL_PREEMPT:     msg.fsm_event = "GOAL_PREEMPT";      break;
    case ARM_DEPLOYED:     msg.fsm_event = "ARM_DEPLOYED";      break;
    case ARM_STOWED:       msg.fsm_event = "ARM_STOWED";        break;
    case ARM_SUCCESS:      msg.fsm_event = "ARM_SUCCESS";       break;
    case ARM_FAILED:       msg.fsm_event = "ARM_FAILED";        break;
    case SWITCH_SUCCESS:   msg.fsm_event = "SWITCH_SUCCESS";    break;
    case SWITCH_FAILED:    msg.fsm_event = "SWITCH_FAILED";     break;
    case MOTION_SUCCESS:   msg.fsm_event = "MOTION_SUCCESS";    break;
    case MOTION_FAILED:    msg.fsm_event = "MOTION_FAILED";     break;
    case MANUAL_STATE_SET: msg.fsm_event = "MANUAL_STATE_SET";  break;
    }
    FF_DEBUG_STREAM("Received event " << msg.fsm_event);
    // Debug state changes
    switch (state) {
    case STATE::INITIALIZING:
      msg.fsm_state = "INITIALIZING";                         break;
    case STATE::UNKNOWN:
      msg.fsm_state = "UNKNOWN";                              break;
    case STATE::PERCHED:
      msg.fsm_state = "PERCHED";                              break;
    case STATE::UNPERCHED:
      msg.fsm_state = "UNPERCHED";                            break;
    case STATE::RECOVERY_SWITCHING_TO_ML_LOC :
      msg.fsm_state = "RECOVERY_SWITCHING_TO_ML_LOC";         break;
    case STATE::PERCHING_SWITCHING_TO_HR_LOC :
      msg.fsm_state = "PERCHING_SWITCHING_TO_HR_LOC";         break;
    case STATE::PERCHING_SWITCHING_TO_PL_LOC :
      msg.fsm_state = "PERCHING_SWITCHING_TO_PL_LOC";         break;
    case STATE::UNPERCHING_SWITCHING_TO_HR_LOC :
      msg.fsm_state = "UNPERCHING_SWITCHING_TO_HR_LOC";       break;
    case STATE::UNPERCHING_SWITCHING_TO_ML_LOC :
      msg.fsm_state = "UNPERCHING_SWITCHING_TO_ML_LOC";       break;
    case STATE::RECOVERY_MOVING_TO_RECOVERY_POSE :
      msg.fsm_state = "RECOVERY_MOVING_TO_RECOVERY_POSE";     break;
    case STATE::PERCHING_MOVING_TO_APPROACH_POSE :
      msg.fsm_state = "PERCHING_MOVING_TO_APPROACH_POSE";     break;
    case STATE::PERCHING_ENSURING_APPROACH_POSE :
      msg.fsm_state = "PERCHING_ENSURING_APPROACH_POSE";      break;
    case STATE::PERCHING_MOVING_TO_COMPLETE_POSE :
      msg.fsm_state = "PERCHING_MOVING_TO_COMPLETE_POSE";     break;
    case STATE::PERCHING_CHECKING_ATTACHED :
      msg.fsm_state = "PERCHING_CHECKING_ATTACHED";           break;
    case STATE::PERCHING_WAITING_FOR_SPIN_DOWN :
      msg.fsm_state = "PERCHING_WAITING_FOR_SPIN_DOWN";       break;
    case STATE::UNPERCHING_WAITING_FOR_SPIN_UP :
      msg.fsm_state = "UNPERCHING_WAITING_FOR_SPIN_UP";       break;
    case STATE::UNPERCHING_MOVING_TO_APPROACH_POSE :
      msg.fsm_state = "UNPERCHING_MOVING_TO_APPROACH_POSE";   break;
    case STATE::RECOVERY_STOWING_ARM :
      msg.fsm_state = "RECOVERY_STOWING_ARM";                 break;
    case STATE::RECOVERY_OPENING_GRIPPER :
      msg.fsm_state = "RECOVERY_OPENING_GRIPPER";             break;
    case STATE::PERCHING_DEPLOYING_ARM :
      msg.fsm_state = "PERCHING_DEPLOYING_ARM";               break;
    case STATE::PERCHING_OPENING_GRIPPER :
      msg.fsm_state = "PERCHING_OPENING_GRIPPER";             break;
    case STATE::PERCHING_CLOSING_GRIPPER :
      msg.fsm_state = "PERCHING_CLOSING_GRIPPER";             break;
    case STATE::UNPERCHING_OPENING_GRIPPER :
      msg.fsm_state = "UNPERCHING_OPENING_GRIPPER";           break;
    case STATE::UNPERCHING_STOWING_ARM :
      msg.fsm_state = "UNPERCHING_STOWING_ARM";               break;
    }
    FF_DEBUG_STREAM("State changed to " << msg.fsm_state);
    // Broadcast the perching state
    pub_->publish(msg);
    // Send the feedback if needed
    switch (state) {
    case STATE::INITIALIZING:
    case STATE::UNKNOWN:
    case STATE::PERCHED:
    case STATE::UNPERCHED:
      break;
    default:
      {
        auto feedback = std::make_shared<ff_msgs::Perch::Feedback>();
        feedback->state = msg;
        server_.SendFeedback(feedback);
      }
    }
  }

  // SWITCH action

  // Helper function for localization switching
  bool Switch(std::string const& pipeline) {
    // Send the switch goal
    ff_msgs::Localization::Goal goal;
    goal.command = ff_msgs::Localization::Goal::COMMAND_SWITCH_PIPELINE;
    goal.pipeline = pipeline;
    FF_WARN("Asking for switch");
    return client_s_.SendGoal(goal);
  }

  // Ignore the switch feedback for now
  void SFeedbackCallback(const std::shared_ptr<const ff_msgs::Localization::Feedback> feedback) {}

  // Do something with the switch result
  void SResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    std::shared_ptr<const ff_msgs::Localization::Result> result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      FF_WARN("Switch Success");
      return fsm_.Update(SWITCH_SUCCESS);
    default:
      FF_WARN("Switch Failed");
      return fsm_.Update(SWITCH_FAILED);
    }
  }

  // ARM ACTION CLIENT

  // Asynchronous arm state callback
  void ArmStateStampedCallback(const std::shared_ptr<ff_msgs::ArmStateStamped> msg) {
    switch (msg->joint_state.state) {
    case ff_msgs::ArmJointState::STOWED:
      return fsm_.Update(ARM_STOWED);
    case ff_msgs::ArmJointState::DEPLOYING:
    case ff_msgs::ArmJointState::STOPPED:
    case ff_msgs::ArmJointState::MOVING:
    case ff_msgs::ArmJointState::STOWING:
      return fsm_.Update(ARM_DEPLOYED);
    case ff_msgs::ArmJointState::UNKNOWN:
    default:
      break;
    }
  }

  // Send a move command to the arm.
  bool Arm(uint8_t command) {
    ff_msgs::Arm::Goal goal;
    goal.command = command;
    return client_a_.SendGoal(goal);
  }

  // Ignore the move feedback, for now
  void AFeedbackCallback(const std::shared_ptr<const ff_msgs::Arm::Feedback> feedback) {}

  // Result of a move action
  void AResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    std::shared_ptr<const ff_msgs::Arm::Result> result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return fsm_.Update(ARM_SUCCESS);
    default:
      return fsm_.Update(ARM_FAILED);
    }
  }

  // MOTION ACTION CLIENT

  /* Save approach pose to not have to recalculate handrail/approach
   * to world transform. This only happens when reaching with
   * success the approach pose via handrail localization. It is only
   * called in the perching step, and never in the unperching.
   */
  void SaveApproachPose(void) {
    geometry_msgs::TransformStamped tf = tf_buffer_->lookupTransform(
     "world", platform_name_ + "body", tf2::TimePointZero);

    FF_WARN("[Perch] Saving Approach Pose");

    // Save the transform
    approach_pose_ = tf.transform;
  }

  // Enable or disable optical flow
  bool OpticalFlow(bool enable) {
    ff_msgs::SetBool::Request request;
    auto response = std::make_shared<ff_msgs::SetBool::Response>();
    request.enable = enable;
    return client_service_of_enable_.call(request, response);
  }

  // Reset the position of the handrail in the world.
  bool HandraiLReset(void) {
    std_srvs::Empty::Request request;
    auto response = std::make_shared<std_srvs::Empty::Response>();
    return client_service_hr_reset_.call(request, response);
  }

  // Prepare for a motion
  bool Prep(std::string const& flight_mode) {
    static ff_msgs::Motion::Goal goal;
    goal.command = ff_msgs::Motion::Goal::PREP;
    goal.flight_mode = flight_mode;
    return client_m_.SendGoal(goal);
  }

  // Send a move command
  bool Move(PerchPose type, std::string const& mode) {
    // Create a new motion foal
    ff_msgs::Motion::Goal goal;
    goal.command = ff_msgs::Motion::Goal::MOVE;
    goal.flight_mode = mode;

    // Package up the desired end pose
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = GetTimeNow();
    switch (type) {
      // Move to approach pose again, for robustness of trajectory,
      // then to the complete pose.
      case COMPLETE_POSE:
        // The movement to approach pose is already done twice
        msg.header.frame_id = platform_name_ + "handrail/approach";
        goal.states.push_back(msg);
        msg.header.frame_id = platform_name_ + "handrail/complete";
        goal.states.push_back(msg);
        break;
      // Move to the approach pose.
      case APPROACH_POSE:
        msg.header.frame_id = platform_name_ + "handrail/approach";
        goal.states.push_back(msg);
        break;
      // Move to the recovery pose. This option is currently used to move to the
      // approach pose, but it is here for potential change in recovery options.
      case RECOVERY_POSE:
        msg.header.frame_id = platform_name_ + "body";
        goal.states.push_back(msg);
        break;
      // Move to the perched pose. This option is currently used as a placeholder
      // operation for perched localization. It is implemented for futrure use,
      // if the user wants the robot to have a pan and tilt position for example.
      case PERCHED_POSE:
        msg.header.frame_id = platform_name_ + "body";
        goal.states.push_back(msg);
        break;
      // Move to the unperched pose. Placeholder to return to a default pan/tilt
      // option, to make the aft face parallel to the wall supporting the handrail,
      // for example.
      case UNPERCHED_POSE:
        msg.header.frame_id = platform_name_ + "body";
        goal.states.push_back(msg);
        break;
      default:
        return false;
    }

    // Iterate over all poses in action, finding the location of each
    for (auto & pose : goal.states) {
      try {
        // Look up the world -> body transform
        if (type == RECOVERY_POSE || type == UNPERCHED_POSE) {
        /* 
         * If we call move with RECOVERY_POSE, no need to check a transform.
         * We only need to ask a movement to the pose saved in the 
         * void SaveApproachPose(void) function.
         * For the moment, apply same reasoning when unperching.
         * This is due to the fact that we may be too close to the handrail to
         * see it, and hence to localize. However, I (Julien) believe it would
         * be more robust to move relatively forward a little (10-15 cm), by
         * having a perched localization which would place the Astrobee's
         * aft face parallel to the wall supporting the handrail prior to movement.
         * Or just move relatively forward away from the handrail, swap to HR loc,
         * move to approach pose, swap to ML loc.
         * However, as long as the approach pose is saved correctly, this works.
         * Need to be careful with the arm state and robot orientation!
         * 
         * TL;DR: This is good if we don't lose localization while perched,
         * otherwise relative movements are needed to securely move away.
         */
          pose.pose = msg_conversions::ros_transform_to_ros_pose(approach_pose_);

        } else {
          /*
           * Any other movement implies a copy of a transform
           * with the world.
           */
          geometry_msgs::TransformStamped tf = tf_buffer_->lookupTransform(
          "world", pose.header.frame_id, tf2::TimePointZero);

          // Copy the transform
          pose.pose = msg_conversions::ros_transform_to_ros_pose(tf.transform);
        }
      } catch (const tf2::TransformException &ex) {
        FF_WARN_STREAM("Transform failed" << ex.what());
        return false;
      }
    }


    // Reconfigure the choreographer
    ff_util::ConfigClient cfg(node_, NODE_CHOREOGRAPHER);
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
    if (!cfg.Reconfigure())
      return false;

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

  // PERCH ACTION SERVER

  // A new arm action has been called
  void GoalCallback(std::shared_ptr<const ff_msgs::Perch::Goal> goal) {
    auto result = std::make_shared<ff_msgs::Perch::Result>();
    switch (goal->command) {
    case ff_msgs::Perch::Goal::PERCH:
      // We are unperched
      if (fsm_.GetState() == STATE::UNPERCHED) {
        // Do we know about the specified handrail?
        return fsm_.Update(GOAL_PERCH);
      // We are already perched
      } else if (fsm_.GetState() == STATE::PERCHED) {
        result->fsm_result = "Currently perched, so ignoring perch request.";
        result->response = RESPONSE::ALREADY_PERCHED;
        server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
        return;
      // We are in initialize or unknown
      } else {
        result->fsm_result = "Perch state unknown, waiting for arm callback.";
        result->response = RESPONSE::NOT_IN_UNPERCHED_STATE;
        server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
        return;
      }
      break;
    // Unperch command
    case ff_msgs::Perch::Goal::UNPERCH:
      // We are perched
      if (fsm_.GetState() == STATE::PERCHED) {
        return fsm_.Update(GOAL_UNPERCH);
      // We are already unperched
      } else if (fsm_.GetState() == STATE::UNPERCHED) {
        result->fsm_result = "Currently unperched, so ignoring unperch request.";
        result->response = RESPONSE::ALREADY_UNPERCHED;
        server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
        return;
      // We are in initialize or unknown
      } else {
        result->fsm_result = "Perch state unknown, waiting for arm callback.";
        result->response = RESPONSE::NOT_IN_PERCHED_STATE;
      }
      break;
    // Invalid command
    default:
      result->fsm_result = "Invalid command in request.";
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
  ff_util::FSM fsm_;
  ff_util::FreeFlyerActionClient<ff_msgs::Motion> client_m_;
  ff_util::FreeFlyerActionClient<ff_msgs::Localization> client_s_;
  ff_util::FreeFlyerActionClient<ff_msgs::Arm> client_a_;
  ff_util::FreeFlyerActionServer<ff_msgs::Perch> server_;
  ff_util::ConfigServer cfg_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<ff_msgs::PerchState>::SharedPtr pub_;
  rclcpp::Subscription<ff_msgs::ArmStateStamped>::SharedPtr sub_;
  rclcpp::Service<ff_msgs::SetState>::SharedPtr server_set_state_;
  ff_util::FreeFlyerServiceClient<ff_msgs::SetBool> client_service_of_enable_;
  ff_util::FreeFlyerServiceClient<std_srvs::Empty> client_service_hr_reset_;
  geometry_msgs::Transform approach_pose_;
  int32_t err_;
  std::string err_msg_;
  std::string platform_name_;

 public:
  // This fixes the Eigen alignment issue
  // http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace perch

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(perch::PerchComponent)
