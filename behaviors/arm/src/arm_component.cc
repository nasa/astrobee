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

// Standard sensor messages
#include <sensor_msgs/msg/joint_state.hpp>
namespace sensor_msgs {
typedef msg::JointState JointState;
}  // namespace sensor_msgs

// FSW shared libraries
#include <config_reader/config_reader.h>
#include <ff_util/config_server.h>
#include <ff_util/ff_component.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_fsm.h>
#include <ff_util/ff_service.h>

// FSW actions, services, messages
#include <ff_msgs/action/arm.hpp>
#include <ff_msgs/msg/arm_state.hpp>
#include <ff_msgs/msg/arm_state_stamped.hpp>
#include <ff_msgs/msg/joint_sample.hpp>
#include <ff_msgs/msg/joint_sample_stamped.hpp>
#include <ff_msgs/msg/arm_joint_state.hpp>
#include <ff_msgs/msg/arm_gripper_state.hpp>
#include <ff_msgs/srv/set_state.hpp>
namespace ff_msgs {
typedef action::Arm Arm;
typedef msg::ArmState ArmState;
typedef msg::ArmStateStamped ArmStateStamped;
typedef msg::JointSample JointSample;
typedef msg::JointSampleStamped JointSampleStamped;
typedef msg::ArmJointState ArmJointState;
typedef msg::ArmGripperState ArmGripperState;
typedef srv::SetState SetState;
}  // namespace ff_msgs

#include <ff_hw_msgs/srv/set_enabled.hpp>
#include <ff_hw_msgs/srv/calibrate_gripper.hpp>
namespace ff_hw_msgs {
typedef srv::SetEnabled SetEnabled;
typedef srv::CalibrateGripper CalibrateGripper;
}  // namespace ff_hw_msgs

/**
 * \ingroup beh
 */
namespace arm {

FF_DEFINE_LOGGER("arm");

// Different joint types
enum JointType { PAN, TILT, GRIPPER };

// Perching arm servos to enable / disable
enum ServoID {
  PROXIMAL_SERVO,           // Proximal joint servo
  DISTAL_SERVO,             // Distal joint servo
  GRIPPER_SERVO,            // Gripper joint servo
  ALL_SERVOS          // Proximal, Distal and Gripper servos
};

// Joint information, where HUMAN = SCALE * DRIVER + OFFSET
struct JointInfo {
  std::string name;     // Low level joint state name
  std::string generic;  // Generic name for joint state
  double val;           // Current value in HUMAN form
  double goal;          // Current goal in HUMAN form
  double tol;           // Tolerance in HUMAN form
  double offset;        // DRIVER -> HUMAN offset
  double scale;         // FRIVER -> HUMAN SCALE
};

// List of generic joints "pan", "tilt" and "gripper"
typedef std::map<JointType, JointInfo> JointMap;

// Reverse lookup for joint name -> generic joint name
typedef std::map<std::string, JointType> JointDictionary;

// Match the internal states and responses with the message definition
using FSM = ff_util::FSM;
using STATE = ff_msgs::ArmState;
using RESPONSE = ff_msgs::Arm::Result;

class ArmComponent : public ff_util::FreeFlyerComponent {
 public:
  // Possible events
  enum : FSM::Event {
    READY              = (1<<0),     // We are connected to the arm
    DEPLOYED           = (1<<1),     // Background deploy
    STOWED             = (1<<2),     // Background stow
    GOAL_DEPLOY        = (1<<3),     // Start a new deploy action
    GOAL_STOW          = (1<<4),     // Start a new deploy action
    GOAL_MOVE          = (1<<5),     // Start a new move (pan and tilt)
    GOAL_SET           = (1<<6),     // Start a new gripper action
    GOAL_DISABLE       = (1<<7),     // Disable the servos
    GOAL_CANCEL        = (1<<8),     // Cancel the current goal
    PAN_COMPLETE       = (1<<9),     // Pan complete
    TILT_COMPLETE      = (1<<10),    // Tilt complete
    GRIPPER_COMPLETE   = (1<<11),    // Gripper action complete
    TIMEOUT            = (1<<12),    // Current action doesn't complete in time
    MANUAL_STATE_SET   = (1<<13)     // Setting the state manually with service
  };

  // Constructor
  explicit ArmComponent(const rclcpp::NodeOptions& options) : ff_util::FreeFlyerComponent(options, NODE_ARM, true),
    fsm_(STATE::INITIALIZING, std::bind(&ArmComponent::UpdateCallback,
      this, std::placeholders::_1, std::placeholders::_2)) {
    // INITIALIZING -> UNKNOWN
    //   [label="[0]\nREADY", color=blue];
    fsm_.Add(STATE::INITIALIZING,
      READY,
      [this](FSM::Event const& event) -> FSM::State {
        FF_DEBUG("[Arm] INITIALIZING, Enabling servos");
        if (!EnableServo(ALL_SERVOS, true))
          return Result(RESPONSE::ENABLE_FAILED);
        return STATE::UNKNOWN;
      });

    // UNKNOWN -> STOWED
    //   [label="[1]\nARM_STOWED", color=blue];
    fsm_.Add(STATE::UNKNOWN,
      STOWED,
      [this](FSM::Event const& event) -> FSM::State {
        FF_DEBUG("[Arm] STOWED STATE");
        return STATE::STOWED;
      });

    // UNKNOWN -> DEPLOYED
    //   [label="[2]\nARM_DEPLOYED", color=blue];
    fsm_.Add(STATE::UNKNOWN,
      DEPLOYED,
      [this](FSM::Event const& event) -> FSM::State {
        FF_DEBUG("[Arm] DEPLOYED STATE");
        return STATE::DEPLOYED;
      });

    // STOWED -> DEPLOYED
    //   [label="[3]\nARM_DEPLOYED", color=blue];
    fsm_.Add(STATE::STOWED,
      DEPLOYED,
      [this](FSM::Event const& event) -> FSM::State {
        return STATE::DEPLOYED;
      });

    fsm_.Add(STATE::STOWED,
      GOAL_DISABLE,
      [this](FSM::Event const& event) -> FSM::State {
        if (!EnableServo(ALL_SERVOS, false))
          return Result(RESPONSE::DISABLE_FAILED);
        return Result(RESPONSE::SUCCESS);
      });

    // DEPLOYED -> STOWED
    //   [label="[4]\nARM_STOWED", color=blue];
    fsm_.Add(STATE::DEPLOYED,
      STOWED,
      [this](FSM::Event const& event) -> FSM::State {
        return STATE::STOWED;
      });

    // STOWED -> DEPLOYING_PANNING
    //   [label="[5]\nGOAL_DEPLOY\nPan(DEPLOY)"];
    fsm_.Add(GOAL_DEPLOY,
      [this](FSM::State const& state, FSM::Event const& event) -> FSM::State {
        if (!Arm(PAN))
          return Result(RESPONSE::PAN_FAILED);
        return STATE::DEPLOYING_PANNING;
      });

    // DEPLOYING_PANNING -> DEPLOYING_TILTING
    //   [label="[6]\nPAN_COMPLETE\nTilt(DEPLOY)"];
    fsm_.Add(STATE::DEPLOYING_PANNING,
      PAN_COMPLETE,
      [this](FSM::Event const& event) -> FSM::State {
        if (!Arm(TILT))
          return Result(RESPONSE::TILT_FAILED);
        return STATE::DEPLOYING_TILTING;
      });

    // DEPLOYING_PANNING -> DEPLOYED
    //   [label="[10]\nTIMEOUT\nResult(FAILED)", color=red];
    fsm_.Add(STATE::DEPLOYING_PANNING,
      GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        Arm(PAN);
        return STATE::PANNING;
      });

    // DEPLOYING_PANNING -> DEPLOYED
    //   [label="[10]\nTIMEOUT\nResult(FAILED)", color=red];
    fsm_.Add(STATE::DEPLOYING_PANNING,
      TIMEOUT,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::PAN_FAILED);
        return STATE::UNKNOWN;
      });

    // DEPLOYING_TILTING -> DEPLOYED
    //   [label="[7]\nTILT_COMPLETE\nResult(SUCCESS)", color=darkgreen];
    fsm_.Add(STATE::DEPLOYING_TILTING,
      TILT_COMPLETE,
      [this](FSM::Event const& event) -> FSM::State {
        if (!calibrated_) {
           if (!CalibrateGripper())
            return Result(RESPONSE::CALIBRATE_FAILED);
          return STATE::CALIBRATING;
        }
        return Result(RESPONSE::SUCCESS);
      });

    // DEPLOYING_TILTING -> DEPLOYED
    //   [label="[10]\nTIMEOUT\nResult(FAILED)", color=red];
    fsm_.Add(STATE::DEPLOYING_TILTING,
      GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        Arm(TILT);
        return STATE::TILTING;
      });

    // DEPLOYING_TILTING -> DEPLOYED
    //   [label="[10]\nTIMEOUT\nResult(FAILED)", color=red];
    fsm_.Add(STATE::DEPLOYING_TILTING,
      TIMEOUT,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::TILT_FAILED);
        return STATE::UNKNOWN;
      });

    // DEPLOYED -> STOWING_SETTING
    //   [label="[8]\nGOAL_STOW\nGripper(CLOSE)"];
    fsm_.Add(STATE::DEPLOYED,
      GOAL_STOW,
      [this](FSM::Event const& event) -> FSM::State {
        goal_stow_ = true;
        goal_set_ = false;
        if (!calibrated_) {
           if (!CalibrateGripper())
            return Result(RESPONSE::CALIBRATE_FAILED);
          return STATE::CALIBRATING;
        } else {
          if (RequiresClosing()) {
            if (!Arm(GRIPPER))
              return Result(RESPONSE::GRIPPER_FAILED);
            return STATE::STOWING_SETTING;
          }
          if (!Arm(PAN))
            return Result(RESPONSE::PAN_FAILED);
          return STATE::STOWING_PANNING;
        }
      });

    fsm_.Add(STATE::CALIBRATING,
      GRIPPER_COMPLETE,
      [this](FSM::Event const& event) -> FSM::State {
        if (RequiresClosing()) {
          if (!Arm(GRIPPER)) {
            return Result(RESPONSE::GRIPPER_FAILED);
          }
          // Check which step we were at when asking for calibration
          if (goal_stow_)
            return STATE::STOWING_SETTING;
          else if (goal_set_)
            return STATE::SETTING;
          else   // Finished
            return Result(RESPONSE::SUCCESS);
        }
        if (!Arm(PAN))
          return Result(RESPONSE::PAN_FAILED);
        return STATE::STOWING_PANNING;
      });

    // STOWING_SETTING -> STOWING_PANNING
    //   [label="[9]\nGRIPPER_COMPLETE\nPan(STOWED)", color=black];
    fsm_.Add(STATE::STOWING_SETTING,
      GRIPPER_COMPLETE,
      [this](FSM::Event const& event) -> FSM::State {
        if (!Arm(PAN))
          return Result(RESPONSE::PAN_FAILED);
        return STATE::STOWING_PANNING;
      });

    // STOWING_SETTING -> DEPLOYED
    //   [label="[10]\nTIMEOUT\nResult(FAILED)", color=red];
    fsm_.Add(STATE::STOWING_SETTING,
      GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        Arm(GRIPPER);
        return STATE::SETTING;
      });

    // STOWING_SETTING -> DEPLOYED
    //   [label="[10]\nTIMEOUT\nResult(FAILED)", color=red];
    fsm_.Add(STATE::STOWING_SETTING,
      TIMEOUT,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::GRIPPER_FAILED);
        return STATE::UNKNOWN;
      });

    // STOWING_PANNING -> STOWING_TILTING
    //   [label="[11]\nPAN_COMPLETE\nTilt(STOWED)", color=black];
    fsm_.Add(STATE::STOWING_PANNING,
      PAN_COMPLETE,
      [this](FSM::Event const& event) -> FSM::State {
        if (!Arm(TILT))
          return Result(RESPONSE::TILT_FAILED);
        return STATE::STOWING_TILTING;
      });

    // STOWING_PANNING -> DEPLOYED
    //   [label="[12]\nTIMEOUT\nResult(FAILED)", color=red];
    fsm_.Add(STATE::STOWING_PANNING,
      GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        Arm(PAN);
        return STATE::PANNING;
      });

    // STOWING_PANNING -> DEPLOYED
    //   [label="[12]\nTIMEOUT\nResult(FAILED)", color=red];
    fsm_.Add(STATE::STOWING_PANNING,
      TIMEOUT,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::PAN_FAILED);
        return STATE::UNKNOWN;
      });

    // STOWING_TILTING -> STOWED
    //   [label="[13]\nTILT_COMPLETE\nResult(SUCCESS)", color=darkgreen];
    fsm_.Add(STATE::STOWING_TILTING,
      TILT_COMPLETE,
      [this](FSM::Event const& event) -> FSM::State {
        return Result(RESPONSE::SUCCESS);
      });

    // STOWING_TILTING -> DEPLOYED
    //   [label="[14]\nTIMEOUT\nResult(FAILED)", color=red];
    fsm_.Add(STATE::STOWING_TILTING,
      GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        Arm(TILT);
        return STATE::TILTING;
      });

    // STOWING_TILTING -> DEPLOYED
    //   [label="[14]\nTIMEOUT\nResult(FAILED)", color=red];
    fsm_.Add(STATE::STOWING_TILTING,
      TIMEOUT,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::TILT_FAILED);
        return STATE::UNKNOWN;
      });

    // {STOWED, DEPLOYED} -> PANNING
    //   [label="[15]\nGOAL_MOVE\nPan(angle)"];
    fsm_.Add(STATE::STOWED, STATE::DEPLOYED,
      GOAL_MOVE,
      [this](FSM::Event const& event) -> FSM::State {
        // Check that we have a valid pan value
        if (!Arm(PAN))
          return Result(RESPONSE::PAN_FAILED);
        return STATE::PANNING;
      });

    // PANNING -> TILTING
    //   [label="[16]\nPAN_COMPLETE\nTilt(angle)"];
    fsm_.Add(STATE::PANNING,
      PAN_COMPLETE,
      [this](FSM::Event const& event) -> FSM::State {
        // Check that we have a valid tilt value
        if (!Arm(TILT))
          return Result(RESPONSE::TILT_FAILED);
        return STATE::TILTING;
      });

    // PANNING -> DEPLOYED
    //   [label="[17]\nTIMEOUT\nResult(PAN_FAILED)", color=red];
    fsm_.Add(STATE::PANNING,
      GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        Arm(PAN);
        return STATE::PANNING;
      });

    // PANNING -> DEPLOYED
    //   [label="[17]\nTIMEOUT\nResult(PAN_FAILED)", color=red];
    fsm_.Add(STATE::PANNING,
      TIMEOUT,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::PAN_FAILED);
        return STATE::UNKNOWN;
      });

    // TILTING -> DEPLOYED
    //   [label="[18]\nTILT_COMPLETE\nResult(SUCCESS)", color=darkgreen];
    fsm_.Add(STATE::TILTING,
      TILT_COMPLETE,
      [this](FSM::Event const& event) -> FSM::State {
        return Result(RESPONSE::SUCCESS);
      });

    // TILTING -> DEPLOYED
    //   [label="[19]\nTIMEOUT\nResult(TILT_FAILED)", color=red];
    fsm_.Add(STATE::TILTING,
      GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        Arm(TILT);
        return STATE::TILTING;
      });

    // TILTING -> DEPLOYED
    //   [label="[19]\nTIMEOUT\nResult(TILT_FAILED)", color=red];
    fsm_.Add(STATE::TILTING,
      TIMEOUT,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::TILT_FAILED);
        return STATE::UNKNOWN;
      });

    // DEPLOYED -> SETTING
    //   [label="[20]\nGOAL_SET\nGripper(percent)"];
    fsm_.Add(STATE::DEPLOYED,
      GOAL_SET,
      [this](FSM::Event const& event) -> FSM::State {
        goal_stow_ = false;
        goal_set_ = true;
        if (!calibrated_) {
           if (!CalibrateGripper())
            return Result(RESPONSE::CALIBRATE_FAILED);
          return STATE::CALIBRATING;
        } else {
          if (!Arm(GRIPPER))
            return Result(RESPONSE::GRIPPER_FAILED);
          return STATE::SETTING;
        }
      });

    // SETTING -> DEPLOYED
    //   [label="[21]\nGRIPPER_COMPLETE\nResult(SUCCESS)", color=darkgreen];
    fsm_.Add(STATE::SETTING,
      GRIPPER_COMPLETE,
      [this](FSM::Event const& event) -> FSM::State {
        return Result(RESPONSE::SUCCESS);
      });

    // SETTING -> DEPLOYED
    //   [label="[22]\nTIMEOUT\nResult(GRIPPER_FAILED)", color=red];
    fsm_.Add(STATE::SETTING,
      GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        Arm(GRIPPER);
        return STATE::SETTING;
      });

    // SETTING -> DEPLOYED
    //   [label="[22]\nTIMEOUT\nResult(GRIPPER_FAILED)", color=red];
    fsm_.Add(STATE::SETTING,
      TIMEOUT,
      [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::GRIPPER_FAILED);
        return STATE::UNKNOWN;
      });
  }

  // Destructor
  ~ArmComponent() {}

 protected:
  // Called to initialize this nodelet
  void Initialize(NodeHandle &nh) {
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.AddFile("behaviors/arm.config");
    if (!cfg_.Initialize(nh))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                  "Could not start config server", GetTimeNow());

    // Read the configuration for this specific node
    config_reader::ConfigReader *cfg = cfg_.GetConfigReader();
    config_reader::ConfigReader::Table joints;
    if (!cfg->GetTable(GetName().c_str(), &joints))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                         "Cannot read LUA file", GetTimeNow());
    std::string name;
    if (!joints.GetStr("pan", &name))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                         "Cannot read PAN joint", GetTimeNow());
    joints_[PAN].name = name;
    joints_[PAN].generic = "pan";
    joints_[PAN].tol = cfg_.Get<double>("tol_pan");
    joints_[PAN].scale = K_RADS_TO_DEGS;
    joints_[PAN].offset = K_PAN_OFFSET;
    dictionary_[name] = PAN;
    if (!joints.GetStr("tilt", &name))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                         "Cannot read TILT joint", GetTimeNow());
    joints_[TILT].name = name;
    joints_[TILT].generic = "tilt";
    joints_[TILT].tol = cfg_.Get<double>("tol_tilt");
    joints_[TILT].scale = K_RADS_TO_DEGS;
    joints_[TILT].offset = K_TILT_OFFSET;
    dictionary_[name] = TILT;
    if (!joints.GetStr("gripper", &name))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                         "Cannot read GRIPPER joint", GetTimeNow());
    joints_[GRIPPER].name = name;
    joints_[GRIPPER].generic = "gripper";
    joints_[GRIPPER].tol = cfg_.Get<double>("tol_gripper");
    joints_[GRIPPER].scale = 1.00;
    joints_[GRIPPER].offset = 0.00;
    dictionary_[name] = GRIPPER;

    // Timer for checking goal is reached
    timer_watchdog_.createTimer(cfg_.Get<double>("timeout_watchdog"),
        std::bind(&ArmComponent::WatchdogCallback, this), nh, true, false);

    // Timer for checking goal is reached
    timer_goal_.createTimer(cfg_.Get<double>("timeout_goal"),
        std::bind(&ArmComponent::TimeoutCallback, this), nh, true, false);

    // Publishers for arm and joint state
    sub_joint_states_ = FF_CREATE_SUBSCRIBER(nh, sensor_msgs::JointState, TOPIC_JOINT_STATES, 1,
      std::bind(&ArmComponent::JointStateCallback, this, std::placeholders::_1));
    pub_joint_goals_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::JointState, TOPIC_JOINT_GOALS, 1);

    // Subscribe to Proximal Joint Servo Enabling service
    client_enable_prox_servo_.Create(nh, SERVICE_HARDWARE_PERCHING_ARM_PROX_SERVO);

    // Subscribe to Distal Joint Servo Enabling service
    client_enable_dist_servo_.Create(nh, SERVICE_HARDWARE_PERCHING_ARM_DIST_SERVO);

    // Subscribe to Gripper Joint Servo Enabling service
    client_enable_grip_servo_.Create(nh, SERVICE_HARDWARE_PERCHING_ARM_GRIP_SERVO);

    // Subscribe to Proximal Joint Servo Enabling service
    client_calibrate_gripper_.Create(nh, SERVICE_HARDWARE_PERCHING_ARM_CALIBRATE);

    // Internal state publisher
    pub_state_ = FF_CREATE_PUBLISHER(nh, ff_msgs::ArmState,
      TOPIC_BEHAVIORS_ARM_STATE, 1);

    // Allow the state to be manually set
    srv_set_state_ = FF_CREATE_SERVICE(nh, ff_msgs::SetState, SERVICE_BEHAVIORS_ARM_SET_STATE,
      std::bind(&ArmComponent::SetStateCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Executive state publishers
    pub_arm_state_ = FF_CREATE_PUBLISHER(nh, ff_msgs::ArmStateStamped, TOPIC_BEHAVIORS_ARM_ARM_STATE, 1);
    pub_joint_sample_ = FF_CREATE_PUBLISHER(nh, ff_msgs::JointSampleStamped, TOPIC_BEHAVIORS_ARM_JOINT_SAMPLE, 1);

    // Setup the ARM  action
    server_.SetGoalCallback(std::bind(
      &ArmComponent::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &ArmComponent::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &ArmComponent::CancelCallback, this));
    server_.Create(nh, ACTION_BEHAVIORS_ARM);
  }

  // When the FSM state changes we get a callback here, so that we
  // can choose to do various things.
  void UpdateCallback(FSM::State const& state, FSM::Event const& event) {
    // Debug events
    ff_msgs::ArmState msg;
    msg.header.frame_id = GetPlatform();
    msg.header.stamp = GetTimeNow();
    msg.state = state;
    switch (event) {
    case READY:                    msg.fsm_event = "READY";               break;
    case DEPLOYED:                 msg.fsm_event = "DEPLOYED";            break;
    case STOWED:                   msg.fsm_event = "STOWED";              break;
    case GOAL_DEPLOY:              msg.fsm_event = "GOAL_DEPLOY";         break;
    case GOAL_STOW:                msg.fsm_event = "GOAL_STOW";           break;
    case GOAL_MOVE:                msg.fsm_event = "GOAL_MOVE";           break;
    case GOAL_SET:                 msg.fsm_event = "GOAL_SET";            break;
    case GOAL_CANCEL:              msg.fsm_event = "GOAL_CANCEL";         break;
    case PAN_COMPLETE:             msg.fsm_event = "PAN_COMPLETE";        break;
    case TILT_COMPLETE:            msg.fsm_event = "TILT_COMPLETE";       break;
    case GRIPPER_COMPLETE:         msg.fsm_event = "GRIPPER_COMPLETE";    break;
    case TIMEOUT:                  msg.fsm_event = "TIMEOUT";             break;
    case MANUAL_STATE_SET:         msg.fsm_event = "MANUAL_STATE_SET";    break;
    }
    // Debug state changes
    switch (state) {
    case STATE::INITIALIZING:      msg.fsm_state = "INITIALIZING";        break;
    case STATE::UNKNOWN:           msg.fsm_state = "UNKNOWN";             break;
    case STATE::STOWED:            msg.fsm_state = "STOWED";              break;
    case STATE::DEPLOYED:          msg.fsm_state = "DEPLOYED";            break;
    case STATE::PANNING:           msg.fsm_state = "PANNING";             break;
    case STATE::TILTING:           msg.fsm_state = "TILTING";             break;
    case STATE::SETTING:           msg.fsm_state = "SETTING";             break;
    case STATE::STOWING_SETTING:   msg.fsm_state = "STOWING_SETTING";     break;
    case STATE::STOWING_PANNING:   msg.fsm_state = "STOWING_PANNING";     break;
    case STATE::STOWING_TILTING:   msg.fsm_state = "STOWING_TILTING";     break;
    case STATE::DEPLOYING_PANNING: msg.fsm_state = "DEPLOYING_PANNING";   break;
    case STATE::DEPLOYING_TILTING: msg.fsm_state = "DEPLOYING_TILTING";   break;
    case STATE::CALIBRATING:       msg.fsm_state = "CALIBRATING";         break;
    }
    // Publish the state
    pub_state_->publish(msg);
    FF_DEBUG_STREAM("Received event " << msg.fsm_event);
    FF_DEBUG_STREAM("State changed to " << msg.fsm_state);
  }

  // Complete the current action
  int32_t Result(int32_t response, bool send = false) {
    // Write the current values to the joint states to bring the whole system
    // to a halt. We don't want any movement here. This is the only way that
    // I can think of to stop a position-controller based driver. It's OK if
    // there has been a communication error, as this will be ignored.
    for (JointMap::iterator it = joints_.begin(); it != joints_.end(); it++)
      it->second.goal = it->second.val;
    // Stop pan and tilt, but don't touch the gripper
    switch (fsm_.GetState()) {
    case STATE::PANNING:
    case STATE::TILTING:
    case STATE::SETTING:
    case STATE::STOWING_SETTING:
    case STATE::STOWING_PANNING:
    case STATE::STOWING_TILTING:
    case STATE::DEPLOYING_PANNING:
    case STATE::DEPLOYING_TILTING:
    case STATE::STOWED:
    case STATE::CALIBRATING:
      send = true;
    default:
      break;
    }
    // If we need to physically send a response (we are tracking a goal)
    if (send) {
      ff_msgs::Arm::Result::SharedPtr result = std::make_shared<ff_msgs::Arm::Result>();
      result->response = response;
      switch (response) {
        case ff_msgs::Arm::Result::SUCCESS:
          result->fsm_result = "Successfully completed";             break;
        case ff_msgs::Arm::Result::PREEMPTED:
          result->fsm_result = "Action was preempted";               break;
        case ff_msgs::Arm::Result::INVALID_COMMAND:
          result->fsm_result = "Invalid command";                    break;
        case ff_msgs::Arm::Result::BAD_TILT_VALUE:
          result->fsm_result = "Invalid value for tilt";             break;
        case ff_msgs::Arm::Result::BAD_PAN_VALUE:
          result->fsm_result = "Invalid value for pan";              break;
        case ff_msgs::Arm::Result::BAD_GRIPPER_VALUE:
          result->fsm_result = "Invalid value for gripper";          break;
        case ff_msgs::Arm::Result::NOT_ALLOWED:
          result->fsm_result = "Not allowed";                        break;
        case ff_msgs::Arm::Result::TILT_FAILED:
          result->fsm_result = "Tilt command failed";                break;
        case ff_msgs::Arm::Result::PAN_FAILED:
          result->fsm_result = "Pan command failed";                 break;
        case ff_msgs::Arm::Result::GRIPPER_FAILED:
          result->fsm_result = "Gripper command failed";             break;
        case ff_msgs::Arm::Result::COMMUNICATION_ERROR:
          result->fsm_result = "Cannot communicate with arm";        break;
        case ff_msgs::Arm::Result::COLLISION_AVOIDED:
          result->fsm_result = "The arm might collide, failed";      break;
        case ff_msgs::Arm::Result::ENABLE_FAILED:
          result->fsm_result = "Cannot enable the arm servos";       break;
        case ff_msgs::Arm::Result::DISABLE_FAILED:
          result->fsm_result = "Cannot disable the arm servos";      break;
        case ff_msgs::Arm::Result::CALIBRATE_FAILED:
          result->fsm_result = "Cannot calibrate the gripper";       break;
        case ff_msgs::Arm::Result::NO_GOAL:
          result->fsm_result = "Unknown call to calibration";        break;
        }
      if (response > 0)
        server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
      else if (response < 0)
        server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
      else
        server_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
    }
    // Special case: if we lose communication with the low level arm controller
    // then we need to go back to the initializing state.
    if (response == RESPONSE::COMMUNICATION_ERROR)
      return STATE::INITIALIZING;
    // The new waiting state depends
    if (IsStowed())
      return STATE::STOWED;
    return STATE::DEPLOYED;
  }

  // Called on registration of aplanner
  bool SetStateCallback(const std::shared_ptr<ff_msgs::SetState::Request> req,
                        std::shared_ptr<ff_msgs::SetState::Response> res) {
    fsm_.SetState(req->state);
    res->success = true;
    UpdateCallback(fsm_.GetState(), MANUAL_STATE_SET);
    return true;
  }

  // Get tolerance value
  double GetTolerance(JointType t) {
    switch (fsm_.GetState()) {
    case STATE::DEPLOYED:
    case STATE::STOWED:
    case STATE::UNKNOWN:
      if (t == PAN)
        joints_[PAN].tol = cfg_.Get<double>("tol_pan");
      else if (t == TILT)
        joints_[TILT].tol = cfg_.Get<double>("tol_tilt");
      else if (t == GRIPPER)
        joints_[GRIPPER].tol = cfg_.Get<double>("tol_gripper");
      return joints_[t].tol;
    default:
      break;
    }
    return joints_[PAN].tol;
  }

  // Check if the two angle are sufficiently close, respecting modular math
  bool Equal(JointType t, double v) {
    return ((180. - fabs(fabs(joints_[t].val - v) - 180.)) < GetTolerance(t));
  }

  // Look at the pan and tilt angles to determine if stowed
  bool IsStowed() {
    if (!Equal(PAN, K_PAN_STOW))
      return false;
    if (!Equal(TILT, K_TILT_STOW))
      return false;
    return true;
  }

  // Check if gripper requies closing
  bool RequiresClosing() {
    return !Equal(GRIPPER, K_GRIPPER_STOW);
  }

  // Send a single joint goal to the low-level controller. We could in
  // principle send multiple goals at once, but lets keep this simple.
  bool Arm(JointType type) {
    // Check that we actually have the joint present
    JointMap::const_iterator it = joints_.find(type);
    if (it == joints_.end()) {
      FF_WARN_STREAM("Not a valid control goal");
      return false;
    }
    // Package up the joint state goal
    static sensor_msgs::JointState goal;
    goal.header.stamp = GetTimeNow();
    goal.header.frame_id = GetPlatform();
    goal.name.resize(1);
    goal.position.resize(1);
    // Set the name and conver from human to low-level
    goal.position[0] =
      (it->second.goal - it->second.offset) / it->second.scale;
    goal.name[0] = it->second.name;
    if (type == GRIPPER)
      goal.position[0] -= 100.0;
    // Publish the new goal
    pub_joint_goals_->publish(goal);
    // Start a timer
    timer_goal_.stop();
    timer_goal_.setPeriod(
      rclcpp::Duration::from_seconds(cfg_.Get<double>("timeout_goal")));
    timer_goal_.start();
    // Success!
    return true;
  }

  // Called whenever the low-level driver produces updated joint states.
  void JointStateCallback(const std::shared_ptr<sensor_msgs::JointState> msg) {
    ///////////////////////////
    // Send the joint sample //
    ///////////////////////////
    ff_msgs::JointSampleStamped jss;
    jss.header.stamp = GetTimeNow();
    jss.header.frame_id = GetPlatform();
    jss.samples.clear();
    // Iterate over data
    ff_msgs::JointSample js;
    static JointType generic;
    for (size_t i = 0; i < msg->name.size(); i++) {
      if (dictionary_.find(msg->name[i]) == dictionary_.end())
        continue;
      // Get the generic name
      generic = dictionary_[msg->name[i]];
      // In nominal conditions, joints are scaled like this
      joints_[generic].val =
        msg->position[i] * joints_[generic].scale + joints_[generic].offset;
      //  Package up a joint sample
      js.name = joints_[generic].generic;
      js.angle_pos = joints_[generic].val;   // Human-readable
      js.angle_vel = msg->velocity[i];       // SI
      js.current = msg->effort[i];           // SI
      jss.samples.push_back(js);
    }
    // If we dind't receive any valid joint updates, then we are done
    if (jss.samples.empty())
      return;
    // Reset the watchdog timer
    timer_watchdog_.stop();
    timer_watchdog_.setPeriod(cfg_.Get<double>("timeout_watchdog"));
    timer_watchdog_.start();
    // Update the state machine
    switch (fsm_.GetState()) {
    // Background states
    case STATE::UNKNOWN:
      if (IsStowed())
        return fsm_.Update(STOWED);
      return fsm_.Update(DEPLOYED);
    // Catch a manual deploy event
    case STATE::STOWED:
      if (!IsStowed())
        return fsm_.Update(DEPLOYED);
      break;
    // Catch a manual stow event
    case STATE::DEPLOYED:
      if (IsStowed())
        return fsm_.Update(STOWED);
      break;
    // Pan wait states
    case STATE::INITIALIZING:
      fsm_.Update(READY);
      break;
    case STATE::PANNING:
    case STATE::STOWING_PANNING:
    case STATE::DEPLOYING_PANNING:
      if (Equal(PAN, joints_[PAN].goal)) {
        timer_goal_.stop();
        fsm_.Update(PAN_COMPLETE);
      }
      break;
    // Tilt wait state
    case STATE::TILTING:
    case STATE::STOWING_TILTING:
    case STATE::DEPLOYING_TILTING:
      if (Equal(TILT, joints_[TILT].goal)) {
        timer_goal_.stop();
        fsm_.Update(TILT_COMPLETE);
      }
      break;
    // Gripper wait calibrate state
    case STATE::CALIBRATING:
      if (Equal(GRIPPER, K_GRIPPER_OPEN)) {
        timer_goal_.stop();
        fsm_.Update(GRIPPER_COMPLETE);
      }
      break;
    // Gripper wait state
    case STATE::SETTING:
    case STATE::STOWING_SETTING:
      if (Equal(GRIPPER, joints_[GRIPPER].goal)) {
        timer_goal_.stop();
        fsm_.Update(GRIPPER_COMPLETE);
      }
      break;
    // Catch-all for other states
    default:
      return;
    }
    // Publish the updated joint samples
    pub_joint_sample_->publish(jss);
    /////////////////////////
    // Send the full state //
    /////////////////////////
    ff_msgs::ArmStateStamped state_msg;
    state_msg.header.frame_id = GetPlatform();
    state_msg.header.stamp = GetTimeNow();
    // Convert our internal state to an ArmGripperState
    if (fabs(joints_[GRIPPER].val - K_GRIPPER_CLOSE) < GetTolerance(GRIPPER))
      state_msg.gripper_state.state = ff_msgs::ArmGripperState::CLOSED;
    else
      state_msg.gripper_state.state = ff_msgs::ArmGripperState::OPEN;
    // Convert the internal state to an ArmJointState
    switch (fsm_.GetState()) {
    default:
    case STATE::UNKNOWN:
      state_msg.joint_state.state = ff_msgs::ArmJointState::UNKNOWN;
      break;
    case STATE::DEPLOYING_PANNING:
    case STATE::DEPLOYING_TILTING:
      state_msg.joint_state.state = ff_msgs::ArmJointState::DEPLOYING;
      break;
    case STATE::CALIBRATING:
      if (goal_stow_)
        state_msg.joint_state.state = ff_msgs::ArmJointState::STOWING;
      else if (goal_set_)
        state_msg.joint_state.state = ff_msgs::ArmJointState::STOPPED;
      break;
    case STATE::DEPLOYED:
    case STATE::SETTING:
      state_msg.joint_state.state = ff_msgs::ArmJointState::STOPPED;
      break;
    case STATE::PANNING:
    case STATE::TILTING:
      state_msg.joint_state.state = ff_msgs::ArmJointState::MOVING;
      break;
    case STATE::STOWING_SETTING:
    case STATE::STOWING_PANNING:
    case STATE::STOWING_TILTING:
      state_msg.joint_state.state = ff_msgs::ArmJointState::STOWING;
      break;
    case STATE::STOWED:
      state_msg.joint_state.state = ff_msgs::ArmJointState::STOWED;
      break;
    }
    pub_arm_state_->publish(state_msg);
    //////////////////////////
    // Send action feedback //
    //////////////////////////
    static ff_msgs::Arm::Feedback::SharedPtr feedback = std::make_shared<ff_msgs::Arm::Feedback>();
    switch (fsm_.GetState()) {
    case STATE::PANNING:
    case STATE::TILTING:
    case STATE::SETTING:
    case STATE::STOWING_SETTING:
    case STATE::STOWING_PANNING:
    case STATE::STOWING_TILTING:
    case STATE::DEPLOYING_PANNING:
    case STATE::DEPLOYING_TILTING:
      feedback->state.state = fsm_.GetState();
      feedback->pan = joints_[PAN].val;
      feedback->tilt = joints_[TILT].val;
      feedback->gripper = joints_[GRIPPER].val;
      server_.SendFeedback(feedback);
    default:
      break;
    }
  }

  // Called to fake feedback to executive during a prep
  void TimeoutCallback() {
    fsm_.Update(TIMEOUT);
  }

  // If the watchdog expires, it means that after connecting we went for a
  // specified period without joint state feedback. In this case we need
  // to send a communication error to the callee.
  void WatchdogCallback() {
    fsm_.SetState(Result(RESPONSE::COMMUNICATION_ERROR));
  }

  // A new arm action has been called
  void GoalCallback(std::shared_ptr<const ff_msgs::Arm::Goal> goal) {
    // We are connected
    switch (goal->command) {
    // Stop the arm
    case ff_msgs::Arm::Goal::ARM_STOP:
      FF_DEBUG_STREAM("Received a new ARM_STOP command");
      joints_[PAN].goal = joints_[PAN].val;
      joints_[TILT].goal = joints_[TILT].val;
      joints_[GRIPPER].goal = joints_[GRIPPER].val;
      return fsm_.Update(GOAL_CANCEL);
    // Deploy the arm
    case ff_msgs::Arm::Goal::ARM_DEPLOY:
      FF_DEBUG_STREAM("Received a new ARM_DEPLOY command");
      joints_[PAN].goal = K_PAN_DEPLOY;
      joints_[TILT].goal = K_TILT_DEPLOY;
      joints_[GRIPPER].goal = K_GRIPPER_DEPLOY;
      return fsm_.Update(GOAL_DEPLOY);
      break;
    // Stow the arm
    case ff_msgs::Arm::Goal::ARM_STOW:
      FF_DEBUG_STREAM("Received a new ARM_STOW command");
      if (fsm_.GetState() == STATE::STOWED) {
        Result(RESPONSE::SUCCESS, true);
      }
      joints_[PAN].goal = K_PAN_STOW;
      joints_[TILT].goal = K_TILT_STOW;
      joints_[GRIPPER].goal = K_GRIPPER_STOW;
      return fsm_.Update(GOAL_STOW);
      break;
    // Pan the arm
    case ff_msgs::Arm::Goal::ARM_PAN:
    case ff_msgs::Arm::Goal::ARM_TILT:
    case ff_msgs::Arm::Goal::ARM_MOVE:
      FF_DEBUG_STREAM("Received a new ARM_{PAN,TILT,MOVE} command");
      if (fsm_.GetState() == STATE::DEPLOYED ||
          fsm_.GetState() == STATE::STOWED) {
        // Get the new, proposed PAN and TILT values
        double new_p = joints_[PAN].goal;
        if (goal->command == ff_msgs::Arm::Goal::ARM_MOVE ||
            goal->command == ff_msgs::Arm::Goal::ARM_PAN)
          new_p = goal->pan;
        double new_t = joints_[TILT].goal;
        if (goal->command == ff_msgs::Arm::Goal::ARM_MOVE ||
            goal->command == ff_msgs::Arm::Goal::ARM_TILT)
          new_t = goal->tilt;
        // Simple bounds and self-collision checking
        if (new_t < K_TILT_MIN || new_t > K_TILT_MAX) {
          Result(RESPONSE::BAD_TILT_VALUE, true);
          return;
        }
        if (new_p < K_PAN_MIN || new_p > K_PAN_MAX) {
          Result(RESPONSE::BAD_PAN_VALUE, true);
          return;
        }
        // Check current and goal tilt, and if close to stowed, make
        // sure the pan value is zero within tolerance
        if ((joints_[TILT].goal > K_TILT_SAFE || new_t > K_TILT_SAFE)
          && fabs(new_p - K_PAN_STOW) > GetTolerance(PAN)) {
          Result(RESPONSE::COLLISION_AVOIDED, true);
          return;
        }
        // check gripper, to make sure if doesn't close with gripper opened
        if (new_t > K_TILT_GRIP && !Equal(GRIPPER, K_GRIPPER_STOW)) {
          Result(RESPONSE::COLLISION_AVOIDED, true);
          return;
        }
        // Set the new goals
        joints_[PAN].goal = new_p;
        joints_[TILT].goal = new_t;
        // Start the action
        return fsm_.Update(GOAL_MOVE);
      }
      break;
    // Set the gripper
    case ff_msgs::Arm::Goal::GRIPPER_SET:
      FF_DEBUG_STREAM("Received a new GRIPPER_SET command");
      if (fsm_.GetState() == STATE::DEPLOYED) {
        // Check that th gripper value is reasonable
        if (goal->gripper < K_GRIPPER_CLOSE || goal->gripper > K_GRIPPER_OPEN) {
          Result(RESPONSE::BAD_GRIPPER_VALUE, true);
          return;
        }
        // Set the goal
        joints_[GRIPPER].goal = goal->gripper;
        // Star the action
        return fsm_.Update(GOAL_SET);
      }
      break;
    // Open the gripper
    case ff_msgs::Arm::Goal::GRIPPER_OPEN:
      FF_DEBUG_STREAM("Received a new GRIPPER_OPEN command");
      if (fsm_.GetState() == STATE::DEPLOYED) {
        joints_[GRIPPER].goal = K_GRIPPER_OPEN;
        return fsm_.Update(GOAL_SET);
      }
      break;
    // Close the gripper
    case ff_msgs::Arm::Goal::GRIPPER_CLOSE:
      FF_DEBUG_STREAM("Received a new GRIPPER_CLOSE command");
      if (fsm_.GetState() == STATE::DEPLOYED) {
        joints_[GRIPPER].goal = K_GRIPPER_CLOSE;
        return fsm_.Update(GOAL_SET);
      }
      break;
    case ff_msgs::Arm::Goal::DISABLE_SERVO:
      FF_DEBUG_STREAM("Received a new DISABLE_SERVO command");
      if (fsm_.GetState() == STATE::STOWED) {
        return fsm_.Update(GOAL_DISABLE);
      }
      break;
    default:
      // Catch-all for impossible state transitions
      Result(RESPONSE::INVALID_COMMAND, true);
      return;
    }
    // Catch-all for impossible state transitions
    Result(RESPONSE::NOT_ALLOWED, true);
  }

  // Preempt the current action with a new action
  void PreemptCallback() {
    Result(RESPONSE::PREEMPTED);
    return;
  }

  // A Cancellation request arrives
  void CancelCallback() {
    joints_[PAN].goal = joints_[PAN].val;
    joints_[TILT].goal = joints_[TILT].val;
    joints_[GRIPPER].goal = joints_[GRIPPER].val;
    return fsm_.Update(GOAL_CANCEL);
  }

  // Calibrates the Gripper (surprising!)
  bool CalibrateGripper(void) {
    bool success = false;
    // CalibrateGripper callback doesn't have a request field
    ff_hw_msgs::CalibrateGripper::Request enable_req;
    auto response = std::make_shared<ff_hw_msgs::CalibrateGripper::Response>();

    timer_goal_.setPeriod(
    rclcpp::Duration::from_seconds(cfg_.Get<double>("timeout_goal")));
    timer_goal_.start();
    if (client_calibrate_gripper_.call(enable_req, response)) {
      success = response->success;
    }
    if (success)
      calibrated_ = true;
    return success;
  }

  // Enable / Disable the perching arm servos
  bool EnableServo(ServoID servo_number, bool servo_enable) {
    bool success = false;
    ff_hw_msgs::SetEnabled::Request enable_req;
    auto response = std::make_shared<ff_hw_msgs::SetEnabled::Response>();
    enable_req.enabled = servo_enable;
      switch (servo_number) {
      case PROXIMAL_SERVO:    // Proximal
        client_enable_prox_servo_.waitForExistence(5.0);
        if (client_enable_prox_servo_.isValid() && client_enable_prox_servo_.call(enable_req, response))
          success = response->success;
        break;
      case DISTAL_SERVO:      // Distal
        client_enable_dist_servo_.waitForExistence(5.0);
        if (client_enable_dist_servo_.isValid() && client_enable_dist_servo_.call(enable_req, response))
          success = response->success;
        break;
      case GRIPPER_SERVO:     // Gripper
        client_enable_grip_servo_.waitForExistence(5.0);
        if (client_enable_grip_servo_.isValid() && client_enable_grip_servo_.call(enable_req, response))
          success = response->success;
        break;
      case ALL_SERVOS:        // All
        client_enable_prox_servo_.waitForExistence(5.0);
        if (client_enable_prox_servo_.isValid() && client_enable_prox_servo_.call(enable_req, response))
          success = response->success;
        client_enable_dist_servo_.waitForExistence(5.0);
        if (client_enable_dist_servo_.isValid() && client_enable_dist_servo_.call(enable_req, response))
          success = success && response->success;
        client_enable_grip_servo_.waitForExistence(5.0);
        if (client_enable_grip_servo_.isValid() && client_enable_grip_servo_.call(enable_req, response))
          success = success && response->success;
        break;
      default:
        FF_WARN("Unknown servo number");
        break;
    }
     return success;
  }




 protected:
  // Finite state machine
  ff_util::FSM fsm_;
  // Config server
  ff_util::ConfigServer cfg_;
  // Action server
  ff_util::FreeFlyerActionServer<ff_msgs::Arm> server_;
  // Data structures supporting lookup of joint data
  JointMap joints_;
  JointDictionary dictionary_;
  // Timers, publishers and subscribers
  ff_util::FreeFlyerTimer timer_goal_;              // Timer for goal to complete
  ff_util::FreeFlyerTimer timer_watchdog_;          // Watchdog timer for LL data
  rclcpp::Publisher<ff_msgs::ArmState>::SharedPtr pub_state_;           // State publisher
  rclcpp::Service<ff_msgs::SetState>::SharedPtr srv_set_state_;   // Set the current state
  rclcpp::Subscription<sensor_msgs::JointState>::SharedPtr sub_joint_states_;   // Joint state subscriber
  rclcpp::Publisher<sensor_msgs::JointState>::SharedPtr pub_joint_goals_;     // Joint goal publisher
  rclcpp::Publisher<ff_msgs::ArmStateStamped>::SharedPtr pub_arm_state_;       // Executive arm state publisher
  rclcpp::Publisher<ff_msgs::JointSampleStamped>::SharedPtr pub_joint_sample_;    // Executive joint state publisher
  ff_util::FreeFlyerServiceClient<ff_hw_msgs::SetEnabled> client_enable_prox_servo_;
  ff_util::FreeFlyerServiceClient<ff_hw_msgs::SetEnabled> client_enable_dist_servo_;
  ff_util::FreeFlyerServiceClient<ff_hw_msgs::SetEnabled> client_enable_grip_servo_;
  ff_util::FreeFlyerServiceClient<ff_hw_msgs::CalibrateGripper> client_calibrate_gripper_;

  // Constant vales
  static constexpr double K_PAN_OFFSET      =    0.0;
  static constexpr double K_PAN_MIN         =  -90.0;
  static constexpr double K_PAN_MAX         =   90.0;
  static constexpr double K_PAN_STOW        =    0.0;
  static constexpr double K_PAN_DEPLOY      =    0.0;
  static constexpr double K_TILT_OFFSET     =   90.0;
  static constexpr double K_TILT_MIN        =  -20.0;
  static constexpr double K_TILT_MAX        =  180.0;
  static constexpr double K_TILT_STOW       =  180.0;
  static constexpr double K_TILT_DEPLOY     =    0.0;
  static constexpr double K_TILT_SAFE       =   90.0;
  static constexpr double K_TILT_GRIP       =  160.0;
  static constexpr double K_GRIPPER_STOW    =    0.0;
  static constexpr double K_GRIPPER_DEPLOY  =    0.0;
  static constexpr double K_GRIPPER_OPEN    =  100.0;
  static constexpr double K_GRIPPER_CLOSE   =    0.0;
  static constexpr double K_RADS_TO_DEGS    = 180.0 / M_PI;
  static constexpr double K_DEGS_TO_RADS    = M_PI / 180.0;

  bool calibrated_ = false;
  bool goal_stow_ = false;
  bool goal_set_ = false;
};

}  // namespace arm

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(arm::ArmComponent)
