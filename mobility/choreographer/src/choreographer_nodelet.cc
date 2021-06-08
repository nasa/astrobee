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
#include <tf2_ros/transform_listener.h>

// FSW libraries
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_service.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_fsm.h>
#include <ff_util/config_server.h>
#include <ff_util/conversion.h>

// Generic messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/InertiaStamped.h>

// Hardware messages
#include <ff_hw_msgs/PmcState.h>

// Messages
#include <ff_msgs/Hazard.h>
#include <ff_msgs/MotionState.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/ControlCommand.h>

// Services
#include <ff_msgs/RegisterPlanner.h>
#include <ff_msgs/SetState.h>
#include <ff_msgs/SetInertia.h>

// Actions
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/ControlAction.h>
#include <ff_msgs/PlanAction.h>

// Segment validation code
#include <choreographer/validator.h>

// STL includes
#include <string>
#include <memory>
#include <functional>
#include <map>
#include <utility>

/**
 * \ingroup mobility
 */
namespace choreographer {

// Convenience declarations
using STATE = ff_msgs::MotionState;
using RESPONSE = ff_msgs::MotionResult;
using FSM = ff_util::FSM;

// Data structure for managing planner registration
typedef std::map<std::string, std::string> PlannerInfo;
typedef std::map<std::string,
  ff_util::FreeFlyerActionClient<ff_msgs::PlanAction>> PlannerMap;

// The choreograph class manages core logic for mobility
class ChoreographerNodelet : public ff_util::FreeFlyerNodelet {
 public:
  // Bitmask of all events
  enum : FSM::Event {
    READY                          = (1<<0),      // System is initialized
    GOAL_EXEC                      = (1<<1),      // New execute command
    GOAL_MOVE                      = (1<<2),      // New move command
    GOAL_STOP                      = (1<<3),      // New stop command
    GOAL_IDLE                      = (1<<4),      // New idle command
    GOAL_PREP                      = (1<<5),      // New prep command
    GOAL_CANCEL                    = (1<<6),      // Cancel existing goal
    PLAN_SUCCESS                   = (1<<7),      // Segment plan success
    PLAN_FAILED                    = (1<<8),      // Segment plan failed
    PMC_READY                      = (1<<9),      // PMC is ramped and ready
    PMC_TIMEOUT                    = (1<<10),     // PMC has timed out
    CONTROL_SUCCESS                = (1<<11),     // Control success
    CONTROL_FAILED                 = (1<<12),     // Control failure
    TOLERANCE_POS                  = (1<<13),     // Position olerance failure
    TOLERANCE_ATT                  = (1<<14),     // Attitude tolerance failure
    TOLERANCE_VEL                  = (1<<15),     // Velocity tolerance failure
    TOLERANCE_OMEGA                = (1<<16),     // Omega tolerance failure
    OBSTACLE_DETECTED              = (1<<17),     // Hazard: obstacle
    MANUAL_STATE_SET               = (1<<18)     // Setting the state manually with service
  };

  // The various types of control
  enum ControlType { IDLE, STOP, NOMINAL };

  // Constructor
  ChoreographerNodelet() : ff_util::FreeFlyerNodelet(NODE_CHOREOGRAPHER, true),
    fsm_(STATE::INITIALIZING, std::bind(&ChoreographerNodelet::UpdateCallback,
      this, std::placeholders::_1, std::placeholders::_2)) {
    // [0]
    fsm_.Add(STATE::INITIALIZING,
      READY,
      [this](FSM::Event const& event) -> FSM::State {
        if (!Control(IDLE))
          AssertFault(ff_util::INITIALIZATION_FAILED,
                      "Cannot call Idle() action");
        return STATE::IDLE;
      });
    // [2]
    fsm_.Add(STATE::STOPPED, STATE::IDLE,
      GOAL_EXEC,
      [this](FSM::Event const& event) -> FSM::State {
        // Assemble a simple planning request
        geometry_msgs::PoseStamped pose;
        if (!GetRobotPose(pose))
          return Result(RESPONSE::CANNOT_QUERY_ROBOT_POSE);
        // If the robot is already on the target pose then no bootstrap needed
        if (ff_util::FlightUtil::WithinTolerance(segment_.front().pose,
          pose.pose, cfg_.Get<double>("tolerance_pos"),
            cfg_.Get<double>("tolerance_att"))) {
          // If we need to validate
          if (cfg_.Get<bool>("enable_validation")) {
            Validator::Response result = validator_.CheckSegment(segment_,
              flight_mode_, cfg_.Get<bool>("enable_faceforward"));
            if (result != Validator::SUCCESS)
              return ValidateResult(result);
          }
          // Take care of the special case where we have to prep!
          if (FlightMode())
            return STATE::PREPARING;
          // If we send control directly
          if (!Control(NOMINAL, segment_))
            return Result(RESPONSE::CONTROL_FAILED);
          return STATE::CONTROLLING;
        }
        // We are not on the first pose, we need to plan to get there
        if (cfg_.Get<bool>("enable_bootstrapping")) {
          // Assemble a simple plan
          std::vector<geometry_msgs::PoseStamped> states;
          geometry_msgs::PoseStamped pose;
          // Get the robot pose and force timestamp alignment
          if (!GetRobotPose(pose))
            return Result(RESPONSE::CANNOT_QUERY_ROBOT_POSE);
          pose.header.stamp = segment_.front().when;
          states.push_back(pose);
          // Get the first pose of the segment
          pose.header.stamp = segment_.front().when;
          pose.pose = segment_.front().pose;
          states.push_back(pose);
          // Now, query the planning subsystem to find a suitable segment
          if (!Plan(states))
            return Result(RESPONSE::PLAN_FAILED);
          return STATE::BOOTSTRAPPING;
        }
        return Result(RESPONSE::NOT_ON_FIRST_POSE);
      });
    // [3a]
    fsm_.Add(STATE::BOOTSTRAPPING,
      PLAN_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        // If we need to validate
        if (cfg_.Get<bool>("enable_validation")) {
          Validator::Response result = validator_.CheckSegment(segment_,
            flight_mode_, cfg_.Get<bool>("enable_faceforward"));
          if (result != Validator::SUCCESS)
            return ValidateResult(result);
        }
        // Take care of the case where we have to prep!
        if (FlightMode())
          return STATE::PREPARING;
        // If we send control directly
        if (!Control(NOMINAL, segment_))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::CONTROLLING;
      });
    // [4]
    fsm_.Add(STATE::BOOTSTRAPPING,
      PLAN_FAILED | GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::PLAN_FAILED);
      });
    // [5]
    fsm_.Add(STATE::STOPPED, STATE::IDLE,
      GOAL_MOVE,
      [this](FSM::Event const& event) -> FSM::State {
        // Assemble a simple planning request
        geometry_msgs::PoseStamped pose;
        // Get the robot pose
        if (!GetRobotPose(pose))
          return Result(RESPONSE::CANNOT_QUERY_ROBOT_POSE);
        // Cover a special case where we are already on ALL desired poses.
        bool aligned = true;
        std::vector<geometry_msgs::PoseStamped>::iterator it;
        for (it = states_.begin(); it != states_.end() && aligned; ++it)
          aligned &= ff_util::FlightUtil::WithinTolerance(pose.pose, it->pose,
            cfg_.Get<double>("tolerance_pos"), cfg_.Get<double>("tolerance_att"));
        if (aligned)
          return Result(RESPONSE::ALREADY_THERE);
        // Add the current pose to the front of the states
        states_.insert(states_.begin(), pose);
        // Now, query the planning subsystem to find a suitable segment
        if (!Plan(states_))
          return Result(RESPONSE::PLAN_FAILED);
        return STATE::PLANNING;
      });
    // [6]
    fsm_.Add(STATE::PLANNING,
      PLAN_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        // If we need to validate
        if (cfg_.Get<bool>("enable_validation")) {
          Validator::Response result = validator_.CheckSegment(segment_,
            flight_mode_, cfg_.Get<bool>("enable_faceforward"));
          if (result != Validator::SUCCESS)
            return ValidateResult(result);
        }
        // If this returns false then we are already on the correct speed gain
        if (FlightMode())
          return STATE::PREPARING;
        // Move straight to control, as we don't need to wait for speed prep.
        if (!Control(NOMINAL, segment_))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::CONTROLLING;
      });
    // [7]
    fsm_.Add(STATE::PLANNING,
      PLAN_FAILED | GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::PLAN_FAILED);
      });
    // [10]
    fsm_.Add(STATE::REPLANNING,
      PLAN_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        // If we need to validate the segment
        if (cfg_.Get<bool>("enable_validation")) {
          Validator::Response result = validator_.CheckSegment(segment_,
            flight_mode_, cfg_.Get<bool>("enable_faceforward"));
          if (result != Validator::SUCCESS)
            return ValidateResult(result);
        }
        // If this returns false then we are already on the correct speed gain
        if (FlightMode())
          return STATE::PREPARING;
        // If we send control directly
        if (!Control(NOMINAL, segment_))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::CONTROLLING;
      });
    // [13]
    fsm_.Add(STATE::CONTROLLING,
      CONTROL_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        return Result(RESPONSE::SUCCESS);
      });
    // [14]
    fsm_.Add(STATE::REPLANNING,
      PLAN_FAILED | GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        int max_attempts = cfg_.Get<int>("max_replanning_attempts");
        NODELET_DEBUG_STREAM("State Replanning: Replanning failed %i / %i times" <<
          replan_attempts_ << max_attempts);
        if (replan_attempts_ >= max_attempts) {
          return Result(RESPONSE::REPLAN_FAILED);
        }
        // Get current Astrobee pose
        geometry_msgs::PoseStamped current_pose;
        if (!GetRobotPose(current_pose)) {
          return Result(RESPONSE::CANNOT_QUERY_ROBOT_POSE);
        }
        states_.insert(states_.begin(), current_pose);
        replan_attempts_++;
        ros::Duration(1).sleep();  // sleep for a second
        if (!Plan(states_)) {
          return Result(RESPONSE::PLAN_FAILED);
        }
        return STATE::REPLANNING;
      });
    // [15]
    fsm_.Add(STATE::CONTROLLING,
      CONTROL_FAILED | GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::CONTROL_FAILED);
      });
    // [28]
    fsm_.Add(STATE::CONTROLLING,
      TOLERANCE_POS | TOLERANCE_ATT | TOLERANCE_VEL | TOLERANCE_OMEGA,
      [this](FSM::Event const& event) -> FSM::State {
        switch (event) {
        case TOLERANCE_POS:
          return Result(RESPONSE::TOLERANCE_VIOLATION_POSITION);
        case TOLERANCE_ATT:
          return Result(RESPONSE::TOLERANCE_VIOLATION_ATTITUDE);
        case TOLERANCE_VEL:
          return Result(RESPONSE::TOLERANCE_VIOLATION_VELOCITY);
        case TOLERANCE_OMEGA:
          return Result(RESPONSE::TOLERANCE_VIOLATION_OMEGA);
        default:
          break;
        }
        return Result(RESPONSE::CONTROL_FAILED);
      });
    // [16]
    fsm_.Add(STATE::CONTROLLING, OBSTACLE_DETECTED,
      [this](FSM::Event const& event) -> FSM::State {
        if (cfg_.Get<bool>("enable_replanning")) {
          // Request a stop using the controller
          if (!Control(STOP)) {
            return Result(RESPONSE::CONTROL_FAILED);
          }
          return STATE::HALTING;
        }
        // If we're not doing replanning
        return Result(RESPONSE::OBSTACLE_DETECTED);
      });
    // [17]
    fsm_.Add(STATE::STOPPED, STATE::IDLE,
      GOAL_STOP,
      [this](FSM::Event const& event) -> FSM::State {
        // Force the flight mode  change and immediately send control
        FlightMode();
        if (!Control(STOP))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::STOPPING;
      });
    // [18]
    fsm_.Add(STATE::STOPPING,
      CONTROL_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        return Result(RESPONSE::SUCCESS);
      });
    // [19]
    fsm_.Add(STATE::STOPPING,
      CONTROL_FAILED | GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::CONTROL_FAILED);
      });
    // [20]
    fsm_.Add(STATE::STOPPED, STATE::IDLE,
      GOAL_IDLE,
      [this](FSM::Event const& event) -> FSM::State {
        // Force the flight mode  change and immediately send control
        FlightMode();
        if (!Control(IDLE))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::IDLING;
      });
    // [21]
    fsm_.Add(STATE::IDLING,
      CONTROL_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        return Result(RESPONSE::SUCCESS);
      });
    // [22]
    fsm_.Add(STATE::IDLING,
      CONTROL_FAILED | GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::CONTROL_FAILED);
      });
    // [23]
    fsm_.Add(STATE::PREPARING,
      PMC_READY,
      [this](FSM::Event const& event) -> FSM::State {
        if (!Control(NOMINAL, segment_))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::CONTROLLING;
      });
    // [24]
    fsm_.Add(STATE::PREPARING,
      PMC_TIMEOUT | GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::PMC_FAILED);
      });
    // [25]
    fsm_.Add(STATE::STOPPED, STATE::IDLE,
      GOAL_PREP,
      [this](FSM::Event const& event) -> FSM::State {
        if (FlightMode())
          return STATE::PREPPING;
        return Result(RESPONSE::SUCCESS);
      });
    // [26]
    fsm_.Add(STATE::PREPPING,
      PMC_READY,
      [this](FSM::Event const& event) -> FSM::State {
        return Result(RESPONSE::SUCCESS);
      });
    // [27]
    fsm_.Add(STATE::PREPPING,
      PMC_TIMEOUT |  GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::PMC_FAILED);
      });
    // [28]
    fsm_.Add(STATE::HALTING, CONTROL_SUCCESS,
      [this](FSM::Event const& event) -> FSM::State {
        // Get current Astrobee pose
        geometry_msgs::PoseStamped current_pose;
        if (!GetRobotPose(current_pose)) {
          return Result(RESPONSE::CANNOT_QUERY_ROBOT_POSE);
        }
        states_.insert(states_.begin(), current_pose);
        // Send plan request
        replan_attempts_ = 1;
        if (!Plan(states_)) {
          return Result(RESPONSE::PLAN_FAILED);
        }
        return STATE::REPLANNING;
      });
    // [29]
    fsm_.Add(STATE::HALTING,
      CONTROL_FAILED | GOAL_CANCEL,
      [this](FSM::Event const& event) -> FSM::State {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::CONTROL_FAILED);
      });
  }

  // Destructor
  ~ChoreographerNodelet() {}

 protected:
  // Called on statup
  void Initialize(ros::NodeHandle *nh) {
    // Configuration parameters
    cfg_.Initialize(GetPrivateHandle(), "mobility/choreographer.config");

    // Listen for parameter changes
    if (!cfg_.Listen(boost::bind(
        &ChoreographerNodelet::ReconfigureCallback, this, _1))) {
      AssertFault(ff_util::INITIALIZATION_FAILED,
                  "Could not start config server");
      return;
    }

    // Initialize the zone manager
    validator_.Init(nh, cfg_);

    // Get the default flight mode, it will be published later
    if (!ff_util::FlightUtil::GetInitialFlightMode(flight_mode_)) {
      AssertFault(ff_util::INITIALIZATION_FAILED,
                  "Problem with initial flight mode");
      return;
    }

    // Get the default inertia parameters, it will be published later
    geometry_msgs::InertiaStamped msg;
    if (!ff_util::FlightUtil::GetInertiaConfig(msg)) {
      AssertFault(ff_util::INITIALIZATION_FAILED,
                  "Problem with default inertia");
      return;
    }

    // Read max time a tolerance is allowed
    tolerance_max_time_ = cfg_.Get<double>("tolerance_max_time");

    // Create a transform buffer ot listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(tf_buffer_));

    // One shot timer to check if we undock with a timeout (no autostart)
    timer_speed_ = nh->createTimer(
      ros::Duration(cfg_.Get<double>("timeout_speed_gain")),
        &ChoreographerNodelet::PmcTimeout, this, true, false);

    // Periodic timer to send feedback to executive to avoid timeout
    timer_feedback_ = nh->createTimer(ros::Rate(2.0),
        &ChoreographerNodelet::FeedbackCallback, this, false, false);

    // Publish segments to be picked up by rviz
    pub_segment_ = nh->advertise<nav_msgs::Path>(
      TOPIC_MOBILITY_SEGMENT, 5, true);

    // Publish an internal state for executive to track
    pub_state_ = nh->advertise<ff_msgs::MotionState>(
      TOPIC_MOBILITY_MOTION_STATE, 5, true);

    // Publish flight mode for the benefit of the whole system
    // gnc::EKF : uses speed gain to peform virbation compensation
    // gnc::FAM : uses speed gain to set impeller speeds
    // gnc::CTL : uses controller gains to control platform
    pub_flight_mode_ = nh->advertise<ff_msgs::FlightMode>(
      TOPIC_MOBILITY_FLIGHT_MODE, 1, true);

    // Publish inertia once on start to ensure GNC is bootstrapped
    // correctly even if executive is not loaded.
    pub_inertia_ = nh->advertise<geometry_msgs::InertiaStamped>(
      TOPIC_MOBILITY_INERTIA, 1, true);

    // Subscribe to collisions from the sentinel node
    sub_hazard_ = nh->subscribe(TOPIC_MOBILITY_HAZARD, 5,
      &ChoreographerNodelet::HazardCallback, this);

    // Subscribe to the latched PMC state to be notified of propulsion events
    sub_pmc_state_= nh->subscribe(TOPIC_HARDWARE_PMC_STATE, 5,
      &ChoreographerNodelet::PmcStateCallback, this);

    // Allow planners to register themselves
    server_register_ = nh->advertiseService(SERVICE_MOBILITY_PLANNER_REGISTER,
      &ChoreographerNodelet::PlannerRegisterCallback, this);

    // Allow the state to be manually set
    server_set_state_ = nh->advertiseService(SERVICE_MOBILITY_SET_STATE,
      &ChoreographerNodelet::SetStateCallback, this);

    // Allow the state to be manually set
    server_set_inertia_ = nh->advertiseService(SERVICE_MOBILITY_SET_INERTIA,
      &ChoreographerNodelet::SetInertiaCallback, this);

    // Setup control client action
    client_c_.SetConnectedTimeout(
      cfg_.Get<double>("timeout_control_connected"));
    client_c_.SetActiveTimeout(
      cfg_.Get<double>("timeout_control_active"));
    client_c_.SetResponseTimeout(
      cfg_.Get<double>("timeout_control_response"));
    client_c_.SetFeedbackCallback(std::bind(
      &ChoreographerNodelet::CFeedbackCallback, this,
        std::placeholders::_1));
    client_c_.SetResultCallback(std::bind(
      &ChoreographerNodelet::CResultCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    client_c_.SetConnectedCallback(std::bind(
      &ChoreographerNodelet::ConnectedCallback, this));
    client_c_.Create(nh, ACTION_GNC_CTL_CONTROL);

    // Setup the move action
    server_.SetGoalCallback(std::bind(
      &ChoreographerNodelet::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &ChoreographerNodelet::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &ChoreographerNodelet::CancelCallback, this));
    server_.Create(nh, ACTION_MOBILITY_MOTION);

    // Publish the default flight mode so the system boots predictably
    pub_flight_mode_.publish(flight_mode_);

    // Publish the default inertia parameters
    pub_inertia_.publish(msg);
  }

  // Callback to handle reconfiguration requests
  bool ReconfigureCallback(dynamic_reconfigure::Config & config) {
    return cfg_.Reconfigure(config);
  }

  // Ensure all clients are connected
  void ConnectedCallback() {
    if (!client_c_.IsConnected()) return;       // Control action
    fsm_.Update(READY);
  }

  // Called on registration of aplanner
  bool SetStateCallback(ff_msgs::SetState::Request& req,
                        ff_msgs::SetState::Response& res) {
    fsm_.SetState(req.state);
    res.success = true;
    UpdateCallback(fsm_.GetState(), MANUAL_STATE_SET);
    return true;
  }

  // Called on registration of aplanner
  bool SetInertiaCallback(ff_msgs::SetInertia::Request& req,
                          ff_msgs::SetInertia::Response& res) {
    // TODO(Andrew) Potentially sanity check inertia?
    pub_inertia_.publish(req.inertia);
    res.success = true;
    return true;
  }

  // Called on registration of aplanner
  bool PlannerRegisterCallback(ff_msgs::RegisterPlanner::Request& req,
    ff_msgs::RegisterPlanner::Response& res) {
    // See if the planner already exists
    bool found = planners_.find(req.name) != planners_.end();
    if (req.unregister) {
      if (found) {
        planners_.erase(req.name);
        info_.erase(req.name);
        cfg_.Lim<std::string>("planner", info_);
        return true;
      }
      return false;
    }
    // See if the planner already exists
    if (found)
      return false;
    // Register the planner
    info_[req.name] = req.description;
    planners_[req.name].SetConnectedTimeout(
      cfg_.Get<double>("timeout_plan_connected"));
    planners_[req.name].SetActiveTimeout(
      cfg_.Get<double>("timeout_plan_active"));
    planners_[req.name].SetResponseTimeout(
      cfg_.Get<double>("timeout_plan_response"));
    planners_[req.name].SetDeadlineTimeout(
      cfg_.Get<double>("timeout_plan_deadline"));
    planners_[req.name].SetFeedbackCallback(std::bind(
      &ChoreographerNodelet::PFeedbackCallback, this,
        std::placeholders::_1));
    planners_[req.name].SetResultCallback(std::bind(
      &ChoreographerNodelet::PResultCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    planners_[req.name].SetConnectedCallback(
      std::bind(&ChoreographerNodelet::ConnectedCallback, this));
    std::string topic = std::string(PREFIX_MOBILITY_PLANNER)
                      + req.name
                      + std::string(SUFFIX_MOBILITY_PLANNER);
    planners_[req.name].Create(GetPlatformHandle(), topic);
    // Update the list of planning algorithms
    cfg_.Lim < std::string > ("planner", info_);
    // Success!
    return true;
  }

  // Proxy for validation responses
  int32_t ValidateResult(int validator_response) {
    int32_t result = 0;
    switch (validator_response) {
    case Validator::VIOLATES_RESAMPLING:
      result = RESPONSE::VIOLATES_RESAMPLING; break;
    case Validator::VIOLATES_KEEP_OUT:
      result = RESPONSE::VIOLATES_KEEP_OUT; break;
    case Validator::VIOLATES_KEEP_IN:
      result = RESPONSE::VIOLATES_KEEP_IN; break;
    case Validator::VIOLATES_MINIMUM_FREQUENCY:
      result = RESPONSE::VIOLATES_MINIMUM_FREQUENCY; break;
    case Validator::VIOLATES_STATIONARY_ENDPOINT:
      result = RESPONSE::VIOLATES_STATIONARY_ENDPOINT; break;
    case Validator::VIOLATES_FIRST_IN_PAST:
      result = RESPONSE::VIOLATES_FIRST_IN_PAST; break;
    case Validator::VIOLATES_MINIMUM_SETPOINTS:
      result = RESPONSE::VIOLATES_MINIMUM_SETPOINTS; break;
    case Validator::VIOLATES_HARD_LIMIT_VEL:
      result = RESPONSE::VIOLATES_HARD_LIMIT_VEL; break;
    case Validator::VIOLATES_HARD_LIMIT_ACCEL:
      result = RESPONSE::VIOLATES_HARD_LIMIT_ACCEL; break;
    case Validator::VIOLATES_HARD_LIMIT_OMEGA:
      result = RESPONSE::VIOLATES_HARD_LIMIT_OMEGA; break;
    case Validator::VIOLATES_HARD_LIMIT_ALPHA:
      result = RESPONSE::VIOLATES_HARD_LIMIT_ALPHA; break;
    default:
      result = RESPONSE::SUCCESS; break;
    }
    return Result(result);
  }

  // Complete the current dock or undock action
  int32_t Result(int32_t response) {
    ff_msgs::MotionResult result;
    // Teardown any child services correctly
    switch (response) {
    // Do nothing for these states
    default:
    case RESPONSE::ALREADY_THERE:
      result.fsm_result = "Already on the requested setpoint";          break;
    case RESPONSE::SUCCESS:
      result.fsm_result = "Motion completed successfully";              break;
    case RESPONSE::PLAN_FAILED:
      result.fsm_result = "Unable to plan a segment";                   break;
    case RESPONSE::CONTROL_FAILED:
      result.fsm_result = "The control action failed";                  break;
    case RESPONSE::NOT_IN_WAITING_MODE:
      result.fsm_result = "Choreographer not in WAITING mode";          break;
    case RESPONSE::INVALID_FLIGHT_MODE:
      result.fsm_result = "Unknown flight mode";                        break;
    case RESPONSE::UNEXPECTED_EMPTY_SEGMENT:
      result.fsm_result = "Unexpected empty segment";                   break;
    case RESPONSE::COULD_NOT_RESAMPLE:
      result.fsm_result = "Could not resample segment";                 break;
    case RESPONSE::UNEXPECTED_EMPTY_STATES:
      result.fsm_result = "Unexpected empty set of setpoints";          break;
    case RESPONSE::INVALID_COMMAND:
      result.fsm_result = "Unknown command";                            break;
    case RESPONSE::CANNOT_QUERY_ROBOT_POSE:
      result.fsm_result = "Cannot query the robot pose";                break;
    case RESPONSE::NOT_ON_FIRST_POSE:
      result.fsm_result = "Not on first setpoint. No bootstrapping";    break;
    case RESPONSE::BAD_DESIRED_VELOCITY:
      result.fsm_result = "Velocity exceeds hard limit";                break;
    case RESPONSE::BAD_DESIRED_ACCELERATION:
      result.fsm_result = "Acceleration exceeds hard limit";            break;
    case RESPONSE::BAD_DESIRED_OMEGA:
      result.fsm_result = "Angular velocity exceeds hard limit";        break;
    case RESPONSE::BAD_DESIRED_ALPHA:
      result.fsm_result = "Angular acceleration exceeds hard limit";    break;
    case RESPONSE::BAD_DESIRED_RATE:
      result.fsm_result = "Sample rate below hard minimum";             break;
    // Called in parallel to an active controller
    case RESPONSE::REVALIDATE_FAILED:
      result.fsm_result = "Replanned segment is not valid";
      client_c_.CancelGoal();
      break;
    case RESPONSE::OBSTACLE_DETECTED:
      result.fsm_result = "Segment collides with a mapped obstacle";
      client_c_.CancelGoal();
      break;
    case RESPONSE::REPLAN_NOT_ENOUGH_TIME:
      result.fsm_result = "Not enough time available to replan";
      client_c_.CancelGoal();
      break;
    case RESPONSE::REPLAN_FAILED:
      result.fsm_result = "Replanning failed";
      client_c_.CancelGoal();
      break;
    case RESPONSE::TOLERANCE_VIOLATION_POSITION:
      result.fsm_result = "Position tolerance violated";
      client_c_.CancelGoal();
      break;
    case RESPONSE::TOLERANCE_VIOLATION_ATTITUDE:
      result.fsm_result = "Attitude tolerance violated";
      client_c_.CancelGoal();
      break;
    case RESPONSE::TOLERANCE_VIOLATION_VELOCITY:
      result.fsm_result = "Velocity tolerance violated";
      client_c_.CancelGoal();
      break;
    case RESPONSE::TOLERANCE_VIOLATION_OMEGA:
      result.fsm_result = "Angular velocity tolerance violated";
      client_c_.CancelGoal();
      break;
    // Failures before control has been started
    case RESPONSE::VIOLATES_MINIMUM_FREQUENCY:
      result.fsm_result = "Could not resample at 10Hz to do zone checking";
      break;
    case RESPONSE::VIOLATES_KEEP_OUT:
      result.fsm_result = "Keep out zone violation";
      break;
    case RESPONSE::VIOLATES_KEEP_IN:
      result.fsm_result = "Keep in zone violation";
      break;
    case RESPONSE::VIOLATES_STATIONARY_ENDPOINT:
      result.fsm_result = "Last setpoint in segment must be stationery";
      break;
    case RESPONSE::VIOLATES_FIRST_IN_PAST:
      result.fsm_result = "Your first setpoint cannot be in the past";
      break;
    case RESPONSE::VIOLATES_MINIMUM_SETPOINTS:
      result.fsm_result = "You need at least two setpoints in a segment";
      break;
    case RESPONSE::VIOLATES_HARD_LIMIT_VEL:
      result.fsm_result = "Velocity hard limit violated";
      break;
    case RESPONSE::VIOLATES_HARD_LIMIT_ACCEL:
      result.fsm_result = "Acceleration hard limit violated";
      break;
    case RESPONSE::VIOLATES_HARD_LIMIT_OMEGA:
      result.fsm_result = "Omega hard limit violated";
      break;
    case RESPONSE::VIOLATES_HARD_LIMIT_ALPHA:
      result.fsm_result = "Alpha hard limit violated";
      break;
    // Preemptions or cancellations
    case RESPONSE::CANCELLED:
    case RESPONSE::PREEMPTED:
      result.fsm_result = "Operation interrupted either by callee or third party";
      switch (fsm_.GetState()) {
      // Planning
      case STATE::REPLANNING:
        client_c_.CancelGoal();
      case STATE::BOOTSTRAPPING:
      case STATE::PLANNING:
        {
          std::string planner = cfg_.Get<std::string>("planner");
          if (planners_.find(planner) != planners_.end())
            planners_[planner].CancelGoal();
        }
        break;
      // Controlling
      case STATE::CONTROLLING:
        client_c_.CancelGoal();
        break;
      // Do nothing
      case STATE::IDLING:
      case STATE::STOPPING:
      case STATE::HALTING:
        break;
      }
      break;
    }
    // If we get here then we are in a valid action state, so we will need
    // to produce a meaningful result for the callee.
    result.response = response;
    result.segment = segment_;
    result.flight_mode = flight_mode_;
    if (response > 0)
      server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    else if (response < 0)
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
    else
      server_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
    // Special case: IDLING needs to move to an IDLE state
    if (fsm_.GetState() == STATE::IDLING && response == RESPONSE::SUCCESS)
      return STATE::IDLE;
    // General case: move back to a stopped state
    return STATE::STOPPED;
  }

  // Get the pose of the robot in the world frame
  bool GetRobotPose(geometry_msgs::PoseStamped & pose) {
    std::string child_frame = std::string(FRAME_NAME_BODY);
    if (!GetPlatform().empty())
      child_frame = GetPlatform() + "/" + child_frame;
    try {
      geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
        std::string(FRAME_NAME_WORLD), child_frame, ros::Time(0));
      pose.header = tf.header;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
    }
    catch (tf2::TransformException &ex) {
      NODELET_WARN_STREAM("Transform failed" << ex.what());
      return false;
    }
    return true;
  }

  // When the FSM state changes we get a callback here, so that we can send
  // feedback to any active client, print debug info and publish our state
  void UpdateCallback(FSM::State const& state, FSM::Event const& event) {
    // Debug events
    ff_msgs::MotionState msg;
    msg.header.frame_id = GetPlatform();
    msg.header.stamp = ros::Time::now();
    msg.state = state;
    switch (event) {
    case READY:                   msg.fsm_event = "READY";              break;
    case GOAL_EXEC:               msg.fsm_event = "GOAL_EXEC";          break;
    case GOAL_MOVE:               msg.fsm_event = "GOAL_MOVE";          break;
    case GOAL_STOP:               msg.fsm_event = "GOAL_STOP";          break;
    case GOAL_IDLE:               msg.fsm_event = "GOAL_IDLE";          break;
    case GOAL_PREP:               msg.fsm_event = "GOAL_PREP";          break;
    case GOAL_CANCEL:             msg.fsm_event = "GOAL_CANCEL";        break;
    case PMC_READY:               msg.fsm_event = "PMC_READY";          break;
    case PMC_TIMEOUT:             msg.fsm_event = "PMC_TIMEOUT";        break;
    case PLAN_SUCCESS:            msg.fsm_event = "PLAN_SUCCESS";       break;
    case PLAN_FAILED:             msg.fsm_event = "PLAN_FAILED";        break;
    case CONTROL_SUCCESS:         msg.fsm_event = "CONTROL_SUCCESS";    break;
    case CONTROL_FAILED:          msg.fsm_event = "CONTROL_FAILED";     break;
    case TOLERANCE_POS:           msg.fsm_event = "TOLERANCE_POS";      break;
    case TOLERANCE_ATT:           msg.fsm_event = "TOLERANCE_ATT";      break;
    case TOLERANCE_VEL:           msg.fsm_event = "TOLERANCE_VEL";      break;
    case TOLERANCE_OMEGA:         msg.fsm_event = "TOLERANCE_OMEGA";    break;
    case OBSTACLE_DETECTED:       msg.fsm_event = "OBSTACLE_DETECTED";  break;
    case MANUAL_STATE_SET:        msg.fsm_event = "MANUAL_STATE_SET";   break;
    }
    // Debug state changes
    switch (state) {
    case STATE::INITIALIZING:     msg.fsm_state = "INITIALIZING";       break;
    case STATE::IDLE:             msg.fsm_state = "IDLE";               break;
    case STATE::STOPPED:          msg.fsm_state = "STOPPED";            break;
    case STATE::IDLING:           msg.fsm_state = "IDLING";             break;
    case STATE::STOPPING:         msg.fsm_state = "STOPPING";           break;
    case STATE::PREPPING:         msg.fsm_state = "PREPPING";           break;
    case STATE::BOOTSTRAPPING:    msg.fsm_state = "BOOTSTRAPPING";      break;
    case STATE::PLANNING:         msg.fsm_state = "PLANNING";           break;
    case STATE::PREPARING:        msg.fsm_state = "PREPARING";          break;
    case STATE::CONTROLLING:      msg.fsm_state = "CONTROLLING";        break;
    case STATE::REPLANNING:       msg.fsm_state = "REPLANNING";         break;
    case STATE::HALTING:          msg.fsm_state = "HALTING";            break;
    }
    // Publish the state
    pub_state_.publish(msg);
    // Debug information for the nodelet
    NODELET_DEBUG_STREAM("Received event " << msg.fsm_event);
    NODELET_DEBUG_STREAM("State changed to " << msg.fsm_state);
    // Send the feedback if needed
    switch (state) {
    case STATE::IDLING:
    case STATE::STOPPING:
    case STATE::HALTING:
    case STATE::BOOTSTRAPPING:
    case STATE::PLANNING:
    case STATE::PREPARING:
    case STATE::CONTROLLING:
    case STATE::REPLANNING: {
      feedback_.state = msg;
      server_.SendFeedback(feedback_);
      break;
    }
    default:
      break;
    }
  }

  // COLLISION

  // Called when the sentinel forecasts an upcoming collision
  void HazardCallback(ff_msgs::Hazard::ConstPtr const& msg) {
    if (!cfg_.Get<bool>("enable_collision_checking"))
      return;
    return fsm_.Update(OBSTACLE_DETECTED);
  }

  // PLAN

  // Initiate the planning of a segment
  bool Plan(std::vector<geometry_msgs::PoseStamped> const& states,
    ros::Duration duration = ros::Duration(0)) {
    // Divide by a constant in holonomic mode to avoid saturation
    double divider = 1.0;
    if (!cfg_.Get<bool>("enable_faceforward"))
      divider = 2.0;
    // Package up a skeleton plan request
    static ff_msgs::PlanGoal plan_goal;
    plan_goal.states = states;
    plan_goal.faceforward = cfg_.Get<bool>("enable_faceforward");
    plan_goal.desired_vel = cfg_.Get<double>("desired_vel");
    plan_goal.desired_accel = cfg_.Get<double>("desired_accel") / divider;
    plan_goal.desired_omega = cfg_.Get<double>("desired_omega");
    plan_goal.desired_alpha = cfg_.Get<double>("desired_alpha") / divider;
    plan_goal.desired_rate = cfg_.Get<double>("desired_rate");
    // Check desired velocity
    if (plan_goal.desired_vel < 0) {
      plan_goal.desired_vel = goal_flight_mode_.hard_limit_vel;
    } else if (plan_goal.desired_vel > goal_flight_mode_.hard_limit_vel) {
      NODELET_WARN_STREAM("Velocity violated " << goal_flight_mode_.name);
      return false;
    }
    // Check desired acceleration
    if (plan_goal.desired_accel < 0) {
      plan_goal.desired_accel = goal_flight_mode_.hard_limit_accel / divider;
    } else if (plan_goal.desired_accel > goal_flight_mode_.hard_limit_accel / divider) {
      NODELET_WARN_STREAM("Accel violated " << goal_flight_mode_.name);
      return false;
    }
    // Check desired omega
    if (plan_goal.desired_omega < 0) {
      plan_goal.desired_omega = goal_flight_mode_.hard_limit_omega;
    } else if (plan_goal.desired_omega > goal_flight_mode_.hard_limit_omega) {
      NODELET_WARN_STREAM("Omega violated " << goal_flight_mode_.name);
      return false;
    }
    // Check  desired alpha
    if (plan_goal.desired_alpha < 0) {
      plan_goal.desired_alpha = goal_flight_mode_.hard_limit_alpha / divider;
    } else if (plan_goal.desired_alpha > goal_flight_mode_.hard_limit_alpha / divider) {
      NODELET_WARN_STREAM("Alpha violated " << goal_flight_mode_.name);
      return false;
    }
    // Check control frequency
    if (plan_goal.desired_rate < 0) {
      plan_goal.desired_rate = ff_util::FlightUtil::MIN_CONTROL_RATE;
    } else if (plan_goal.desired_rate < ff_util::FlightUtil::MIN_CONTROL_RATE) {
      NODELET_WARN_STREAM("Rate violated " << goal_flight_mode_.name);
      return false;
    }
    // Make sure we communicate the maximum allowed time to the planner
    if (duration > ros::Duration(0))
      plan_goal.max_time = duration;
    else
      plan_goal.max_time = ros::Duration(
        cfg_.Get<double>("timeout_plan_deadline"));
    // Find and send to the planner
    std::string planner = cfg_.Get<std::string>("planner");
    if (planners_.find(planner) != planners_.end()) {
      planners_[planner].SetDeadlineTimeout(plan_goal.max_time.toSec());
      if (!planners_[planner].SendGoal(plan_goal)) {
        NODELET_WARN_STREAM("Planner rejected goal");
        return false;
      }
      return true;
    }
    // Planner does not exist
    NODELET_WARN_STREAM("Planner does not exist");
    return false;
  }

  // Planner feedback - simply forward to motion feeedback
  void PFeedbackCallback(ff_msgs::PlanFeedbackConstPtr const& feedback) {
    switch (fsm_.GetState()) {
    case STATE::BOOTSTRAPPING:
    case STATE::PLANNING:
    case STATE::REPLANNING:
      feedback_.perc_complete = feedback->perc_complete;
      feedback_.secs_remaining = feedback->secs_remaining;
      return server_.SendFeedback(feedback_);
    default:
      break;
    }
  }

  // Planner result -- trigger an update to the FSM
  void PResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::PlanResultConstPtr const& result) {
    if (result_code ==  ff_util::FreeFlyerActionState::SUCCESS) {
      switch (fsm_.GetState()) {
      // If the result from the boostrap is ALREADY_THERE, then we expect
      // the segment to be empty / invalid. So, only in the case where we
      // actually have a boostrapped move to prepend to the segment, do so.
      case STATE::BOOTSTRAPPING:
        if (result->response == RESPONSE::SUCCESS) {
          ros::Duration tdiff = result->segment.back().when
                              - segment_.front().when;
          for (ff_util::Segment::iterator it = segment_.begin();
            it != segment_.end(); it++) it->when += tdiff;
          segment_.insert(segment_.begin(),
            result->segment.begin(), result->segment.end());
        }
        break;
      // For the case of planning or replanning, just copy over the segment.
      case STATE::PLANNING:
      case STATE::REPLANNING:
        segment_ = result->segment;
      default:
        break;
      }
      return fsm_.Update(PLAN_SUCCESS);
    }
    return fsm_.Update(PLAN_FAILED);
  }

  // SPEED

  // Send the flight mode out to subscribers
  bool FlightMode() {
    // Always publish the new flight mode
    pub_flight_mode_.publish(goal_flight_mode_);
    // In the case of a speed mismatch we need to wait for prep
    if (goal_flight_mode_.speed != flight_mode_.speed) {
      // Start timers
      timer_speed_.start();
      timer_feedback_.start();
      return true;
    }
    // Prep not required, so set new flight mode immediately
    flight_mode_ = goal_flight_mode_;
    return false;
  }

  // Called when the sentinel forecasts an upcoming collision
  void PmcStateCallback(ff_hw_msgs::PmcState::ConstPtr const& msg) {
    // If prepping, send a state update to prevent execute and move
    // actions from timing out on the client side
    if (fsm_.GetState() == STATE::PREPPING ||
        fsm_.GetState() == STATE::PREPARING)
      server_.SendFeedback(feedback_);
    // Check if the PMCs are ready
    bool ready = true;
    for (size_t i = 0; i < msg->states.size() && ready; i++)
      ready &= (msg->states[i] == ff_hw_msgs::PmcState::READY);
    if (!ready)
      return;
    // Stop the timers
    timer_speed_.stop();
    timer_feedback_.stop();
    // Set the new flight mode to indicate prep is complete
    flight_mode_ = goal_flight_mode_;
    // We are now prepped and ready to fly
    return fsm_.Update(PMC_READY);
  }

  // Called if the prep fails
  void PmcTimeout(const ros::TimerEvent&) {
    return fsm_.Update(PMC_TIMEOUT);
  }

  // Called to fake feedback to executive during a prep
  void FeedbackCallback(const ros::TimerEvent&) {
    return server_.SendFeedback(feedback_);
  }

  // CONTROL

  // Send specific control
  bool Control(ControlType type,
    ff_util::Segment const& segment = ff_util::Segment()) {
    // Package and send the segment to control
    ff_msgs::ControlGoal goal;
    switch (type) {
    default:
      return false;
    case STOP: goal.command = ff_msgs::ControlGoal::STOP; break;
    case IDLE: goal.command = ff_msgs::ControlGoal::IDLE; break;
    case NOMINAL:
      // No point in executing an empty segment
      if (segment.empty())
        return false;
      // Populate the basics
      goal.command = ff_msgs::ControlGoal::NOMINAL;
      goal.segment = segment;
      // If enable immediate is turned on, then shift the setpoint timestamps
      //  so that the first setpoint coincides with the current time.
      ros::Time reftime = goal.segment.front().when;
      if (cfg_.Get<bool>("enable_immediate"))
        reftime = ros::Time::now();
      // Timesync was an attempt to synchronize Astrobees without Astrobee to
      // Astrobee communication. It is not recommended to enable it since it
      // hasn't been tested and the synchronization protocol is not robust; one
      // can not guarentee that the different Astrobees will receive the
      // command within the same discrete time unit window.
      if (cfg_.Get<bool>("enable_timesync")) {
        reftime.sec += cfg_.Get<int>("discrete_time_unit");
        reftime.sec += reftime.sec % cfg_.Get<int>("discrete_time_unit");
        reftime.nsec = 0;
      }
      // Calculate the difference
      ros::Duration diff = reftime - goal.segment.front().when;
      NODELET_DEBUG_STREAM("Time shift:" << diff);
      // Now shift the timestamps are all setpoints accordingly
      for (ff_util::Segment::iterator it = goal.segment.begin();
        it != goal.segment.end(); it++) it->when += diff;
      // We are now ready to execute the segment
      break;
    }
    // Send the goal
    if (!client_c_.SendGoal(goal))
      return false;
    // Publish the segment to rviz for user introspection
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    path.poses.reserve(goal.segment.size());
    geometry_msgs::PoseStamped ps;
    ps.header = path.header;
    for (ff_util::Segment::iterator it = goal.segment.begin();
      it != goal.segment.end(); it++) {
      ps.header.stamp = it->when;
      ps.pose = it->pose;
      path.poses.push_back(ps);
    }
    pub_segment_.publish(path);
    // Success!
    return true;
  }

  // Control feedback - simple pass through for control -> feedback
  void CFeedbackCallback(ff_msgs::ControlFeedbackConstPtr const& feedback) {
    switch (fsm_.GetState()) {
    // Perform tolerance checking while controling
    case STATE::CONTROLLING:
      if (flight_mode_.tolerance_pos > 0.0 &&
          feedback->error_position > flight_mode_.tolerance_pos) {
        // If tolerance is present more that the allowable time
        if ((ros::Time::now() - tolerance_pos_timer).toSec() > tolerance_max_time_) {
          NODELET_DEBUG_STREAM("Position tolerance violated");
          NODELET_DEBUG_STREAM("- Value: " << feedback->error_position
                                          << ", Thresh: "
                                          << flight_mode_.tolerance_pos);
          fsm_.Update(TOLERANCE_POS);
          return;
        }
      } else {
        // If there is no tolerance violation, reset time
        tolerance_pos_timer = ros::Time::now();
      }
      // Check attitude tolerance
      if (flight_mode_.tolerance_att > 0.0 &&
          feedback->error_attitude > flight_mode_.tolerance_att) {
        // If tolerance is present more that the allowable time
        if ((ros::Time::now() - tolerance_att_timer).toSec() > tolerance_max_time_) {
          NODELET_DEBUG_STREAM("Attitude tolerance violated");
          NODELET_DEBUG_STREAM("- Value: " << feedback->error_attitude
                                          << ", Thresh: "
                                          << flight_mode_.tolerance_att);
          fsm_.Update(TOLERANCE_ATT);
          return;
        }
      } else {
        // If there is no tolerance violation, reset time
        tolerance_att_timer = ros::Time::now();
      }
      // Check velocity tolerance
      if (flight_mode_.tolerance_vel > 0.0 &&
          feedback->error_velocity > flight_mode_.tolerance_vel) {
        // If tolerance is present more that the allowable time
        if ((ros::Time::now() - tolerance_vel_timer).toSec() > tolerance_max_time_) {
          NODELET_DEBUG_STREAM("Velocity tolerance violated");
          NODELET_DEBUG_STREAM("- Value: " << feedback->error_velocity
                                          << ", Thresh: "
                                          << flight_mode_.tolerance_vel);
          fsm_.Update(TOLERANCE_VEL);
          return;
        }
      } else {
        // If there is no tolerance violation, reset time
        tolerance_vel_timer = ros::Time::now();
      }
      // Check angular velocity tolerance
      if (flight_mode_.tolerance_omega > 0.0 &&
          feedback->error_omega > flight_mode_.tolerance_omega) {
        // If tolerance is present more that the allowable time
        if ((ros::Time::now() - tolerance_omega_timer).toSec() > tolerance_max_time_) {
          NODELET_DEBUG_STREAM("Angular velocity tolerance violated");
          NODELET_DEBUG_STREAM("- Value: " << feedback->error_omega
                                          << ", Thresh: "
                                          << flight_mode_.tolerance_omega);
          fsm_.Update(TOLERANCE_OMEGA);
          return;
        }
      } else {
        // If there is no tolerance violation, reset time
        tolerance_omega_timer = ros::Time::now();
      }
    // Send progress in stopping/idling/replanning
    case STATE::STOPPING:
    case STATE::IDLING:
    case STATE::REPLANNING:
      feedback_.progress = *feedback;
      return server_.SendFeedback(feedback_);
    default:
      break;
    }
  }

  // Control result
  void CResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::ControlResultConstPtr const& result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      segment_ = result->segment;
      return fsm_.Update(CONTROL_SUCCESS);
    default:
      return fsm_.Update(CONTROL_FAILED);
    }
  }

  // MOTION ACTION SERVER

  void GoalCallback(ff_msgs::MotionGoalConstPtr const& goal) {
    ff_msgs::MotionResult result;
    // We can only accept new commands if we are currently in an idle state.
    // This should be the case if Preempt() was called beforehand -- as it
    // should be automatically -- in the case of preemption.
    if (fsm_.GetState() != STATE::IDLE &&
        fsm_.GetState() != STATE::STOPPED) {
      result.response = RESPONSE::NOT_IN_WAITING_MODE;
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
      return;
    }
    // If the specified flight mode string is empty then we wont try and
    // change the flight mode from what it currently is.
    goal_flight_mode_ = flight_mode_;
    if (!goal->flight_mode.empty()) {
      if (!ff_util::FlightUtil::GetFlightMode(
        goal_flight_mode_, goal->flight_mode)) {
        result.response = RESPONSE::INVALID_FLIGHT_MODE;
        server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
        return;
      }
    }
    // What we do now depends on the command that was sent
    switch (goal->command) {
    case ff_msgs::MotionGoal::STOP:
      NODELET_DEBUG_STREAM("Received new STOP command");
      return fsm_.Update(GOAL_STOP);
    case ff_msgs::MotionGoal::IDLE:
      NODELET_DEBUG_STREAM("Received new IDLE command");
      return fsm_.Update(GOAL_IDLE);
    case ff_msgs::MotionGoal::PREP:
      NODELET_DEBUG_STREAM("Received new PREP command");
      return fsm_.Update(GOAL_PREP);
    case ff_msgs::MotionGoal::EXEC:
      NODELET_DEBUG_STREAM("Received new EXEC command");
      // We need a valid segment to execute
      if (goal->segment.empty()) {
        result.response = RESPONSE::UNEXPECTED_EMPTY_SEGMENT;
        server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
        return;
      }
      // Zach's planner in GDS (as well as other planners) do not respect
      // the minimum control period required by GNC. This segment of code
      // numerically resamples the input segment, so control accepts it.
      if (ff_util::FlightUtil::Resample(goal->segment, segment_)
        != ff_util::SUCCESS) {
        result.response = RESPONSE::COULD_NOT_RESAMPLE;
        server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
        return;
      }
      // We can now start the execute process
      return fsm_.Update(GOAL_EXEC);
    case ff_msgs::MotionGoal::MOVE:
      // We need valid states to move through
      if (goal->states.empty()) {
        result.response = RESPONSE::UNEXPECTED_EMPTY_STATES;
        server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
        return;
      }
      // Save the states
      states_ = goal->states;
      // If the reference frame is not empty,
      // we need to transform the states to the correct frame
      if (!goal->reference_frame.empty()) {
        geometry_msgs::TransformStamped tfs;
        try {
          // transform from reference frame to FRAME_NAME_WORLD
          tfs = tf_buffer_.lookupTransform(
            FRAME_NAME_WORLD,
            goal->reference_frame,
            ros::Time(0));
        } catch (tf2::TransformException &ex) {
          result.response = RESPONSE::INVALID_REFERENCE_FRAME;
          result.fsm_result = "Invalid reference frame";
          break;
        }
        for (uint i = 0; i < states_.size(); i++)
            tf2::doTransform(states_[i], states_[i], tfs);  // frame is now FRAME_NAME_WORLD
      }
      // Start the move
      return fsm_.Update(GOAL_MOVE);
    default:
      break;
    }
    // A catch-all for any mistakenly unhandled commands
    result.response = RESPONSE::INVALID_COMMAND;
    server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
  }

  // Preempt the current action with a new action
  void PreemptCallback() {
    return fsm_.Update(GOAL_CANCEL);
  }

  // A Cancellation request arrives
  void CancelCallback() {
    return fsm_.Update(GOAL_CANCEL);
  }

 private:
  // Action managemement
  ff_util::FSM fsm_;
  ff_util::FreeFlyerActionClient<ff_msgs::ControlAction> client_c_;
  ff_util::FreeFlyerActionServer<ff_msgs::MotionAction> server_;
  ff_msgs::MotionFeedback feedback_;
  // Runtime configuration
  ff_util::ConfigServer cfg_;
  // Zone management`
  Validator validator_;
  // Timeout on speed preps
  ros::Timer timer_speed_, timer_feedback_;
  // TF2
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // Planner registration
  PlannerMap planners_;
  PlannerInfo info_;
  // Publishers and subscribers
  ros::Publisher pub_state_, pub_segment_, pub_flight_mode_, pub_inertia_;
  ros::Subscriber sub_hazard_, sub_pmc_state_;
  ros::ServiceServer server_register_, server_set_state_, server_set_inertia_;
  // Cached between callbacks
  ff_msgs::FlightMode flight_mode_, goal_flight_mode_;  // Flight mode
  std::vector<geometry_msgs::PoseStamped> states_;      // Plan request
  ff_util::Segment segment_;                            // Segment
  geometry_msgs::PointStamped obstacle_;                // Obstacle
  // Tolerance check Timers
  double tolerance_max_time_;
  ros::Time tolerance_pos_timer, tolerance_att_timer,
                tolerance_vel_timer, tolerance_omega_timer;
  // Cached number of replan attempts
  int replan_attempts_;
};

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(choreographer::ChoreographerNodelet, nodelet::Nodelet);

}  // namespace choreographer
