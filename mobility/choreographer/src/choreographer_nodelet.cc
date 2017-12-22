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
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Inertia.h>

// Hardware messages
#include <ff_hw_msgs/PmcState.h>

// Messages
#include <ff_msgs/MotionState.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/ControlCommand.h>

// Services
#include <ff_msgs/RegisterPlanner.h>
#include <ff_msgs/SetState.h>

// Actions
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/ControlAction.h>
#include <ff_msgs/PlanAction.h>
#include <ff_msgs/ValidateAction.h>

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

// Data structure for managing planner registration
typedef std::map<std::string, std::string > PlannerInfo;
typedef std::map<std::string,
  ff_util::FreeFlyerActionClient<ff_msgs::PlanAction>> PlannerMap;

// The choreograph class manages core logic for mobility
class ChoreographerNodelet : public ff_util::FreeFlyerNodelet {
 public:
  enum ChoreographerEvent {
    READY,                             // System is initialized
    GOAL_EXEC,                         // New execute command
    GOAL_MOVE,                         // New move command
    GOAL_STOP,                         // New stop command
    GOAL_IDLE,                         // New idle command
    GOAL_PREP,                         // New prep command
    GOAL_CANCEL,                       // Cancel existing goal
    VALIDATE_SUCCESS,                  // Segment validation success
    VALIDATE_FAILED,                   // Segment validation failed
    PLAN_SUCCESS,                      // Segment plan success
    PLAN_FAILED,                       // Segment plan failed
    PMC_READY,                         // PMC is ramped and ready
    PMC_TIMEOUT,                       // PMC has timed out
    CONTROL_SUCCESS,                   // Control success
    CONTROL_FAILED,                    // Control failure
    TOLERANCE_POS,                     // Tolerance failure
    TOLERANCE_ATT,                     // Tolerance failure
    TOLERANCE_VEL,                     // Tolerance failure
    TOLERANCE_OMEGA,                   // Tolerance failure
    OBSTACLE_DETECTED                  // Obstacle detected
  };

  // The various types of control
  enum ControlType { IDLE, STOP, NOMINAL };

  // Constructor
  ChoreographerNodelet() : ff_util::FreeFlyerNodelet(NODE_CHOREOGRAPHER, true),
    fsm_(STATE::INITIALIZING, std::bind(&ChoreographerNodelet::UpdateCallback,
      this, std::placeholders::_1, std::placeholders::_2)) {
    // Add the state transition lambda functions - refer to the FSM diagram
    // [0]
    fsm_.Add(READY,                               // These events move...
      STATE::INITIALIZING,                        // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        if (!Control(STOP))
          AssertFault("INITIALIZATION_FAULT", "Cannot call Stop() action");
        return STATE::WAITING_FOR_STOP;            // The next state...
      });
    // [1]
    fsm_.Add(CONTROL_SUCCESS, CONTROL_FAILED,     // These events move...
      STATE::WAITING_FOR_STOP,                    // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        if (event == CONTROL_FAILED)
          AssertFault("INITIALIZATION_FAULT", "Stop() goal failed on init");
        return STATE::WAITING;
      });
    // [2]
    fsm_.Add(GOAL_EXEC,                           // These events move...
      STATE::WAITING,                             // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        // Assemble a simple planning request
        geometry_msgs::PoseStamped pose;
        if (!GetRobotPose(pose))
          return Result(RESPONSE::CANNOT_QUERY_ROBOT_POSE);
        // Get the robot pose
        if (ff_util::FlightUtil::WithinTolerance(
          flight_mode_, segment_.front().pose, pose.pose)) {
          // If we need to validate
          if (cfg_.Get<bool>("enable_validation")) {
            if (!Validate(segment_))
              return Result(RESPONSE::VALIDATE_FAILED);
            return STATE::VALIDATING;
          }
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
    fsm_.Add(PLAN_SUCCESS,                        // These events move...
      STATE::BOOTSTRAPPING,                       // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        // If we need to validate
        if (cfg_.Get<bool>("enable_validation")) {
          if (!Validate(segment_))
            return Result(RESPONSE::VALIDATE_FAILED);
          return STATE::VALIDATING;
        }
        // If we send control directly
        if (!Control(NOMINAL, segment_))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::CONTROLLING;
      });
    // [4]
    fsm_.Add(PLAN_FAILED, GOAL_CANCEL,            // These events move...
      STATE::BOOTSTRAPPING,                       // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::PLAN_FAILED);
      });
    // [5]
    fsm_.Add(GOAL_MOVE,                           // These events move...
      STATE::WAITING,                             // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        // Assemble a simple planning request
        geometry_msgs::PoseStamped pose;
        // Get the robot pose
        if (!GetRobotPose(pose))
          return Result(RESPONSE::CANNOT_QUERY_ROBOT_POSE);
        // Cover a special case where we are already on ALL desired poses.
        bool aligned = true;
        std::vector<geometry_msgs::PoseStamped>::iterator it;
        for (it = states_.begin(); it != states_.end() && aligned; ++it)
          aligned &= ff_util::FlightUtil::WithinTolerance(
            flight_mode_, pose.pose, it->pose);
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
    fsm_.Add(PLAN_SUCCESS,                        // These events move...
      STATE::PLANNING,                            // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        // If we need to validate
        if (cfg_.Get<bool>("enable_validation")) {
          if (!Validate(segment_))
            return Result(RESPONSE::VALIDATE_FAILED);
          return STATE::VALIDATING;
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
    fsm_.Add(PLAN_FAILED, GOAL_CANCEL,            // These events move...
      STATE::PLANNING,                            // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::PLAN_FAILED);
      });
    // [8]
    fsm_.Add(VALIDATE_SUCCESS,                    // These events move...
      STATE::VALIDATING,                          // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        // If this returns false then we are already on the correct speed gain
        if (FlightMode())
          return STATE::PREPARING;
        // Move straight to control, as we don't need to wait for speed prep.
        if (!Control(NOMINAL, segment_))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::CONTROLLING;
      });
    // [9]
    fsm_.Add(VALIDATE_FAILED, GOAL_CANCEL,        // These events move...
      STATE::VALIDATING,                          // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::VALIDATE_FAILED);
      });
    // [10]
    fsm_.Add(PLAN_SUCCESS,                        // These events move...
      STATE::REPLANNING,                          // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        // If we need to validate the segment
        if (cfg_.Get<bool>("enable_validation")) {
          if (!Validate(segment_))
            return Result(RESPONSE::VALIDATE_FAILED);
          return STATE::REVALIDATING;
        }
        // If we send control directly
        if (!Control(NOMINAL, segment_))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::CONTROLLING;
      });
    // [11]
    fsm_.Add(VALIDATE_SUCCESS,                    // These events move...
      STATE::REVALIDATING,                        // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        // If we send control directly
        if (!Control(NOMINAL, segment_))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::CONTROLLING;
      });
    // [12]
    fsm_.Add(VALIDATE_FAILED, GOAL_CANCEL,        // These events move...
      STATE::REVALIDATING,                        // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        Control(STOP);
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::REVALIDATE_FAILED);
      });
    // [13]
    fsm_.Add(CONTROL_SUCCESS,                     // These events move...
      STATE::CONTROLLING,                         // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        return Result(RESPONSE::SUCCESS);         // The next state...
      });
    // [14]
    fsm_.Add(PLAN_FAILED, GOAL_CANCEL,            // These events move...
      STATE::REPLANNING,                          // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        Control(STOP);
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::REPLAN_FAILED);
      });
    // [15]
    fsm_.Add(CONTROL_FAILED, GOAL_CANCEL,         // These events move...
      STATE::CONTROLLING,                         // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        Control(STOP);
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::CONTROL_FAILED);
      });
    // [28]
    fsm_.Add(TOLERANCE_POS, TOLERANCE_ATT, TOLERANCE_VEL, TOLERANCE_OMEGA,
      STATE::CONTROLLING,
      [this](ChoreographerEvent const& event) -> int32_t {
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
    fsm_.Add(OBSTACLE_DETECTED,                   // These events move...
      STATE::CONTROLLING,                         // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        // If we need to validate the segment
        if (cfg_.Get<bool>("enable_replanning")) {
          // Find a suitable handover point between now and the collision time
          ros::Time handover = obstacle_.header.stamp
                             - ros::Duration(cfg_.Get<double>("time_buffer"));
          // Find the setpoint immediately before the handover time
          ff_util::Segment::reverse_iterator it;
          for (it = segment_.rbegin(); it != segment_.rend(); ++it)
            if (it->when < handover) break;
          // Check that the handover time is after the current time
          if (it == segment_.rend() || it->when < ros::Time::now()) {
            Control(STOP);
            return Result(RESPONSE::REPLAN_NOT_ENOUGH_TIME);
          }
          // Assemble a simple planning request
          std::vector<geometry_msgs::PoseStamped> states;
          geometry_msgs::PoseStamped pose;
          // Replan from the handover point to the end of the segment
          pose.header.stamp = it->when;
          pose.pose = it->pose;
          states.push_back(pose);
          pose.header.stamp = segment_.back().when;
          pose.pose = segment_.back().pose;
          states.push_back(pose);
          // Now, generate a plan between the supplied states
          if (!Plan(states, ros::Time::now() - it->when))
            return Result(RESPONSE::PLAN_FAILED);
          return STATE::REPLANNING;
        }
        // If replanning is disabled then this is just an obstacle detection
        return Result(RESPONSE::OBSTACLE_DETECTED);
      });
    // [17]
    fsm_.Add(GOAL_STOP,                           // These events move...
      STATE::WAITING,                             // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        // Force the flight mode  change and immediately send control
        FlightMode();
        if (!Control(STOP))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::STOPPING;                   // The next state...
      });
    // [18]
    fsm_.Add(CONTROL_SUCCESS,                     // These events move...
      STATE::STOPPING,                            // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        return Result(RESPONSE::SUCCESS);         // The next state...
      });
    // [19]
    fsm_.Add(CONTROL_FAILED, GOAL_CANCEL,         // These events move...
      STATE::STOPPING,                            // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::CONTROL_FAILED);
      });
    // [20]
    fsm_.Add(GOAL_IDLE,                           // These events move...
      STATE::WAITING,                             // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        // Force the flight mode  change and immediately send control
        FlightMode();
        if (!Control(IDLE))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::IDLING;                     // The next state...
      });
    // [21]
    fsm_.Add(CONTROL_SUCCESS,                     // These events move...
      STATE::IDLING,                              // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        return Result(RESPONSE::SUCCESS);         // The next state...
      });
    // [22]
    fsm_.Add(CONTROL_FAILED, GOAL_CANCEL,         // These events move...
      STATE::IDLING,                              // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::CONTROL_FAILED);
      });
    // [23]
    fsm_.Add(PMC_READY,                           // These events move...
      STATE::PREPARING,                           // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        if (!Control(NOMINAL, segment_))
          return Result(RESPONSE::CONTROL_FAILED);
        return STATE::CONTROLLING;
      });
    // [24]
    fsm_.Add(PMC_TIMEOUT, GOAL_CANCEL,            // These events move...
      STATE::PREPARING,                           // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::PMC_FAILED);
      });
    // [25]
    fsm_.Add(GOAL_PREP,                           // These events move...
      STATE::WAITING,                             // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        if (FlightMode())
          return STATE::PREPPING;
        return Result(RESPONSE::SUCCESS);
      });
    // [26]
    fsm_.Add(PMC_READY,                           // These events move...
      STATE::PREPPING,                            // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        return Result(RESPONSE::SUCCESS);         // The next state...
      });
    // [27]
    fsm_.Add(PMC_TIMEOUT, GOAL_CANCEL,            // These events move...
      STATE::PREPPING,                            // The current state to...
      [this](ChoreographerEvent const& event) -> int32_t {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED);
        return Result(RESPONSE::PMC_FAILED);
      });
  }

  // Destructor
  ~ChoreographerNodelet() {}

 protected:
  // Called on statup
  void Initialize(ros::NodeHandle *nh) {
    // Configuration parameters
    cfg_.Initialize(GetPrivateHandle(), "mobility/choreographer.config");
    cfg_.Listen(boost::bind(
      &ChoreographerNodelet::ReconfigureCallback, this, _1));

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
    pub_inertia_ = nh->advertise<geometry_msgs::Inertia>(
      TOPIC_MANAGEMENT_INERTIA, 1, true);

    // Subscribe to collisions from the sentinel node
    sub_collisions_ = nh->subscribe(TOPIC_MOBILITY_COLLISIONS, 5,
      &ChoreographerNodelet::CollisionCallback, this);

    // Subscribe to the latched PMC state to be notified of propulsion events
    sub_pmc_state_= nh->subscribe(TOPIC_HARDWARE_PMC_STATE, 5,
      &ChoreographerNodelet::PmcStateCallback, this);

    // Allow planners to register themselves
    server_register_ = nh->advertiseService(SERVICE_MOBILITY_PLANNER_REGISTER,
      &ChoreographerNodelet::PlannerRegisterCallback, this);

    // Allow the state to be manually set
    server_set_state_ = nh->advertiseService(SERVICE_MOBILITY_SET_STATE,
      &ChoreographerNodelet::SetStateCallback, this);

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

    // Setup validate client action
    client_v_.SetConnectedTimeout(
      cfg_.Get<double>("timeout_validate_connected"));
    client_v_.SetActiveTimeout(
      cfg_.Get<double>("timeout_validate_active"));
    client_v_.SetResponseTimeout(
      cfg_.Get<double>("timeout_validate_response"));
    client_v_.SetDeadlineTimeout(
      cfg_.Get<double>("timeout_validate_deadline"));
    client_v_.SetFeedbackCallback(std::bind(
      &ChoreographerNodelet::VFeedbackCallback, this,
        std::placeholders::_1));
    client_v_.SetResultCallback(std::bind(
      &ChoreographerNodelet::VResultCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    client_v_.SetConnectedCallback(std::bind(
      &ChoreographerNodelet::ConnectedCallback, this));
    client_v_.Create(nh, ACTION_MOBILITY_VALIDATE);

    // Setup the move action
    server_.SetGoalCallback(std::bind(
      &ChoreographerNodelet::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &ChoreographerNodelet::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &ChoreographerNodelet::CancelCallback, this));
    server_.Create(nh, ACTION_MOBILITY_MOTION);

    // Publish the default flight mode so the system boots predictably
    if (!ff_util::FlightUtil::GetInitialFlightMode(flight_mode_))
      AssertFault("INITIALIZATION_FAULT", "Problem with initial flight mode");
    pub_flight_mode_.publish(flight_mode_);

    // Publish the default inertia parameters -- this will eventually be
    // moved to executive.
    geometry_msgs::Inertia msg;
    if (!ff_util::FlightUtil::GetInertiaConfig(msg))
      AssertFault("INITIALIZATION_FAULT", "Problem with default inertia");
    pub_inertia_.publish(msg);
  }

  // Callback to handle reconfiguration requests
  bool ReconfigureCallback(dynamic_reconfigure::Config & config) {
    switch (fsm_.GetState()) {
    case STATE::WAITING:
      return cfg_.Reconfigure(config);
    default:
      break;
    }
    return false;
  }

  // Ensure all clients are connected
  void ConnectedCallback() {
    if (!client_v_.IsConnected()) return;       // Validate action
    if (!client_c_.IsConnected()) return;       // Control action
    fsm_.Update(READY);
  }

  // Called on registration of aplanner
  bool SetStateCallback(ff_msgs::SetState::Request& req,
                        ff_msgs::SetState::Response& res) {
    fsm_.SetState(req.state);
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

  // Complete the current dock or undock action
  int32_t Result(int32_t response) {
    // Based on the design of this class only one child action is running
    // at any point in time (planner, validation, control). If the action
    // gets cancelled (through preemption or cancellation) we might need
    // to clean up, based on the state of the system
    if (response == RESPONSE::CANCELLED) {
      switch (fsm_.GetState()) {
      case STATE::BOOTSTRAPPING:
      case STATE::PLANNING:
      case STATE::REPLANNING: {
        std::string planner = cfg_.Get<std::string>("planner");
        if (planners_.find(planner) != planners_.end())
          planners_[planner].CancelGoal();
        break;
      }
      case STATE::VALIDATING:
      case STATE::REVALIDATING:
        client_v_.CancelGoal();
        break;
      case STATE::IDLING:
      case STATE::STOPPING:
      case STATE::CONTROLLING:
        client_c_.CancelGoal();
        break;
      }
    }
    // Prevent a spurious result from blocking initializtion
    switch (fsm_.GetState()) {
    case STATE::WAITING:
    case STATE::INITIALIZING:
    case STATE::WAITING_FOR_STOP:
      return fsm_.GetState();
    default:
      break;
    }
    // If we get here then we are in a valid action state, so we will need
    // to produce a meanngful result for the callee.
    static ff_msgs::MotionResult result;
    result.response = response;
    result.segment = segment_;
    result.flight_mode = flight_mode_;
    if (response > 0)
      server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    else if (response < 0)
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
    else
      server_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
    // Return to the waiting state
    return STATE::WAITING;
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
  void UpdateCallback(int32_t state, ChoreographerEvent event) {
    // Debug events
    std::string str = "UNKNOWN";
    switch (event) {
    case READY:                   str = "READY";              break;
    case GOAL_EXEC:               str = "GOAL_EXEC";          break;
    case GOAL_MOVE:               str = "GOAL_MOVE";          break;
    case GOAL_STOP:               str = "GOAL_STOP";          break;
    case GOAL_IDLE:               str = "GOAL_IDLE";          break;
    case GOAL_PREP:               str = "GOAL_PREP";          break;
    case GOAL_CANCEL:             str = "GOAL_CANCEL";        break;
    case VALIDATE_SUCCESS:        str = "VALIDATE_SUCCESS";   break;
    case VALIDATE_FAILED:         str = "VALIDATE_FAILED";    break;
    case PMC_READY:               str = "PMC_READY";          break;
    case PMC_TIMEOUT:             str = "PMC_TIMEOUT";        break;
    case PLAN_SUCCESS:            str = "PLAN_SUCCESS";       break;
    case PLAN_FAILED:             str = "PLAN_FAILED";        break;
    case CONTROL_SUCCESS:         str = "CONTROL_SUCCESS";    break;
    case CONTROL_FAILED:          str = "CONTROL_FAILED";     break;
    case TOLERANCE_POS:           str = "TOLERANCE_POS";      break;
    case TOLERANCE_ATT:           str = "TOLERANCE_ATT";      break;
    case TOLERANCE_VEL:           str = "TOLERANCE_VEL";      break;
    case TOLERANCE_OMEGA:         str = "TOLERANCE_OMEGA";    break;
    case OBSTACLE_DETECTED:       str = "OBSTACLE_DETECTED";  break;
    }
    NODELET_DEBUG_STREAM("Received event " << str);
    // Debug state changes
    switch (state) {
    case STATE::INITIALIZING:     str = "INITIALIZING";       break;
    case STATE::WAITING_FOR_STOP: str = "WAITING_FOR_STOP";   break;
    case STATE::WAITING:          str = "WAITING";            break;
    case STATE::IDLING:           str = "IDLING";             break;
    case STATE::STOPPING:         str = "STOPPING";           break;
    case STATE::PREPPING:         str = "PREPPING";           break;
    case STATE::BOOTSTRAPPING:    str = "BOOTSTRAPPING";      break;
    case STATE::PLANNING:         str = "PLANNING";           break;
    case STATE::VALIDATING:       str = "VALIDATING";         break;
    case STATE::PREPARING:        str = "PREPARING";          break;
    case STATE::CONTROLLING:      str = "CONTROLLING";        break;
    case STATE::REPLANNING:       str = "REPLANNING";         break;
    case STATE::REVALIDATING:     str = "REVALIDATING";       break;
    }
    NODELET_DEBUG_STREAM("State changed to " << str);
    // Broadcast the state
    static ff_msgs::MotionState msg;
    msg.header.frame_id = GetPlatform();
    msg.header.stamp = ros::Time::now();
    msg.state = state;
    pub_state_.publish(msg);
    // Send the feedback if needed
    switch (state) {
    case STATE::IDLING:
    case STATE::STOPPING:
    case STATE::BOOTSTRAPPING:
    case STATE::PLANNING:
    case STATE::VALIDATING:
    case STATE::PREPARING:
    case STATE::CONTROLLING:
    case STATE::REPLANNING:
    case STATE::REVALIDATING: {
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
  void CollisionCallback(geometry_msgs::PointStamped::ConstPtr const& point) {
    obstacle_ = *point;
    return fsm_.Update(OBSTACLE_DETECTED);
  }

  // VALIDATE

  // Validate the current working segment and flight mode
  bool Validate(ff_util::Segment const& segment) {
    static ff_msgs::ValidateGoal goal;
    goal.flight_mode = flight_mode_;
    goal.faceforward = cfg_.Get<bool>("enable_faceforward");
    goal.segment = segment;
    goal.max_time = ros::Duration(
      cfg_.Get<double>("timeout_validate_deadline"));
    client_v_.SetDeadlineTimeout(
      cfg_.Get<double>("timeout_validate_deadline"));
    return client_v_.SendGoal(goal);
  }

  // Validate feedback -- forward to the motion state
  void VFeedbackCallback(ff_msgs::ValidateFeedbackConstPtr const& feedback) {
    switch (fsm_.GetState()) {
    case STATE::VALIDATING:
    case STATE::REVALIDATING:
      feedback_.perc_complete = feedback->perc_complete;
      feedback_.secs_remaining = feedback->secs_remaining;
      return server_.SendFeedback(feedback_);
    default:
      break;
    }
  }

  // Validate result  - trigger a FSM update
  void VResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::ValidateResultConstPtr const& result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      segment_ = result->segment;
      return fsm_.Update(VALIDATE_SUCCESS);
    default:
      return fsm_.Update(VALIDATE_FAILED);
    }
  }

  // PLAN

  // Initiate the planning of a segment
  bool Plan(std::vector<geometry_msgs::PoseStamped> const& states,
    ros::Duration duration = ros::Duration(0)) {
    // Divide by a constant in holonomic mode to avoid saturation
    double divider = 1;
    if (!cfg_.Get<bool>("enable_faceforward") && flight_mode_.hard_divider > 0)
      divider = flight_mode_.hard_divider;
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
    if (plan_goal.desired_vel < 0)
      plan_goal.desired_vel = flight_mode_.hard_limit_vel;
    else if (plan_goal.desired_vel > flight_mode_.hard_limit_vel)
      return false;
    // Check deesired acceleration
    if (plan_goal.desired_accel < 0)
      plan_goal.desired_accel = flight_mode_.hard_limit_accel / divider;
    else if (plan_goal.desired_accel > flight_mode_.hard_limit_accel / divider)
      return false;
    // Check desired omega
    if (plan_goal.desired_omega < 0)
      plan_goal.desired_omega = flight_mode_.hard_limit_omega;
    else if (plan_goal.desired_omega > flight_mode_.hard_limit_omega)
      return false;
    // Check  desired alpha
    if (plan_goal.desired_alpha < 0)
      plan_goal.desired_alpha = flight_mode_.hard_limit_alpha / divider;
    else if (plan_goal.desired_alpha > flight_mode_.hard_limit_alpha / divider)
      return false;
    // Check control frequency
    if (plan_goal.desired_rate < 0)
      plan_goal.desired_rate = ff_util::FlightUtil::MIN_CONTROL_RATE;
    else if (plan_goal.desired_rate < ff_util::FlightUtil::MIN_CONTROL_RATE)
      return false;
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
      return planners_[planner].SendGoal(plan_goal);
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
        if (result->response == ff_msgs::PlanResult::SUCCESS) {
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
    pub_flight_mode_.publish(flight_mode_);
    if (prep_required_) {
      timer_speed_.start();
      timer_feedback_.start();
      return true;
    }
    return false;
  }

  // Called when the sentinel forecasts an upcoming collision
  void PmcStateCallback(ff_hw_msgs::PmcState::ConstPtr const& msg) {
    bool ready = true;
    for (size_t i = 0; i < msg->states.size() && ready; i++)
      ready &= (msg->states[i] == ff_hw_msgs::PmcState::READY);
    if (!ready)
      return;
    timer_speed_.stop();
    timer_feedback_.stop();
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
      //  so that the first setpoint conincides with the current time.
      ros::Time reftime = goal.segment.front().when;
      if (cfg_.Get<bool>("enable_immediate"))
        reftime = ros::Time::now();
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
        NODELET_WARN_STREAM("Position tolerance violated");
        NODELET_WARN_STREAM("- Value: " << feedback->error_position
                                        << ", Thresh: "
                                        << flight_mode_.tolerance_pos);
        fsm_.Update(TOLERANCE_POS);
        return;
      }
      // Check attitude tolerance
      if (flight_mode_.tolerance_att > 0.0 &&
          feedback->error_attitude > flight_mode_.tolerance_att) {
        NODELET_WARN_STREAM("Attitude tolerance violated");
        NODELET_WARN_STREAM("- Value: " << feedback->error_attitude
                                        << ", Thresh: "
                                        << flight_mode_.tolerance_att);
        fsm_.Update(TOLERANCE_ATT);
        return;
      }
      // Check velocity tolerance
      if (flight_mode_.tolerance_vel > 0.0 &&
          feedback->error_velocity > flight_mode_.tolerance_vel) {
        NODELET_WARN_STREAM("Velocity tolerance violated");
        NODELET_WARN_STREAM("- Value: " << feedback->error_velocity
                                        << ", Thresh: "
                                        << flight_mode_.tolerance_vel);
        fsm_.Update(TOLERANCE_VEL);
        return;
      }
      // Check angular velocity tolerance
      if (flight_mode_.tolerance_omega > 0.0 &&
          feedback->error_omega > flight_mode_.tolerance_omega) {
        NODELET_WARN_STREAM("Angular velocity tolerance violated");
        NODELET_WARN_STREAM("- Value: " << feedback->error_omega
                                        << ", Thresh: "
                                        << flight_mode_.tolerance_omega);
        fsm_.Update(TOLERANCE_OMEGA);
        return;
      }
    // Send progress in stopping/idling
    case STATE::STOPPING:
    case STATE::IDLING:
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

  // A new arm action has been called
  void GoalCallback(ff_msgs::MotionGoalConstPtr const& goal) {
    ff_msgs::MotionResult result;
    // We can only accept new commands if we are currently in an idle state.
    // This should be the case if Preempt() was called beforehand -- as it
    // should be automatically -- in the case of preemption.
    if (fsm_.GetState() != STATE::WAITING) {
      result.response = RESPONSE::NOT_IN_WAITING_MODE;
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
      return;
    }
    // If the specified flight mode string is empty then we wont try and
    // change the flight mode from what it currently is.
    prep_required_ = false;
    if (!goal->flight_mode.empty()) {
      uint8_t old_gain = flight_mode_.speed;
      if (!ff_util::FlightUtil::GetFlightMode(
        flight_mode_, goal->flight_mode)) {
        result.response = RESPONSE::INVALID_FLIGHT_MODE;
        server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
        return;
      }
      prep_required_ = (old_gain != flight_mode_.speed);
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
  ff_util::FiniteStateMachine<int32_t, ChoreographerEvent> fsm_;
  ff_util::FreeFlyerActionClient<ff_msgs::ControlAction> client_c_;
  ff_util::FreeFlyerActionClient<ff_msgs::ValidateAction> client_v_;
  ff_util::FreeFlyerActionServer<ff_msgs::MotionAction> server_;
  ff_msgs::MotionFeedback feedback_;
  // Runtime configuration
  ff_util::ConfigServer cfg_;
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
  ros::Subscriber sub_collisions_, sub_pmc_state_;
  ros::ServiceServer server_register_, server_set_state_;
  // Cached between callbacks
  ff_msgs::FlightMode flight_mode_;                   // Desired flight mode
  std::vector<geometry_msgs::PoseStamped> states_;    // Plan request
  ff_util::Segment segment_;                          // Segment
  geometry_msgs::PointStamped obstacle_;              // Obstacle
  bool prep_required_;                                // Check if prep needed
};

// Declare the plugin
PLUGINLIB_DECLARE_CLASS(choreographer, ChoreographerNodelet,
                        choreographer::ChoreographerNodelet, nodelet::Nodelet);

}  // namespace choreographer
