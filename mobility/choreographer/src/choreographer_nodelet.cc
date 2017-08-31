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
#include <ff_util/config_server.h>
#include <ff_util/conversion.h>

// Generic messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>

// FreeFlyer messages
#include <ff_msgs/ControlCommand.h>
#include <ff_msgs/MobilityProgress.h>
#include <ff_msgs/MobilityResult.h>
#include <ff_msgs/Fault.h>
#include <ff_msgs/SetBool.h>
#include <ff_msgs/RegisterPlanner.h>

// Action servers
#include <ff_msgs/ExecuteAction.h>
#include <ff_msgs/StopAction.h>
#include <ff_msgs/MoveAction.h>
#include <ff_msgs/IdleAction.h>

// Action clients
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
using STATE = ff_msgs::MobilityProgress;
using RESPONSE = ff_msgs::MobilityResult;

// Data structure for managing planner registration
typedef std::map < std::string, std::string > PlannerInfo;
typedef std::map < std::string, ff_util::FreeFlyerActionClient < ff_msgs::PlanAction > > PlannerMap;

// The choreograph class manages core logic for mobility
class ChoreographerNodelet : public ff_util::FreeFlyerNodelet {
 public:
  // Constructor
  ChoreographerNodelet() :
    ff_util::FreeFlyerNodelet(NODE_CHOREOGRAPHER, true), state_(STATE::INITIALIZING) {}

  // Destructor
  ~ChoreographerNodelet() {}

 protected:
  // Called on statup
  void Initialize(ros::NodeHandle *nh) {
    // Configuration parameters
    cfg_.Initialize(GetPrivateHandle(), "mobility/choreographer.config");
    cfg_.Listen(boost::bind(&ChoreographerNodelet::ReconfigureCallback, this, _1));

    // Setup a timer to forward diagnostics
    timer_d_ = nh->createTimer(ros::Duration(ros::Rate(DEFAULT_DIAGNOSTICS_RATE)),
      &ChoreographerNodelet::DiagnosticsCallback, this, false, true);

    // Create a transform buffer ot listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));

    // Segment and trajectory publisher
    pub_s_ = nh->advertise < nav_msgs::Path > (TOPIC_MOBILITY_SEGMENT, 5, true);

    // Subscribe to collisions from the sentinel node
    sub_c_ = nh->subscribe(TOPIC_MOBILITY_COLLISIONS, 5, &ChoreographerNodelet::CollisionCallback, this);

    // Allow planners to register themselves
    server_p_ = nh->advertiseService(SERVICE_MOBILITY_PLANNER_REGISTER,
      &ChoreographerNodelet::PlannerRegisterCallback, this);

    // Contact GNC service for enabling control of the platform
    client_t_.SetConnectedTimeout(cfg_.Get<double>("timeout_toggle_connected"));
    client_t_.SetConnectedCallback(std::bind(&ChoreographerNodelet::ConnectedCallback, this));
    client_t_.SetTimeoutCallback(std::bind(&ChoreographerNodelet::ServiceTimeoutCallback, this));
    client_t_.Create(nh, SERVICE_GNC_CTL_ENABLE);

    // Setup control client action
    client_c_.SetConnectedTimeout(cfg_.Get<double>("timeout_control_connected"));
    client_c_.SetActiveTimeout(cfg_.Get<double>("timeout_control_active"));
    client_c_.SetResponseTimeout(cfg_.Get<double>("timeout_control_response"));
    client_c_.SetFeedbackCallback(
      std::bind(&ChoreographerNodelet::CFeedbackCallback, this, std::placeholders::_1));
    client_c_.SetResultCallback(
      std::bind(&ChoreographerNodelet::CResultCallback, this, std::placeholders::_1, std::placeholders::_2));
    client_c_.SetConnectedCallback(
      std::bind(&ChoreographerNodelet::ConnectedCallback, this));
    client_c_.Create(nh, ACTION_GNC_CTL_CONTROL);

    // Setup validate client action
    client_v_.SetConnectedTimeout(cfg_.Get<double>("timeout_validate_connected"));
    client_v_.SetActiveTimeout(cfg_.Get<double>("timeout_validate_active"));
    client_v_.SetResponseTimeout(cfg_.Get<double>("timeout_validate_response"));
    client_v_.SetDeadlineTimeout(cfg_.Get<double>("timeout_validate_deadline"));
    client_v_.SetFeedbackCallback(
      std::bind(&ChoreographerNodelet::VFeedbackCallback, this, std::placeholders::_1));
    client_v_.SetResultCallback(
      std::bind(&ChoreographerNodelet::VResultCallback, this, std::placeholders::_1, std::placeholders::_2));
    client_v_.SetConnectedCallback(
      std::bind(&ChoreographerNodelet::ConnectedCallback, this));
    client_v_.Create(nh, ACTION_MOBILITY_VALIDATE);

    // Setup the execute action
    server_e_.SetGoalCallback(std::bind(&ChoreographerNodelet::EGoalCallback, this, std::placeholders::_1));
    server_e_.SetPreemptCallback(std::bind(&ChoreographerNodelet::PreemptCallback, this));
    server_e_.SetCancelCallback(std::bind(&ChoreographerNodelet::CancelCallback, this));
    server_e_.Create(nh, ACTION_MOBILITY_EXECUTE);

    // Setup the move action
    server_m_.SetGoalCallback(std::bind(&ChoreographerNodelet::MGoalCallback, this, std::placeholders::_1));
    server_m_.SetPreemptCallback(std::bind(&ChoreographerNodelet::PreemptCallback, this));
    server_m_.SetCancelCallback(std::bind(&ChoreographerNodelet::CancelCallback, this));
    server_m_.Create(nh, ACTION_MOBILITY_MOVE);

    // Setup the stop action
    server_s_.SetGoalCallback(std::bind(&ChoreographerNodelet::SGoalCallback, this, std::placeholders::_1));
    server_s_.SetPreemptCallback(std::bind(&ChoreographerNodelet::PreemptCallback, this));
    server_s_.SetCancelCallback(std::bind(&ChoreographerNodelet::CancelCallback, this));
    server_s_.Create(nh, ACTION_MOBILITY_STOP);

    // Setup the idle action
    server_i_.SetGoalCallback(std::bind(&ChoreographerNodelet::IGoalCallback, this, std::placeholders::_1));
    server_i_.SetPreemptCallback(std::bind(&ChoreographerNodelet::PreemptCallback, this));
    server_i_.SetCancelCallback(std::bind(&ChoreographerNodelet::CancelCallback, this));
    server_i_.Create(nh, ACTION_MOBILITY_IDLE);

    // Notify initialization complete
    NODELET_DEBUG_STREAM("Initialization complete");
  }

  // Send diagnostics
  void DiagnosticsCallback(const ros::TimerEvent &event) {
    SendDiagnostics(cfg_.Dump());
  }

  // Callback to handle reconfiguration requests
  bool ReconfigureCallback(dynamic_reconfigure::Config & config) {
    NODELET_DEBUG_STREAM("ReconfigureCallback()");
    // Only allow reconfiguration within certain states
    switch (state_) {
    case STATE::INITIALIZING:
      NODELET_WARN_STREAM("Cannot reconfigure a node in INITIALIZING state");
      return false;
    default:
      NODELET_DEBUG_STREAM("Reconfiguring node");
      break;
    }
    // Grab the new config parameters
    if (!cfg_.Reconfigure(config))
      return false;
        // Send an empty reconfigure to enable control
    return ToggleControl(cfg_.Get<bool>("enable_control"));
  }

  // Enable control
  bool ToggleControl(bool enable) {
    ff_msgs::SetBool msg;
    msg.request.enable = enable;
    if (!client_t_.Call(msg)) {
      Complete(RESPONSE::COULD_NOT_CALL_CONTROL_ENABLE_SERVICE);
      return false;
    }
    // Success!
    return true;
  }

  // Called on registration of aplanner
  bool PlannerRegisterCallback(ff_msgs::RegisterPlanner::Request& req, ff_msgs::RegisterPlanner::Response& res) {
    bool found = planners_.find(req.name) != planners_.end();
    if (req.unregister) {
      NODELET_DEBUG_STREAM("A planner called '" << req.name << "' unregister itself");
      if (found) {
        planners_.erase(req.name);
        info_.erase(req.name);
        cfg_.Lim < std::string > ("planner", info_);
        return true;
      }
      return false;
    }
    // See if the planner already exists
    if (found) {
      NODELET_WARN_STREAM_NAMED("choreographer",
        "A planner called '" << req.name << "' tried to register itself multiple times.");
      return false;
    }
    // Register the planner
    NODELET_DEBUG_STREAM("A planner called '" << req.name << "' just registered itself.");
    info_[req.name] = req.description;
    planners_[req.name].SetConnectedTimeout(cfg_.Get<double>("timeout_plan_connected"));
    planners_[req.name].SetActiveTimeout(cfg_.Get<double>("timeout_plan_active"));
    planners_[req.name].SetResponseTimeout(cfg_.Get<double>("timeout_plan_response"));
    planners_[req.name].SetDeadlineTimeout(cfg_.Get<double>("timeout_plan_deadline"));
    planners_[req.name].SetFeedbackCallback(
      std::bind(&ChoreographerNodelet::PFeedbackCallback, this, std::placeholders::_1));
    planners_[req.name].SetResultCallback(
      std::bind(&ChoreographerNodelet::PResultCallback, this, std::placeholders::_1, std::placeholders::_2));
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

  // Start executing the segment, and return false if control is disabled
  bool SendControl(uint8_t mode, std::string const& flight_mode, ff_util::Segment const& segment = ff_util::Segment()) {
    NODELET_DEBUG_STREAM("Sending control");
    // Cache the segment and flight mode
    flight_mode_ = flight_mode;
    segment_ = segment;
    // Special case: modify the initial timestamp
    if (cfg_.Get<bool>("enable_immediate") && !segment_.empty()) {
      ros::Duration diff = ros::Time::now() - segment_.front().when;
      for (ff_util::Segment::iterator it = segment_.begin(); it != segment_.end(); it++)
        it->when += diff;
    }
    // Special case: if we don't want to control the platform, just return immediately
    if (!cfg_.Get<bool>("enable_control"))
      return false;
    // Send the control
    ff_msgs::ControlGoal control_goal;
    control_goal.mode = mode;
    control_goal.segment = segment_;
    control_goal.flight_mode = flight_mode_;
    client_c_.SendGoal(control_goal);
    // Publish the segment to RVIZ!
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    path.poses.reserve(segment_.size());
    geometry_msgs::PoseStamped ps;
    ps.header = path.header;
    for (ff_util::Segment::iterator it = segment_.begin(); it != segment_.end(); it++) {
      ps.header.stamp = it->when;
      ps.pose = it->pose;
      path.poses.push_back(ps);
    }
    pub_s_.publish(path);
    // Success!
    return true;
  }

  bool GetWorldPose(geometry_msgs::PoseStamped & pose) {
    try {
      geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
        std::string(FRAME_NAME_WORLD),
        (GetPlatform().empty() ? "body" : GetPlatform() + "/" + std::string(FRAME_NAME_BODY)),
        ros::Time(0));
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

  ///////////////////////// FINITE STATE MACHINE //////////////////////////////////////

  // Control state change with nice debug output
  void ChangeState(uint8_t state) {
    switch (state) {
    case STATE::INITIALIZING:
      NODELET_DEBUG_STREAM("Moving to STATE::INITIALIZING");              break;
    case STATE::WAITING:
      NODELET_DEBUG_STREAM("Moving to STATE::WAITING");                   break;
    case STATE::EXEC_VALIDATING:
      NODELET_DEBUG_STREAM("Moving to STATE::EXEC_VALIDATING");           break;
    case STATE::EXEC_BOOTSTRAPPING:
      NODELET_DEBUG_STREAM("Moving to STATE::EXEC_BOOTSTRAPPING");        break;
    case STATE::EXEC_CONTROLLING:
      NODELET_DEBUG_STREAM("Moving to STATE::EXEC_CONTROLLING");          break;
    case STATE::EXEC_REPLANNING:
      NODELET_DEBUG_STREAM("Moving to STATE::EXEC_REPLANNING");           break;
    case STATE::MOVE_PLANNING:
      NODELET_DEBUG_STREAM("Moving to STATE::MOVE_PLANNING");             break;
    case STATE::MOVE_CONTROLLING:
      NODELET_DEBUG_STREAM("Moving to STATE::MOVE_CONTROLLING");          break;
    case STATE::MOVE_REPLANNING:
      NODELET_DEBUG_STREAM("Moving to STATE::MOVE_REPLANNING");           break;
    case STATE::STOP_CONTROLLING:
      NODELET_DEBUG_STREAM("Moving to STATE::STOP_CONTROLLING");          break;
    case STATE::IDLE_CONTROLLING:
      NODELET_DEBUG_STREAM("Moving to STATE::IDLE_CONTROLLING");          break;
    }
    state_ = state;
  }

  // Assert a fault - katie's fault code handling will eventually go in here
  void Complete(int32_t response, ff_msgs::ControlProgress const& progress = ff_msgs::ControlProgress()) {
    // Print some debug info for error events
    switch (response) {
    case RESPONSE::ALREADY_THERE:
      NODELET_DEBUG_STREAM("Already in position");                        break;
    case RESPONSE::CALCULATED:
      NODELET_DEBUG_STREAM("Action completed without control");           break;
    case RESPONSE::SUCCESS:
      NODELET_DEBUG_STREAM("Action completed with control");              break;
    case RESPONSE::PREEMPTED:
      NODELET_DEBUG_STREAM("Preemption occurred");                        break;
    case RESPONSE::CANCELLED:
      NODELET_DEBUG_STREAM("Cancellation occurred");                        break;
    case RESPONSE::COULD_NOT_FIND_CONTROL_ENABLE_SERVICE:
      NODELET_ERROR_STREAM("Could not find  control enable service");     break;
    case RESPONSE::COULD_NOT_CALL_CONTROL_ENABLE_SERVICE:
      NODELET_ERROR_STREAM("Could not call  control enable service");     break;
    case RESPONSE::ACTION_TIMEOUT_WAITING_FOR_ACTIVE:
      NODELET_ERROR_STREAM("Timeout on action active");                   break;
    case RESPONSE::ACTION_TIMEOUT_WAITING_FOR_RESPONSE:
      NODELET_ERROR_STREAM("Timeout on action response");                 break;
    case RESPONSE::ACTION_TIMEOUT_WAITING_FOR_DEADLINE:
      NODELET_ERROR_STREAM("Timeout on action deadline");                 break;
    case RESPONSE::BAD_STATE_TRANSITION:
      NODELET_ERROR_STREAM("Bad state transition");                       break;
    case RESPONSE::OBSTACLE_AND_REPLANNING_DISABLED:
      NODELET_ERROR_STREAM("Obstacle detected but replanning disabled");  break;
    case RESPONSE::OBSTACLE_AND_INSUFFICIENT_REPLAN_TIME:
      NODELET_ERROR_STREAM("Obstacle detected but no time to replan");    break;
    case RESPONSE::UNAVOIDABLE_COLLISION_DETECTED:
      NODELET_ERROR_STREAM("Unavoidable obstacle detected");              break;
    case RESPONSE::NOT_YET_INITIALIZED:
      NODELET_ERROR_STREAM("Node not yet initialized");                   break;
    case RESPONSE::WATCH_ACTION_DIED_UNEXPECTEDLY:
      NODELET_ERROR_STREAM("Watch action died unexpectedly.");            break;
    case RESPONSE::STOP_CALLED_WITHOUT_CONTROL_ENABLED:
      NODELET_ERROR_STREAM("Stop called without control enabled");        break;
    case RESPONSE::IDLE_CALLED_WITHOUT_CONTROL_ENABLED:
      NODELET_ERROR_STREAM("Idle called without control enabled");        break;
    case RESPONSE::CONTROL_ENABLE_SERVICE_LOST:
      NODELET_ERROR_STREAM("The control enable service disappeared");     break;
    case RESPONSE::NO_FLIGHT_MODE_SPECIFIED:
      NODELET_ERROR_STREAM("Flight mode not specified");                  break;
    case RESPONSE::UNEXPECTED_EMPTY_SEGMENT:
      NODELET_ERROR_STREAM("Unexpected empty segment");                   break;
    case RESPONSE::CONTROL_FAILED:
      NODELET_ERROR_STREAM("Control failed");                             break;
    case RESPONSE::SELECTED_PLANNER_DOES_NOT_EXIST:
      NODELET_ERROR_STREAM("Planner does not exist");                     break;
    case RESPONSE::PLANNER_FAILED:
      NODELET_ERROR_STREAM("Planner failed");                             break;
    default:
      break;
    }
    // In the case where the current action was either preempted or cancelled, we need to
    // perform a controlled cleanup of resources before accepting a new goal!
    switch (response) {
    case RESPONSE::CANCELLED:
    case RESPONSE::PREEMPTED:
      switch (state_) {
      case STATE::INITIALIZING:
      case STATE::WAITING:
      default:
        break;
      case STATE::EXEC_VALIDATING:
        client_v_.CancelGoal();
        break;
      case STATE::MOVE_PLANNING:
      case STATE::EXEC_BOOTSTRAPPING: {
        std::string planner = cfg_.Get < std::string > ("planner");
        if (planners_.find(planner) != planners_.end())
          planners_.at(planner).CancelGoal();
        break;
      }
      case STATE::EXEC_REPLANNING:
      case STATE::MOVE_REPLANNING: {
        std::string planner = cfg_.Get < std::string > ("planner");
        if (planners_.find(planner) != planners_.end())
          planners_.at(planner).CancelGoal();
        break;
      }
      case STATE::MOVE_CONTROLLING:
      case STATE::EXEC_CONTROLLING:
      case STATE::STOP_CONTROLLING:
      case STATE::IDLE_CONTROLLING:
        client_c_.CancelGoal();
        break;
      }
    default:
      break;
    }
    // Send a final result to the client
    switch (state_) {
    case STATE::INITIALIZING:
    case STATE::WAITING:
      break;
    case STATE::EXEC_VALIDATING:
    case STATE::EXEC_CONTROLLING:
    case STATE::EXEC_BOOTSTRAPPING:
    case STATE::EXEC_REPLANNING: {
      ff_msgs::ExecuteResult exec_result;
      exec_result.result.response = response;
      exec_result.result.state = state_;
      exec_result.result.progress = progress;
      exec_result.result.segment = segment_;
      exec_result.result.flight_mode = flight_mode_;
      if (response > 0)
        server_e_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, exec_result);
      else if (response < 0)
        server_e_.SendResult(ff_util::FreeFlyerActionState::ABORTED, exec_result);
      else
        server_e_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, exec_result);
      break;
    }
    case STATE::MOVE_PLANNING:
    case STATE::MOVE_CONTROLLING:
    case STATE::MOVE_REPLANNING: {
      ff_msgs::MoveResult move_result;
      move_result.result.response = response;
      move_result.result.state = state_;
      move_result.result.progress = progress;
      move_result.result.segment = segment_;
      move_result.result.flight_mode = flight_mode_;
      if (response > 0)
        server_m_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, move_result);
      else if (response < 0)
        server_m_.SendResult(ff_util::FreeFlyerActionState::ABORTED, move_result);
      else
        server_m_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, move_result);
      break;
    }
    case STATE::STOP_CONTROLLING: {
      ff_msgs::StopResult stop_result;
      stop_result.result.response = response;
      stop_result.result.state = state_;
      stop_result.result.progress = progress;
      stop_result.result.flight_mode = flight_mode_;
      if (response > 0)
        server_s_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, stop_result);
      else if (response < 0)
        server_s_.SendResult(ff_util::FreeFlyerActionState::ABORTED, stop_result);
      else
        server_s_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, stop_result);
      break;
    }
    case STATE::IDLE_CONTROLLING: {
      ff_msgs::IdleResult idle_result;
      idle_result.result.response = response;
      idle_result.result.state = state_;
      idle_result.result.progress = progress;
      idle_result.result.flight_mode = flight_mode_;
      if (response > 0)
        server_i_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, idle_result);
      else if (response < 0)
        server_i_.SendResult(ff_util::FreeFlyerActionState::ABORTED, idle_result);
      else
        server_i_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, idle_result);
      break;
    }
    default:
      break;
    }
    // Safety mechanism: always send a stop event on completion to ensure the platform comes to a halt
    // SendControl(ff_msgs::ControlCommand::MODE_STOP, "nominal");
    // Move to a waiting state
    ChangeState(STATE::WAITING);
  }

  ///////////////////////// COLLISION CALLBACKS ///////////////////////////////////////

  // Called when the sentinel forecasts an upcoming collision
  void CollisionCallback(geometry_msgs::PointStamped::ConstPtr const& point) {
    NODELET_DEBUG_STREAM("CollisionCallback()");
    if (!cfg_.Get<double>("enable_collision_checking")) {
      NODELET_DEBUG_STREAM("Collision checking disabled, so ignoring");
      return;
    }
    switch (state_) {
    // In a MOVE or EXECUTE state, we want to start replanning
    case STATE::MOVE_CONTROLLING:
      ChangeState(STATE::MOVE_REPLANNING);
    case STATE::MOVE_REPLANNING:
      break;
    case STATE::EXEC_CONTROLLING:
      ChangeState(STATE::EXEC_REPLANNING);
    case STATE::EXEC_REPLANNING:
      break;
    // It's OK to receive watch feedback in wait, stopped and idle states. There's nothing we can do about it.
    case STATE::STOP_CONTROLLING:
    case STATE::IDLE_CONTROLLING:
    case STATE::WAITING:
      return;
    // All other states are not valid
    default:
      return Complete(RESPONSE::BAD_STATE_TRANSITION);
    }
    // Check that we are actually allowing replanning to take place
    if (!cfg_.Get<bool>("enable_replanning"))
      return Complete(RESPONSE::OBSTACLE_AND_REPLANNING_DISABLED);
    // If we get here here it's time to replan - to do so we'll need to pick an exchange point in the future. This
    // will be the point at which we switch from the old to the new segment. It needs to be far enough away from the
    // current time to allow planning to be carried out, but not so far that the exchange happens too close to the
    // point of collision. The 'replan_time_threshold' is a period of time before collision that we consider a safe
    // buffer for handover. The exchange_point is the index the closest setpoint BEFORE this buffer.
    ros::Time replan_time_threshold = segment_.back().when - ros::Duration(cfg_.Get<double>("replan_time_buffer"));
    ff_util::Segment::iterator exchange_point;  // This is the point we'll exchange segments
    for (exchange_point = segment_.begin(); exchange_point != segment_.end(); exchange_point++)
      if (exchange_point->when > replan_time_threshold) break;
    if (exchange_point == segment_.end())
      return Complete(RESPONSE::OBSTACLE_AND_INSUFFICIENT_REPLAN_TIME);
    // We might already be replanning, but this new collision detection event happens at an earlier time in the
    // segment. Perhaps this is because the hazcam picked a new obstacle up. Regardless, for safety we need to
    // replace our old plan request with a new one.
    if (exchange_point_ == segment_.end() || exchange_point->when < exchange_point_->when) {
      // Update the exchange point
      exchange_point_ = exchange_point;
      // Create a new planning goal, which will quietly preempt our last goal
      ff_msgs::PlanGoal plan_goal;
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = "world";
      ps.header.stamp = exchange_point_->when;
      ps.pose = exchange_point_->pose;
      plan_goal.states.push_back(ps);
      ps.header.frame_id = "world";
      ps.header.stamp = segment_.back().when;
      ps.pose = segment_.back().pose;
      plan_goal.states.push_back(ps);
      plan_goal.initial_twist = exchange_point->twist;
      plan_goal.initial_accel = exchange_point->accel;
      plan_goal.flight_mode = flight_mode_;
      plan_goal.faceforward = cfg_.Get<bool>("enable_faceforward");
      plan_goal.desired_vel = cfg_.Get<double>("desired_vel");
      plan_goal.desired_accel = cfg_.Get<double>("desired_accel");
      plan_goal.desired_omega = cfg_.Get<double>("desired_omega");
      plan_goal.desired_alpha = cfg_.Get<double>("desired_alpha");
      plan_goal.desired_rate = cfg_.Get<double>("desired_rate");
      plan_goal.max_time = exchange_point->when - ros::Time::now();
      if (ros::Duration(cfg_.Get<double>("min_plan_time")) > plan_goal.max_time) {
        Complete(RESPONSE::OBSTACLE_AND_INSUFFICIENT_REPLAN_TIME);
      }
      // Send off the new replan goal
      std::string planner = cfg_.Get < std::string > ("planner");
      if (planners_.find(planner) != planners_.end()) {
        planners_[planner].SetDeadlineTimeout(plan_goal.max_time.toSec());
        planners_[planner].SendGoal(plan_goal);
      } else {
        Complete(RESPONSE::SELECTED_PLANNER_DOES_NOT_EXIST);
      }
    }
  }

  /////////////////////// ACTION CLIENT CALLBACKS /////////////////////////////////////

  // Ensure all clients are connected
  void ConnectedCallback() {
    NODELET_DEBUG_STREAM("ConnectedCallback()");
    if (!client_c_.IsConnected()) return;       // Control action
    if (!client_v_.IsConnected()) return;       // Validate action
    if (!client_t_.IsConnected()) return;       // Toggle service
    if (state_ != STATE::INITIALIZING) return;  // Prevent multiple calls
    // Only get here if all four clients are connected
    ChangeState(STATE::WAITING);
    // Send an empty reconfigure to enable control
    ToggleControl(cfg_.Get<bool>("enable_control"));
  }

  // Timeout on a trajectory generation request
  void ServiceTimeoutCallback(void) {
    NODELET_DEBUG_STREAM("ServiceTimeoutCallback()");
    // Complete any action underway and transition to the initializing state
    Complete(RESPONSE::CONTROL_ENABLE_SERVICE_LOST);
    // Only get here if all four clients are connected
    ChangeState(STATE::INITIALIZING);
  }

  // PLAN

  void PFeedbackCallback(ff_msgs::PlanFeedbackConstPtr const& feedback) {
    NODELET_DEBUG_STREAM("PFeedbackCallback()");
    switch (state_) {
    case STATE::MOVE_REPLANNING:
    case STATE::MOVE_PLANNING: {
      ff_msgs::MoveFeedback move_feedback;
      move_feedback.progress.state = state_;
      move_feedback.progress.perc_complete = feedback->perc_complete;
      move_feedback.progress.secs_remaining = feedback->secs_remaining;
      server_m_.SendFeedback(move_feedback);
      }
      break;
    case STATE::EXEC_BOOTSTRAPPING:
    case STATE::EXEC_REPLANNING: {
      ff_msgs::ExecuteFeedback exec_feedback;
      exec_feedback.progress.state = state_;
      exec_feedback.progress.perc_complete = feedback->perc_complete;
      exec_feedback.progress.secs_remaining = feedback->secs_remaining;
      server_e_.SendFeedback(exec_feedback);
      }
      break;
    default:
      return Complete(RESPONSE::BAD_STATE_TRANSITION);
    }
  }

  void PResultCallback(ff_util::FreeFlyerActionState::Enum result_code, ff_msgs::PlanResultConstPtr const& result) {
    NODELET_DEBUG_STREAM("PResultCallback(" << result_code << ")");
    switch (result_code) {
    default:
    case ff_util::FreeFlyerActionState::ABORTED: {
      std::string err;
      switch (result->response) {
      case ff_msgs::PlanResult::CANCELLED:                      err  = "CANCELLED";                       break;
      case ff_msgs::PlanResult::SUCCESS:                        err  = "SUCCESS";                         break;
      case ff_msgs::PlanResult::PREEMPTED:                      err  = "PREEMPTED";                       break;
      case ff_msgs::PlanResult::NOT_ENOUGH_STATES:              err  = "NOT_ENOUGH_STATES";               break;
      case ff_msgs::PlanResult::OBSTACLES_NOT_SUPPORTED:        err  = "OBSTACLES_NOT_SUPPORTED";         break;
      case ff_msgs::PlanResult::BAD_STATE_TRANSITION:           err  = "BAD_STATE_TRANSITION";            break;
      case ff_msgs::PlanResult::BAD_DESIRED_VELOCITY:           err  = "BAD_DESIRED_VELOCITY";            break;
      case ff_msgs::PlanResult::BAD_DESIRED_ACCELERATION:       err  = "BAD_DESIRED_ACCELERATION";        break;
      case ff_msgs::PlanResult::BAD_DESIRED_OMEGA:              err  = "BAD_DESIRED_OMEGA";               break;
      case ff_msgs::PlanResult::BAD_DESIRED_ALPHA:              err  = "BAD_DESIRED_ALPHA";               break;
      case ff_msgs::PlanResult::BAD_DESIRED_RATE:               err  = "BAD_DESIRED_RATE";                break;
      case ff_msgs::PlanResult::CANNOT_LOAD_FLIGHT_DATA:        err  = "CANNOT_LOAD_FLIGHT_DATA";         break;
      case ff_msgs::PlanResult::CANNOT_LOAD_GENERAL_CONFIG:     err  = "CANNOT_LOAD_GENERAL_CONFIG";      break;
      case ff_msgs::PlanResult::NO_PATH_EXISTS:                 err  = "NO_PATH_EXISTS";                  break;
      case ff_msgs::PlanResult::PROBLEM_CONNECTING_TO_SERVICES: err  = "PROBLEM_CONNECTING_TO_SERVICES";  break;
      default: break;
      }
      NODELET_WARN_STREAM("Plan error: " << err);
      return Complete(RESPONSE::PLANNER_FAILED);
    }
    case ff_util::FreeFlyerActionState::PREEMPTED:
      return Complete(RESPONSE::PREEMPTED);
    case ff_util::FreeFlyerActionState::SUCCESS:
      break;
    }
    // We only get here if we successfully generated a segment. What we do with this segment is now up to the state
    // we are currently in. For example, if we are replanning, we need to capture and merge segments.
    switch (state_) {
    case STATE::EXEC_PLANNING:
    case STATE::EXEC_REPLANNING:
    case STATE::MOVE_REPLANNING: {
      ros::Duration shift = result->segment.back().when - segment_.front().when;
      for (ff_util::Segment::iterator it = segment_.begin(); it != segment_.end(); it++)
        it->when += shift;
      segment_.insert(segment_.begin(), result->segment.begin(), result->segment.end());
      switch (state_) {
      case STATE::EXEC_REPLANNING: ChangeState(STATE::EXEC_CONTROLLING); break;
      case STATE::MOVE_REPLANNING: ChangeState(STATE::MOVE_CONTROLLING); break;
      }
      if (!SendControl(ff_msgs::ControlCommand::MODE_NOMINAL, result->flight_mode, result->segment))
        return Complete(RESPONSE::CALCULATED);
      return;
    }
    case STATE::EXEC_BOOTSTRAPPING:
    case STATE::MOVE_PLANNING:
      ChangeState(STATE::MOVE_CONTROLLING);
      if (!SendControl(ff_msgs::ControlCommand::MODE_NOMINAL, result->flight_mode, result->segment))
        return Complete(RESPONSE::CALCULATED);
      break;
    default:
      return Complete(RESPONSE::BAD_STATE_TRANSITION);
    }
  }

  // VALIDATE

  void VFeedbackCallback(ff_msgs::ValidateFeedbackConstPtr const& feedback) {
    NODELET_DEBUG_STREAM_THROTTLE(1.0, "VFeedbackCallback()");
    switch (state_) {
    case STATE::EXEC_VALIDATING: {
      ff_msgs::MoveFeedback move_feedback;
      move_feedback.progress.state = state_;
      move_feedback.progress.perc_complete = feedback->perc_complete;
      move_feedback.progress.secs_remaining = feedback->secs_remaining;
      server_m_.SendFeedback(move_feedback);
      }
      break;
    default:
      return Complete(RESPONSE::BAD_STATE_TRANSITION);
    }
  }

  void VResultCallback(ff_util::FreeFlyerActionState::Enum result_code, ff_msgs::ValidateResultConstPtr const& result) {
    NODELET_DEBUG_STREAM("VResultCallback(" << result_code << ")");
    switch (state_) {
    case STATE::EXEC_VALIDATING:
      break;
    default:
      return Complete(RESPONSE::BAD_STATE_TRANSITION);
    }
    // We only get here if we are in a possition to accept this response
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      ChangeState(STATE::EXEC_CONTROLLING);
      if (!SendControl(ff_msgs::ControlCommand::MODE_NOMINAL, result->flight_mode, result->segment))
        return Complete(RESPONSE::CALCULATED);
      break;
    default:
      return Complete(RESPONSE::VALIDATE_FAILED);
    }
  }

  // CONTROL

  void CFeedbackCallback(ff_msgs::ControlFeedbackConstPtr const& feedback) {
    NODELET_DEBUG_STREAM_THROTTLE(1.0, "CFeedbackCallback()");
    switch (state_) {
    case STATE::STOP_CONTROLLING: {
      ff_msgs::StopFeedback stop_feedback;
      stop_feedback.progress.state = state_;
      stop_feedback.progress.control = feedback->progress;
      server_s_.SendFeedback(stop_feedback);
      break;
    }
    case STATE::IDLE_CONTROLLING: {
      ff_msgs::IdleFeedback idle_feedback;
      idle_feedback.progress.state = state_;
      idle_feedback.progress.control = feedback->progress;
      server_i_.SendFeedback(idle_feedback);
      break;
    }
    case STATE::MOVE_CONTROLLING: {
      ff_msgs::MoveFeedback move_feedback;
      move_feedback.progress.state = state_;
      move_feedback.progress.control = feedback->progress;
      server_m_.SendFeedback(move_feedback);
      break;
    }
    case STATE::EXEC_BOOTSTRAPPING:
      ChangeState(STATE::EXEC_CONTROLLING);
    case STATE::EXEC_CONTROLLING: {
      ff_msgs::ExecuteFeedback exec_feedback;
      exec_feedback.progress.state = state_;
      exec_feedback.progress.control = feedback->progress;
      server_e_.SendFeedback(exec_feedback);
      break;
    }
    // Ignore the control feedback when replanning is underway
    case STATE::MOVE_REPLANNING:
    case STATE::EXEC_REPLANNING:
      break;
    // All other states are not valid
    default:
      return Complete(RESPONSE::BAD_STATE_TRANSITION);
    }
  }

  void CResultCallback(ff_util::FreeFlyerActionState::Enum result_code, ff_msgs::ControlResultConstPtr const& result) {
    NODELET_DEBUG_STREAM("CResultCallback(" << result_code << ")");
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return Complete(RESPONSE::SUCCESS, result->progress);
    case ff_util::FreeFlyerActionState::ABORTED:
      switch (result->response) {
      case ff_msgs::ControlResult::TOLERANCE_VIOLATION_POSITION:
      case ff_msgs::ControlResult::TOLERANCE_VIOLATION_VELOCITY:
      case ff_msgs::ControlResult::TOLERANCE_VIOLATION_ATTITUDE:
      case ff_msgs::ControlResult::TOLERANCE_VIOLATION_OMEGA:
        if (state_ == STATE::EXEC_BOOTSTRAPPING) {
          if (!cfg_.Get<bool>("enable_bootstrapping"))
            return Complete(RESPONSE::NOT_ON_FIRST_POSE);
          // Check that we can query the current robot pose
          geometry_msgs::PoseStamped current_pose;
          if (!GetWorldPose(current_pose))
            return Complete(RESPONSE::CANNOT_QUERY_ROBOT_POSE);
          // Check that we can query the current robot pose
          geometry_msgs::PoseStamped first_pose;
          first_pose.header.stamp = segment_.front().when;
          first_pose.pose = segment_.front().pose;
          // Update the system state
          ChangeState(STATE::EXEC_PLANNING);
              // Send off a planning request
          ff_msgs::PlanGoal plan_goal;
          plan_goal.states.push_back(current_pose);
          plan_goal.states.push_back(first_pose);
          plan_goal.flight_mode = flight_mode_;
          plan_goal.faceforward = cfg_.Get<bool>("enable_faceforward");
          plan_goal.desired_vel = cfg_.Get<double>("desired_vel");
          plan_goal.desired_accel = cfg_.Get<double>("desired_accel");
          plan_goal.desired_omega = cfg_.Get<double>("desired_omega");
          plan_goal.desired_alpha = cfg_.Get<double>("desired_alpha");
          plan_goal.desired_rate = cfg_.Get<double>("desired_rate");
          plan_goal.max_time = ros::Duration(cfg_.Get<double>("timeout_plan_deadline"));
          // Pick a planner and go for it!
          std::string planner = cfg_.Get < std::string > ("planner");
          if (planners_.find(planner) != planners_.end()) {
            planners_[planner].SetDeadlineTimeout(cfg_.Get<double>("timeout_plan_deadline"));
            planners_[planner].SendGoal(plan_goal);
          } else {
            Complete(RESPONSE::SELECTED_PLANNER_DOES_NOT_EXIST);
          }
          return;
        }
      default:
        break;
      }
    default:
      return Complete(RESPONSE::CONTROL_FAILED);
    }
  }

  /////////////////////// ACTION SERVICE CALLBACKS ////////////////////////////////////

  // EXEC

  // A new execute goal arrives asynchronously
  void EGoalCallback(ff_msgs::ExecuteGoalConstPtr const& execute_goal) {
    NODELET_DEBUG_STREAM("EGoalCallback()");
    switch (state_) {
    default:
      Complete(RESPONSE::PREEMPTED);
    case STATE::WAITING: {
      // Cover special cases where not enough info has been provided by the user
      if (execute_goal->flight_mode.empty())
        return Complete(RESPONSE::NO_FLIGHT_MODE_SPECIFIED);
      if (execute_goal->segment.empty())
        return Complete(RESPONSE::UNEXPECTED_EMPTY_SEGMENT);
      // Zach's planner in GDS (as well as other planners) do not respect the minimum
      // control period required by GNC. This segment of code numerically resamples
      // the input segment to ensure that it passes validation!
      ff_util::Segment resampled;
      ff_util::SegmentResult ret = ff_util::SegmentUtil::Resample(execute_goal->segment, resampled);
      if (ret != ff_util::SUCCESS)
        return Complete(RESPONSE::COULD_NOT_RESAMPLE);
      // Switch to validation state
      ChangeState(STATE::EXEC_VALIDATING);
      // Now, send off a validation query to the mapper
      ff_msgs::ValidateGoal validate_goal;
      validate_goal.segment = resampled;
      validate_goal.flight_mode = execute_goal->flight_mode;
      validate_goal.max_time = ros::Duration(cfg_.Get<double>("timeout_validate_deadline"));
      client_v_.SetDeadlineTimeout(cfg_.Get<double>("timeout_validate_deadline"));
      client_v_.SendGoal(validate_goal);
      return;
    }
    case STATE::INITIALIZING:
      return Complete(RESPONSE::NOT_YET_INITIALIZED);
    }
  }

  // MOVE

  // A new move  goal arrives asynchronously
  void MGoalCallback(ff_msgs::MoveGoalConstPtr const& move_goal) {
    NODELET_DEBUG_STREAM("MGoalCallback()");
    switch (state_) {
    default:
      Complete(RESPONSE::PREEMPTED);
    case STATE::WAITING: {
      // Cover a special case where not enough info
      if (move_goal->flight_mode.empty())
        return Complete(RESPONSE::NO_FLIGHT_MODE_SPECIFIED);
      if (move_goal->states.empty())
        return Complete(RESPONSE::UNEXPECTED_EMPTY_SEGMENT);
      // Check that we can query the current robot pose
      geometry_msgs::PoseStamped current_pose;
      if (!GetWorldPose(current_pose))
        return Complete(RESPONSE::CANNOT_QUERY_ROBOT_POSE);
      // Update the system state
      ChangeState(STATE::MOVE_PLANNING);
      // Send off a planning request
      ff_msgs::PlanGoal plan_goal;
      plan_goal.states = move_goal->states;
      plan_goal.states.insert(plan_goal.states.begin(), current_pose);
      plan_goal.flight_mode = move_goal->flight_mode;
      plan_goal.faceforward = cfg_.Get<bool>("enable_faceforward");
      plan_goal.desired_vel = cfg_.Get<double>("desired_vel");
      plan_goal.desired_accel = cfg_.Get<double>("desired_accel");
      plan_goal.desired_omega = cfg_.Get<double>("desired_omega");
      plan_goal.desired_alpha = cfg_.Get<double>("desired_alpha");
      plan_goal.desired_rate = cfg_.Get<double>("desired_rate");
      plan_goal.max_time = ros::Duration(cfg_.Get<double>("timeout_plan_deadline"));
      // Pick a planner and go for it!
      std::string planner = cfg_.Get < std::string > ("planner");
      if (planners_.find(planner) != planners_.end()) {
        planners_[planner].SetDeadlineTimeout(cfg_.Get<double>("timeout_plan_deadline"));
        planners_[planner].SendGoal(plan_goal);
      } else {
        Complete(RESPONSE::SELECTED_PLANNER_DOES_NOT_EXIST);
      }
      return;
    }
    case STATE::INITIALIZING:
      return Complete(RESPONSE::NOT_YET_INITIALIZED);
    }
  }

  // STOP

  // A new move  goal arrives asynchronously
  void SGoalCallback(ff_msgs::StopGoalConstPtr const& stop_goal) {
    NODELET_DEBUG_STREAM("SGoalCallback()");
    switch (state_) {
    default:
      Complete(RESPONSE::PREEMPTED);
    case STATE::WAITING: {
      if (!SendControl(ff_msgs::ControlCommand::MODE_STOP, "nominal"))
        return Complete(RESPONSE::STOP_CALLED_WITHOUT_CONTROL_ENABLED);
      return ChangeState(STATE::STOP_CONTROLLING);
    }
    case STATE::INITIALIZING:
      return Complete(RESPONSE::NOT_YET_INITIALIZED);
    }
  }

  // IDLE

  // A new move  goal arrives asynchronously
  void IGoalCallback(ff_msgs::IdleGoalConstPtr const& idle_goal) {
    NODELET_DEBUG_STREAM("IGoalCallback()");
    switch (state_) {
    default:
      Complete(RESPONSE::PREEMPTED);
    case STATE::WAITING: {
      if (!SendControl(ff_msgs::ControlCommand::MODE_IDLE, "nominal"))
        return Complete(RESPONSE::IDLE_CALLED_WITHOUT_CONTROL_ENABLED);
      return ChangeState(STATE::IDLE_CONTROLLING);
    }
    case STATE::INITIALIZING:
      return Complete(RESPONSE::NOT_YET_INITIALIZED);
    }
  }

  void PreemptCallback(void) {
    NODELET_DEBUG_STREAM("PreemptCallback()");
    return Complete(RESPONSE::PREEMPTED);
  }

  void CancelCallback(void) {
    NODELET_DEBUG_STREAM("CancelCallback()");
    return Complete(RESPONSE::CANCELLED);
  }

 private:
  int8_t state_;                                                          // State
  tf2_ros::Buffer tf_buffer_;                                             // TF2 buffer
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;               // TF2 listener
  ff_util::FreeFlyerActionClient < ff_msgs::ControlAction > client_c_;    // gnc::control
  ff_util::FreeFlyerActionClient < ff_msgs::ValidateAction > client_v_;   // mapper::validate
  ff_util::FreeFlyerActionServer < ff_msgs::ExecuteAction > server_e_;    // mobility::execute
  ff_util::FreeFlyerActionServer < ff_msgs::MoveAction > server_m_;       // mobility::move
  ff_util::FreeFlyerActionServer < ff_msgs::StopAction > server_s_;       // mobility::stop
  ff_util::FreeFlyerActionServer < ff_msgs::IdleAction > server_i_;       // mobility::idle
  ff_util::FreeFlyerServiceClient < ff_msgs::SetBool > client_t_;         // Control enable service client
  ff_util::ConfigServer cfg_;                                             // Configuration
  std::string flight_mode_;                                               // Current flight mode
  ff_util::Segment segment_;                                              // Current segment
  ff_util::Segment::iterator exchange_point_;                             // Segment handover for replanning
  ros::Publisher pub_s_;                                                  // Visualization publisher
  ros::Subscriber sub_c_;                                                 // Collision detection
  ros::ServiceServer server_p_;                                           // Planner registration server
  ros::Timer timer_d_;                                                    // Diagnostics
  PlannerMap planners_;                                                   // Available planners
  PlannerInfo info_;                                                      // Available planner info
};

// Declare the plugin
PLUGINLIB_DECLARE_CLASS(choreographer, ChoreographerNodelet,
                        choreographer::ChoreographerNodelet, nodelet::Nodelet);

}  // namespace choreographer

