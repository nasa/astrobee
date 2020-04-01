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
#include <tf2_ros/transform_listener.h>

// Shared project includes
#include <ff_util/ff_fsm.h>
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/config_client.h>
#include <ff_util/config_server.h>

// Actions
#include <ff_msgs/ReturnToDockAction.h>
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/DockAction.h>
#include <ff_msgs/ReturnToDockState.h>

/**
 * \ingroup beh
 */
namespace return_to_dock {

using FSM = ff_util::FSM;
using STATE = ff_msgs::ReturnToDockState;
using RESPONSE = ff_msgs::ReturnToDockResult;

class ReturnToDockNodelet : public ff_util::FreeFlyerNodelet {
 public:
  // Bitmask of all events
  enum : FSM::Event {
    READY           = (1<<0),
    GOAL_MOVE       = (1<<1),
    GOAL_PREEMPT    = (1<<2),
    GOAL_CANCEL     = (1<<3),
    MOVE_SUCCESS    = (1<<4),
    MOVE_FAILED     = (1<<5),
    DOCK_SUCCESS    = (1<<6),
    DOCK_FAILED     = (1<<7),
  };

  ReturnToDockNodelet() : ff_util::FreeFlyerNodelet(NODE_RETURN_TO_DOCK, true),
    fsm_(STATE::INITIALIZING, std::bind(&ReturnToDockNodelet::UpdateCallback,
      this, std::placeholders::_1, std::placeholders::_2)) {
    // [0]
    fsm_.Add(STATE::INITIALIZING,
      READY, [this](FSM::Event const& event) -> FSM::State {
        return STATE::IDLE;
      });
    // [1]
    fsm_.Add(STATE::IDLE,
      GOAL_MOVE, [this](FSM::Event const& event) -> FSM::State {
        // Initiate the move action
        if (!Move()) {
          Result(RESPONSE::FAILED);
          return STATE::IDLE;
        }
        return STATE::MOVING;
      });
    // [2]
    fsm_.Add(STATE::MOVING,
      MOVE_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        // Initiate the dock action
        if (!Dock()) {
          Result(RESPONSE::FAILED);
          return STATE::IDLE;
        }
        return STATE::DOCKING;
      });
    // [3] Fail fast
    fsm_.Add(STATE::MOVING,
      MOVE_FAILED | GOAL_CANCEL | GOAL_PREEMPT, [this](FSM::Event const& event) -> FSM::State {
        if (event == GOAL_PREEMPT)
          Result(RESPONSE::PREEMPTED);
        else if (event == GOAL_CANCEL)
          Result(RESPONSE::CANCELLED);
        else
          Result(RESPONSE::FAILED);
        return STATE::IDLE;
      });
    // [4]
    fsm_.Add(STATE::DOCKING,
      DOCK_SUCCESS, [this](FSM::Event const& event) -> FSM::State {
        Result(RESPONSE::SUCCESS);
        return STATE::IDLE;
      });
    // [5] Fail fast
    fsm_.Add(STATE::DOCKING,
      DOCK_FAILED | GOAL_CANCEL | GOAL_PREEMPT, [this](FSM::Event const& event) -> FSM::State {
        if (event == GOAL_PREEMPT)
          Result(RESPONSE::PREEMPTED);
        else if (event == GOAL_CANCEL)
          Result(RESPONSE::CANCELLED);
        else
          Result(RESPONSE::FAILED);
        return STATE::IDLE;
      });
    }

  ~ReturnToDockNodelet() {}

 protected:
  void Initialize(ros::NodeHandle *nh) {
    // Load config file for this nodelet
    cfg_.Initialize(GetPrivateHandle(), "behaviors/return_to_dock.config");
    if (!cfg_.Listen(boost::bind(
      &ReturnToDockNodelet::ReconfigureCallback, this, _1)))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                         "Could not load config");
    pub_state_ = nh->advertise<ff_msgs::ReturnToDockState>(
      TOPIC_BEHAVIORS_RETURN_TO_DOCK_STATE, 1, true);
    // Create a transform buffer to listen for transforms
    tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(tf_buffer_));
    // Setup move client
    client_m_.SetConnectedTimeout(cfg_.Get<double>("timeout_motion_connected"));
    client_m_.SetActiveTimeout(cfg_.Get<double>("timeout_motion_active"));
    client_m_.SetResponseTimeout(cfg_.Get<double>("timeout_motion_response"));
    client_m_.SetFeedbackCallback(std::bind(&ReturnToDockNodelet::MFeedbackCallback,
      this, std::placeholders::_1));
    client_m_.SetResultCallback(std::bind(&ReturnToDockNodelet::MResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_m_.SetConnectedCallback(std::bind(
      &ReturnToDockNodelet::ConnectedCallback, this));
    client_m_.Create(nh, ACTION_MOBILITY_MOTION);
    // Setup dock client
    client_d_.SetConnectedTimeout(cfg_.Get<double>("timeout_docker_connected"));
    client_d_.SetActiveTimeout(cfg_.Get<double>("timeout_docker_active"));
    client_d_.SetResponseTimeout(cfg_.Get<double>("timeout_docker_response"));
    client_d_.SetFeedbackCallback(std::bind(&ReturnToDockNodelet::DFeedbackCallback,
      this, std::placeholders::_1));
    client_d_.SetResultCallback(std::bind(&ReturnToDockNodelet::DResultCallback,
      this, std::placeholders::_1, std::placeholders::_2));
    client_d_.SetConnectedCallback(std::bind(
      &ReturnToDockNodelet::ConnectedCallback, this));
    client_d_.Create(nh, ACTION_BEHAVIORS_DOCK);
    // Setup the return to dock server
    server_.SetGoalCallback(std::bind(
      &ReturnToDockNodelet::GoalCallback, this, std::placeholders::_1));
    server_.SetPreemptCallback(std::bind(
      &ReturnToDockNodelet::PreemptCallback, this));
    server_.SetCancelCallback(std::bind(
      &ReturnToDockNodelet::CancelCallback, this));
    server_.Create(nh, ACTION_BEHAVIORS_RETURN_TO_DOCK);
  }

  // Ensure all clients are connected
  void ConnectedCallback() {
    if (!client_m_.IsConnected()) return;       // Move action
    if (!client_d_.IsConnected()) return;       // Docking action
    fsm_.Update(READY);
  }

  // Complete the current return to dock action
  void Result(int32_t response, std::string const& msg = "") {
    // Send the feedback if needed
    switch (fsm_.GetState()) {
    case STATE::INITIALIZING:
    case STATE::IDLE:
      return;
    default:
      break;
    }
    // Package up the feedback
    RESPONSE result;
    result.fsm_result = msg;
    result.response = response;
    if (response == RESPONSE::SUCCESS)
      server_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    else if (response == RESPONSE::PREEMPTED)
      server_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
    else
      server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
  }

  // When the FSM state changes we get a callback here, so that we can send
  // feedback to any active client, print debug info and publish our state
  void UpdateCallback(FSM::State const& state, FSM::Event const& event) {
    // Debug events
    ff_msgs::ReturnToDockState msg;
    msg.header.frame_id = GetPlatform();
    msg.header.stamp = ros::Time::now();
    msg.state = state;
    // Debug event changes
    switch (event) {
    case READY:                   msg.fsm_event = "READY";              break;
    case GOAL_MOVE:               msg.fsm_event = "GOAL_MOVE";          break;
    case GOAL_PREEMPT:            msg.fsm_event = "GOAL_PREEMPT";       break;
    case GOAL_CANCEL:             msg.fsm_event = "GOAL_CANCEL";        break;
    case MOVE_SUCCESS:            msg.fsm_event = "MOVE_SUCCESS";       break;
    case MOVE_FAILED:             msg.fsm_event = "MOVE_FAILED";        break;
    case DOCK_SUCCESS:            msg.fsm_event = "DOCK_SUCCESS";       break;
    case DOCK_FAILED:             msg.fsm_event = "DOCK_FAILED";        break;
    }
    // Debug state changes
    switch (state) {
    case STATE::INITIALIZING:     msg.fsm_state = "INITIALIZING";       break;
    case STATE::IDLE:             msg.fsm_state = "IDLE";               break;
    case STATE::MOVING:           msg.fsm_state = "MOVING";             break;
    case STATE::DOCKING:          msg.fsm_state = "DOCKING";            break;
    }
    // Publish the state
    pub_state_.publish(msg);
    // Debug information for the nodelet
    NODELET_DEBUG_STREAM("Received event " << msg.fsm_event);
    NODELET_DEBUG_STREAM("State changed to " << msg.fsm_state);
    // Send the feedback if needed
    switch (state) {
    case STATE::INITIALIZING:
    case STATE::IDLE:
    case STATE::MOVING:
    case STATE::DOCKING: {
      feedback_.state = msg;
      server_.SendFeedback(feedback_);
      break;
    }
    default:
      break;
    }
  }

  bool Move() {
    // Create a new motion goal
    ff_msgs::MotionGoal m_goal;
    m_goal.command = ff_msgs::MotionGoal::MOVE;
    m_goal.flight_mode = ff_msgs::MotionGoal::NOMINAL;
    m_goal.reference_frame = FRAME_NAME_WORLD;
    // Find the desired berth pose
    geometry_msgs::PoseStamped end_pose;
    end_pose.header.stamp = ros::Time::now();
    std::string berth;
    // If goal berth is specified, overwrite default berth
    if (goal_.berth == ff_msgs::ReturnToDockGoal::BERTH_1)
      berth = "berth1";
    else if (goal_.berth == ff_msgs::ReturnToDockGoal::BERTH_2)
      berth = "berth2";
    try {
      geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
        FRAME_NAME_WORLD, "dock/" + berth + "/approach", ros::Time(0));
      end_pose.pose.position.x = tf.transform.translation.x;
      end_pose.pose.position.y = tf.transform.translation.y;
      end_pose.pose.position.z = tf.transform.translation.z;
      end_pose.pose.orientation = tf.transform.rotation;
      m_goal.states.push_back(end_pose);
    }
    catch (tf2::TransformException &ex) {
      NODELET_ERROR_STREAM("Transform from berth to world failed: " << ex.what());
      return false;
    }
    // Load parameters from config file
    std::string planner = cfg_.Get<std::string>("planner");
    bool coll_check = cfg_.Get<bool>("enable_collision_checking");
    bool face_forward = cfg_.Get<bool>("enable_faceforward");
    bool replanning = cfg_.Get<bool>("enable_replanning");
    int replanning_attempts = cfg_.Get<int>("max_replanning_attempts");
    bool validation = cfg_.Get<bool>("enable_validation");
    bool boostrapping = cfg_.Get<bool>("enable_bootstrapping");
    bool immediate = cfg_.Get<bool>("enable_immediate");
    bool timesync = cfg_.Get<bool>("enable_timesync");
    double desired_vel = cfg_.Get<double>("desired_vel");
    double desired_accel = cfg_.Get<double>("desired_accel");
    double desired_omega = cfg_.Get<double>("desired_omega");
    double desired_alpha = cfg_.Get<double>("desired_alpha");
    // Reconfigure the choreographer/move server
    ff_util::ConfigClient choreographer_cfg_(GetPlatformHandle(), NODE_CHOREOGRAPHER);
    choreographer_cfg_.Set<std::string>("planner", planner);
    choreographer_cfg_.Set<bool>("enable_collision_checking", coll_check);
    choreographer_cfg_.Set<bool>("enable_faceforward", face_forward);
    choreographer_cfg_.Set<bool>("enable_replanning", replanning);
    choreographer_cfg_.Set<int>("max_replanning_attempts", replanning_attempts);
    choreographer_cfg_.Set<bool>("enable_validation", validation);
    choreographer_cfg_.Set<bool>("enable_bootstrapping", boostrapping);
    choreographer_cfg_.Set<bool>("enable_immediate", immediate);
    choreographer_cfg_.Set<bool>("enable_timesync", timesync);
    choreographer_cfg_.Set<double>("desired_vel", desired_vel);
    choreographer_cfg_.Set<double>("desired_accel", desired_accel);
    choreographer_cfg_.Set<double>("desired_omega", desired_omega);
    choreographer_cfg_.Set<double>("desired_alpha", desired_alpha);
    if (!choreographer_cfg_.Reconfigure()) {
      NODELET_ERROR_STREAM("Failed to reconfigure choreographer");
      return false;
    }
    return client_m_.SendGoal(m_goal);
  }

  bool Dock() {
    if (fsm_.GetState() == STATE::MOVING) {
      ff_msgs::DockGoal d_goal;
      d_goal.command = ff_msgs::DockGoal::DOCK;
      d_goal.berth = goal_.berth;
      client_d_.SendGoal(d_goal);
      return true;
    } else {
        return false;
      }
  }

  // Motion feedback - simply send a feedback to return to dock client
  void MFeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback) {
    server_.SendFeedback(feedback_);
  }

  // Motion result - update the FSM accordingly
  void MResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::MotionResultConstPtr const& result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return fsm_.Update(MOVE_SUCCESS);
    default:
      return fsm_.Update(MOVE_FAILED);
    }
  }

  // Dock feedback - simply send a feedback to return to dock client
  void DFeedbackCallback(ff_msgs::DockFeedbackConstPtr const& feedback) {
    server_.SendFeedback(feedback_);
  }

  // Dock result - update the FSM accordingly
  void DResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
    ff_msgs::DockResultConstPtr const& result) {
    switch (result_code) {
    case ff_util::FreeFlyerActionState::SUCCESS:
      return fsm_.Update(DOCK_SUCCESS);
    default:
      return fsm_.Update(DOCK_FAILED);
    }
  }

  void GoalCallback(ff_msgs::ReturnToDockGoalConstPtr const& goal) {
    RESPONSE result;
    switch (fsm_.GetState()) {
    case STATE::IDLE:
      // return to dock flag has to be set to get accepted
      if (goal->command != ff_msgs::ReturnToDockGoal::RETURN_TO_DOCK) {
        result.response = RESPONSE::FAILED;
        return server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
      }
      // load default berth if not specified by goal
      if ((goal->berth == ff_msgs::ReturnToDockGoal::BERTH_1) ||
        (goal->berth == ff_msgs::ReturnToDockGoal::BERTH_2))
        goal_.berth = goal->berth;
      else
        goal_.berth = cfg_.Get<int>("default_berth");
      // Successful request
      return fsm_.Update(GOAL_MOVE);
    default:
      result.response = RESPONSE::FAILED;
      return server_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
    }
  }

  // Preempt the current action with a new action
  void PreemptCallback() {
    NODELET_DEBUG_STREAM("Goal preempted");
    return fsm_.Update(GOAL_PREEMPT);
  }

  // A Cancellation request arrives
  void CancelCallback() {
    NODELET_DEBUG_STREAM("Goal cancelled");
    return fsm_.Update(GOAL_CANCEL);
  }

  // When a new reconfigure request comes in, deal with that request
  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    if ( fsm_.GetState() == STATE::INITIALIZING
      || fsm_.GetState() == STATE::IDLE)
      return cfg_.Reconfigure(config);
    return false;
  }

 private:
  // Server and clients
  ff_util::FreeFlyerActionServer<ff_msgs::ReturnToDockAction> server_;
  ff_util::FreeFlyerActionClient<ff_msgs::MotionAction> client_m_;
  ff_util::FreeFlyerActionClient<ff_msgs::DockAction> client_d_;
  // tf
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // Publisher
  ros::Publisher pub_state_;
  // feedback
  ff_msgs::ReturnToDockFeedback feedback_;
  // fsm
  ff_util::FSM fsm_;
  // cached berth to dock
  ff_msgs::ReturnToDockGoal goal_;
  // Config
  ff_util::ConfigServer cfg_;
};  // class ReturnToDockNodelet

PLUGINLIB_EXPORT_CLASS(return_to_dock::ReturnToDockNodelet, nodelet::Nodelet);

}  // namespace return_to_dock
