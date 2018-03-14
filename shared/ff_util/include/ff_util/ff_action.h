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

#ifndef FF_UTIL_FF_ACTION_H_
#define FF_UTIL_FF_ACTION_H_

// ROS includes
#include <ros/ros.h>

// Actionlib includes
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// C++11 includes
#include <functional>
#include <memory>
#include <string>

namespace ff_util {

/////////////////////////////// ACTION SERVER CODE /////////////////////////////

// Possible result codes
class FreeFlyerActionState {
 public:
  enum Enum {
    SUCCESS               =  1,   // Goal succeeded
    PREEMPTED             =  0,   // Newer goal preempted previous goal
    ABORTED               = -1,   // Goal was aborted by the server (cancelled goals don't call back!)
    TIMEOUT_ON_CONNECT    = -2,   // The server did not accept the goal within CONNECT_TIME
    TIMEOUT_ON_ACTIVE     = -3,   // The goal did not go active within ACTIVE TIME
    TIMEOUT_ON_RESPONSE   = -4,   // No feedback or response was received within WATCHDOG TIME
    TIMEOUT_ON_DEADLINE   = -5,   // No result received the given deadline
  };
};

// Wrapper around the simple action server that allows initialization outside the constructor
template < class ActionSpec >
class FreeFlyerActionServer {
 public:
  // Templated action definition
  ACTION_DEFINITION(ActionSpec);

  // Callback types
  typedef std::function < void (GoalConstPtr const&) > GoalCallbackType;
  typedef std::function < void (void) > PreemptCallbackType;
  typedef std::function < void (void) > CancelCallbackType;

  // Constructor
  FreeFlyerActionServer() {}

  // Destructor
  ~FreeFlyerActionServer() {}

  // Setters for callbacks
  void SetGoalCallback(GoalCallbackType cb_goal)          { cb_goal_ = cb_goal;       }
  void SetPreemptCallback(PreemptCallbackType cb_preempt) { cb_preempt_ = cb_preempt; }
  void SetCancelCallback(CancelCallbackType cb_cancel)    { cb_cancel_ = cb_cancel;   }

  // Start the server
  void Create(ros::NodeHandle *nh, std::string const& topic) {
    sas_ = std::shared_ptr < actionlib::SimpleActionServer < ActionSpec > >(
      new actionlib::SimpleActionServer < ActionSpec > (*nh, topic, false));
    sas_->registerGoalCallback(boost::bind(&FreeFlyerActionServer::GoalCallback, this));
    sas_->registerPreemptCallback(boost::bind(&FreeFlyerActionServer::PreemptCallback, this));
    sas_->start();
  }

  // Send incremental feedback for the current goal
  void SendFeedback(Feedback const& feedback) {
    if (!sas_) return;
    sas_->publishFeedback(feedback);
  }

  // Send the final result for the current goal
  void SendResult(FreeFlyerActionState::Enum result_code, Result const& result) {
    if (!sas_) return;
    switch (result_code) {
    case FreeFlyerActionState::SUCCESS:     // Everything worked
      sas_->setSucceeded(result);
      break;
    case FreeFlyerActionState::PREEMPTED:   // Client B preempts client A
      sas_->setPreempted(result);
      break;
    case FreeFlyerActionState::ABORTED:     // Server encounters an error
      sas_->setAborted(result);
    default:
      break;
    }
  }

 protected:
  // In the case where one goal preempts another, the preempt callback called right before
  // the new goal arrives. In the freeflyer project we differentiate between cancels() and
  // preemptions. This intercepts the preempt call and decides what to do.
  void PreemptCallback() {
    if (sas_->isNewGoalAvailable()) {
      if (cb_preempt_)
        cb_preempt_();
    } else {
      if (cb_cancel_)
        cb_cancel_();
    }
  }

  // A new action has been called. We have to care of a special case where the goal is
  // cancelled / preempted between arrival and the time this callback is called.
  void GoalCallback() {
    boost::shared_ptr<const Goal> goal = sas_->acceptNewGoal();
    if (cb_goal_)
      cb_goal_(goal);
  }

 protected:
  std::shared_ptr < actionlib::SimpleActionServer < ActionSpec > > sas_;
  GoalCallbackType cb_goal_;
  PreemptCallbackType cb_preempt_;
  CancelCallbackType cb_cancel_;
};

//////////////////////////////////////// ACTION CLIENT CODE ///////////////////////////////////

/**
 * \brief A wrapper class around the simple action client
 *
 * While useful, the simple action client lacks a few features that prove useful in the context
 * of the freeflyer project. Notably, timeouts are not enforced, which leads to the potential 
 * situation where the server disappears, and the client remains waiting for a response. To avoid
 * needing to add timers alongside each client, this class internally creates timers to manage
 * the following situations. Zero-values timeouts (default value) are equivalent to no timeout.
 * - Connected timeout - server cannot be connected to within some time.
 * - Active timeout - server does not accept goal by some time.
 * - Response timeout - feedback / result is not received within some time.
 * - Deadline timeout - result is not received within some time.
 * Preemption and cancellation of ones own task will never result in a response. This class
 * also allows construction prior to initialization for use within nodelets.
  **/
template < class ActionSpec >
class FreeFlyerActionClient {
 private:
  enum State {
    WAITING_FOR_CREATE,   /*!< Create() has not been called */
    WAITING_FOR_CONNECT,  /*!< Server has not been found */
    WAITING_FOR_GOAL,     /*!< Server found, waiting on goal */
    WAITING_FOR_ACTIVE,   /*!< Goal sent but not accepted */
    WAITING_FOR_RESPONSE, /*!< Goal accepted but no feedback/result yet */
    WAITING_FOR_DEADLINE  /*!< Deadline */
  };
  static constexpr double DEFAULT_TIMEOUT_CONNECTED = 0.0;
  static constexpr double DEFAULT_TIMEOUT_ACTIVE    = 0.0;
  static constexpr double DEFAULT_TIMEOUT_RESPONSE  = 0.0;
  static constexpr double DEFAULT_TIMEOUT_DEADLINE  = 0.0;
  static constexpr double DEFAULT_POLL_DURATION     = 0.1;

 public:
  // Templated action definition
  ACTION_DEFINITION(ActionSpec);

  // Callback types
  typedef std::function < void (FeedbackConstPtr const&) > FeedbackCallbackType;
  typedef std::function < void (FreeFlyerActionState::Enum, ResultConstPtr const&) > ResultCallbackType;
  typedef std::function < void (void) > ConnectedCallbackType;
  typedef std::function < void (void) > ActiveCallbackType;

  // Constructor
  FreeFlyerActionClient() : state_(WAITING_FOR_CREATE),
    to_connected_(DEFAULT_TIMEOUT_CONNECTED),
    to_active_(DEFAULT_TIMEOUT_ACTIVE),
    to_response_(DEFAULT_TIMEOUT_RESPONSE),
    to_deadline_(DEFAULT_TIMEOUT_DEADLINE),
    to_poll_(DEFAULT_POLL_DURATION) {}

  // Destructor
  ~FreeFlyerActionClient() {}

  // Setters for callbacks
  void SetFeedbackCallback(FeedbackCallbackType cb_feedback)    { cb_feedback_ = cb_feedback;   }
  void SetResultCallback(ResultCallbackType cb_result)          { cb_result_ = cb_result;       }
  void SetConnectedCallback(ConnectedCallbackType cb_connected) { cb_connected_ = cb_connected; }
  void SetActiveCallback(ActiveCallbackType cb_active)          { cb_active_ = cb_active;       }

  // Setters for timeouts
  void SetConnectedTimeout(double to_connected)  { to_connected_ = ros::Duration(to_connected); }
  void SetActiveTimeout(double to_active)        { to_active_ = ros::Duration(to_active);       }
  void SetResponseTimeout(double to_response)    { to_response_ = ros::Duration(to_response);   }
  void SetDeadlineTimeout(double to_deadline)    { to_deadline_ = ros::Duration(to_deadline);   }
  void SetPollTime(double to_poll)               { to_poll_ = ros::Duration(to_poll);           }

  // Initialize the action client and return whether connected by end of call
  bool Create(ros::NodeHandle *nh, std::string const& topic) {
    // Initialize all timers, but do not start them.
    timer_connected_ = nh->createTimer(to_connected_,
      &FreeFlyerActionClient::ConnectedTimeoutCallback, this, true, false);
    timer_active_ = nh->createTimer(to_active_,
      &FreeFlyerActionClient::ActiveTimeoutCallback, this, true, false);
    timer_response_ = nh->createTimer(to_response_,
      &FreeFlyerActionClient::ResponseTimeoutCallback, this, true, false);
    timer_deadline_ = nh->createTimer(to_deadline_,
      &FreeFlyerActionClient::DeadlineTimeoutCallback, this, true, false);
    timer_poll_ = nh->createTimer(to_poll_,
        &FreeFlyerActionClient::ConnectPollCallback, this, false, false);
    // Initialize the action client
    sac_ = std::shared_ptr < actionlib::SimpleActionClient < ActionSpec > > (
      new actionlib::SimpleActionClient < ActionSpec > (*nh, topic, true));
    // Set the state
    state_ = WAITING_FOR_GOAL;
    // If we have a non-zero connect timeout, this means that we'd like to have the system poll
    // for connect in the background and callback when the connection is established
    if (!to_connected_.isZero()) {
      state_ = WAITING_FOR_CONNECT;
      StartOptionalTimer(timer_connected_, to_connected_);
      StartOptionalTimer(timer_poll_, to_poll_);
      ConnectPollCallback(ros::TimerEvent());
    }
    // Return whether the server connected by the end of this function
    return sac_->isServerConnected();
  }

  // Check if connected
  bool IsConnected() {
    return (sac_ && sac_->isServerConnected());
  }

  // Send a new goal
  bool SendGoal(Goal const& goal) {
    if (!sac_) return false;
    // Start the active timer waiting for goal to be accepted
    StartOptionalTimer(timer_active_, to_active_);
    StartOptionalTimer(timer_deadline_, to_deadline_);
    // Send the goal
    sac_->sendGoal(goal,
      boost::bind(&FreeFlyerActionClient::ResultCallback, this, _1, _2),
      boost::bind(&FreeFlyerActionClient::ActiveCallback, this),
      boost::bind(&FreeFlyerActionClient::FeedbackCallback, this, _1));
    // Update the state
    state_ = WAITING_FOR_ACTIVE;
    // Goal accepted
    return true;
  }

  // Cancel the goal that is currently running
  bool CancelGoal() {
    if (!sac_) return false;
    // Only cancel a goal if we are in the correct state
    switch (state_) {
    case WAITING_FOR_ACTIVE:
    case WAITING_FOR_RESPONSE:
    case WAITING_FOR_DEADLINE:
      StopAllTimers();
      sac_->cancelGoal();
      sac_->stopTrackingGoal();
      state_ = WAITING_FOR_GOAL;
      return true;
    default:
      break;
    }
    return false;
  }

 protected:
  // Simple wrapper around an optional timer
  void StartOptionalTimer(ros::Timer & timer, ros::Duration const& duration) {
    if (duration.isZero()) return;
    timer.stop();
    timer.setPeriod(duration);
    timer.start();
  }

  // Stop all timers
  void StopAllTimers() {
    timer_connected_.stop();
    timer_active_.stop();
    timer_response_.stop();
    timer_deadline_.stop();
  }

  // Completes the current goal
  void Complete(FreeFlyerActionState::Enum state, ResultConstPtr const& result) {
    // Stop all timers
    StopAllTimers();
    // Send response
    if (cb_result_)
      cb_result_(state, result);
    // Reset state
    state_ = WAITING_FOR_GOAL;
  }

  // Called periodically until the server is connected
  void ConnectPollCallback(ros::TimerEvent const& event) {
    if (!sac_ || !sac_->isServerConnected()) return;
    timer_connected_.stop();
    timer_poll_.stop();
    state_ = WAITING_FOR_GOAL;
    if (cb_connected_)
      cb_connected_();
  }

  // Called when the server cannot be connected to
  void ConnectedTimeoutCallback(ros::TimerEvent const& event) {
    Complete(FreeFlyerActionState::TIMEOUT_ON_CONNECT, nullptr);
  }

  // Called when the goal does not go active
  void ActiveTimeoutCallback(ros::TimerEvent const& event) {
    CancelGoal();
    Complete(FreeFlyerActionState::TIMEOUT_ON_ACTIVE, nullptr);
  }

  // Called when the task deadline was not met
  void DeadlineTimeoutCallback(ros::TimerEvent const& event) {
    CancelGoal();
    Complete(FreeFlyerActionState::TIMEOUT_ON_DEADLINE, nullptr);
  }

  // Called when no feedback/result is received within a certain period of time
  void ResponseTimeoutCallback(ros::TimerEvent const& event) {
    CancelGoal();
    Complete(FreeFlyerActionState::TIMEOUT_ON_RESPONSE, nullptr);
  }

  // Goal is now active, restart timer and switch state
  void ActiveCallback() {
    timer_active_.stop();
    // Start a responser timer if required
    StartOptionalTimer(timer_response_, to_response_);
    // If we want to know when goal goes active
    if (cb_active_)
      cb_active_();
    // We are now waiting on a response
    state_ = WAITING_FOR_RESPONSE;
  }

  // Feedback received
  void FeedbackCallback(FeedbackConstPtr const& feedback) {
    timer_response_.stop();
    // Start a responser timer if required
    StartOptionalTimer(timer_response_, to_response_);
    // Forward the feedback
    if (cb_feedback_)
      cb_feedback_(feedback);
  }

  // Called when a result is received
  void ResultCallback(actionlib::SimpleClientGoalState const& action_state, ResultConstPtr const& result) {
    // Feedback has been received after a result before. Stop tracking the goal
    // so that this doesn't happen
    sac_->stopTrackingGoal();
    // The response we send depends on the state
    if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED)
      Complete(FreeFlyerActionState::SUCCESS, result);
    else if (action_state == actionlib::SimpleClientGoalState::PREEMPTED)
      Complete(FreeFlyerActionState::PREEMPTED, result);
    else
      Complete(FreeFlyerActionState::ABORTED, result);
    // Return to waiting for a goal
    state_ = WAITING_FOR_GOAL;
  }

 protected:
  State state_;
  ros::Duration to_connected_;
  ros::Duration to_active_;
  ros::Duration to_response_;
  ros::Duration to_deadline_;
  ros::Duration to_poll_;
  std::shared_ptr < actionlib::SimpleActionClient < ActionSpec > > sac_;
  FeedbackCallbackType cb_feedback_;
  ResultCallbackType cb_result_;
  ConnectedCallbackType cb_connected_;
  ActiveCallbackType cb_active_;
  ros::Timer timer_connected_;
  ros::Timer timer_active_;
  ros::Timer timer_response_;
  ros::Timer timer_deadline_;
  ros::Timer timer_poll_;
};

}  // namespace ff_util

#endif  // FF_UTIL_FF_ACTION_H_
