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
#include <ff_common/ff_ros.h>
#include <ff_util/ff_timer.h>

// Actionlib includes
#include <rclcpp_action/rclcpp_action.hpp>

// C++ includes
#include <functional>
#include <memory>
#include <string>

namespace ff_util {

FF_DEFINE_LOGGER("ff_action")

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

// Major changes from ros1 to ros2 are ros2 uses both topics and services to
// implement actions whereas ros1 only used topics. In ros1, new goals
// preempted/canceled the current goal. In ros2, an action server can accept
// multiple goals. This class attempts to apply the ros1 functionality to ros2.
// Please use the ros2 action server if you want to accept multiple goals.
template < typename ActionType >
class FreeFlyerActionServer {
 private:
  // For the purpose of this class, preempt will refer to the case of where
  // a new goal was received before the original goal was accepted and
  // canceled will refer to the case where a new goal was received
  // before the original goal completed.
  // In ros1 the server preempt callback was used for both preempting a goal
  // and canceling a goal so the cancelled state in the above explanation will
  // map to the preempt callback. The new preempt will be invisible to the code
  // using this class.
  enum State {
    WAITING_FOR_CREATE,   /*!< Create() has not been called */
    WAITING_FOR_GOAL,     /*!< Server found, waiting on goal */
    WAITING_FOR_ACCEPTED, /*!< Goal sent but not accepted */
    WAITING_FOR_RESPONSE, /*!< Goal accepted but no feedback/result yet */
    PREEMPTED,            /*!< New goal received before last goal accepted */
    CANCELED              /*!< New goal received before last goal completed */
  };

 public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;

  // Callback types
  typedef std::function < void (std::shared_ptr<const typename ActionType::Goal>)> GoalCallbackType;
  typedef std::function < void (void) > PreemptCallbackType;
  typedef std::function < void (void) > CancelCallbackType;

  // Constructor
  FreeFlyerActionServer() : result_sent_(false), state_(WAITING_FOR_CREATE) {}

  // Destructor
  ~FreeFlyerActionServer() {}

  // Setters for callbacks
  void SetGoalCallback(GoalCallbackType cb_goal) {
    cb_goal_ = cb_goal;
  }

  void SetPreemptCallback(PreemptCallbackType cb_preempt) {
    cb_preempt_ = cb_preempt;
  }

  void SetCancelCallback(CancelCallbackType cb_cancel) {
    cb_cancel_ = cb_cancel;
  }

  // Start the server
  void Create(NodeHandle node, std::string const& topic) {
    sas_ = rclcpp_action::create_server<ActionType>(node,
      topic,
      std::bind(&FreeFlyerActionServer::GoalCallback, this,
                                  std::placeholders::_1, std::placeholders::_2),
      std::bind(&FreeFlyerActionServer::CancelCallback, this,
                                                        std::placeholders::_1),
      std::bind(&FreeFlyerActionServer::AcceptedCallback, this,
                                                        std::placeholders::_1));
    state_ = WAITING_FOR_GOAL;
  }

  // Send incremental feedback for the current goal
  void SendFeedback(std::shared_ptr<typename ActionType::Feedback> const
                                                                    feedback) {
    if (!current_goal_handle_ || !feedback) {
      return;
    }
    current_goal_handle_->publish_feedback(feedback);
  }

  // Send the final result for the current goal
  void SendResult(FreeFlyerActionState::Enum result_code,
                  std::shared_ptr<typename ActionType::Result> const result) {
    if (!current_goal_handle_ || !result) {
      return;
    }

    switch (result_code) {
      case FreeFlyerActionState::SUCCESS:     // Everything worked
        // In ros1, this is where canceled results would go since we were able
        // to successful cancel. Ros2 has an actual canceled function. From the
        // ros2 action diagram, it looks like it is fine to call succeed in the
        // canceling state
        current_goal_handle_->succeed(result);
        break;
      case FreeFlyerActionState::PREEMPTED:
      case FreeFlyerActionState::ABORTED:     // Server encounters an error
        current_goal_handle_->abort(result);
        break;
      default:
        break;
    }

    result_sent_ = true;

    // Check to see if this goal was canceled and there is another goal waiting
    // to be executed
    if (state_ == CANCELED) {
      if (current_goal_handle_->get_goal_id() != latest_uuid_) {
        // Check if the accepted callback was called yet. If not, the goal will
        // be started in the accepted callback
        if (next_goal_handle_) {
          // In this case, the goal was accepted and deferred in the goal
          // callback. Need to set the goal to executing
          next_goal_handle_->execute();
          StartGoal(next_goal_handle_);
          next_goal_handle_ = nullptr;
          return;
        } else {
          current_goal_handle_ = nullptr;
          // Stay in the canceled state so that execute will be called on the
          // goal in the accepted callback
          return;
        }
      }
    }

    current_goal_handle_ = nullptr;
    state_ = WAITING_FOR_GOAL;
  }

 protected:
  // A new action has been called. We have to care of a special case where the
  // goal is preempted or cancelled.
  rclcpp_action::GoalResponse GoalCallback(
                        const rclcpp_action::GoalUUID & uuid,
                        std::shared_ptr<const typename ActionType::Goal> goal) {
    // ROS2 actions are a bit different in that you can reject a goal in the
    // goal callback. So we don't want to start executing the goal in this
    // callback. We will started executing it in the accepted callback.
    // Also, ROS2 allows multiple goals to be executed at one time and this
    // doesn't deal with preemption. Thus this wrapper class will implement
    // preemption. Another note is ROS2 actions are built on topics and services
    // so it is important that the goal and accepted callbacks return quickly to
    // avoid blocking the executor

    // Not sure if this is correct but the goal is part of the goal handle in
    // the accepted callback so the goal passed in to this callback is not
    // being saved.

    latest_uuid_ = uuid;
    // If we are waiting for last goal to go active, this new goal is preempting
    // the last one. Not sure how likely it is to receive a third or forth goal
    // before the first goal is accepted but let's handle it just in case
    if (state_ == WAITING_FOR_ACCEPTED || state_ == PREEMPTED) {
      state_ = PREEMPTED;
      // Accept and start executing the goal in the accepted callback. We will
      // kill previous goals in their accept callback
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else if (state_ == WAITING_FOR_RESPONSE)  {
      // If the current goal is being execution, we need to cancel it.
      // In the ros1 astrobee code, this was referred to as preemption
      state_ = CANCELED;
      result_sent_ = false;
      if (cb_preempt_) {
        cb_preempt_();
      }
      // Check if the result was send. If it was, we can accept and execute the
      // the goal. Otherwise, we need to accept but don't start executing the
      // goal until the previous goal has been completed.
      if (result_sent_) {
        // Set state to waiting for accepted so we don't call execute on an
        // already executing goal
        state_ = WAITING_FOR_ACCEPTED;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      } else {
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
      }
    } else if (state_ == CANCELED) {
      // We are receive yet another goal before working on the last goal or a
      // user canceled a goal and sent a new one before the cancel finished.
      return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    }

    // Nominal case where we are waiting for goal
    state_ = WAITING_FOR_ACCEPTED;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void AcceptedCallback(const std::shared_ptr<GoalHandle> goal_handle) {
    // First check to see if this goal id matches the latest goal id. If it
    // doesn't we don't care about this goal
    if (goal_handle->get_goal_id() != latest_uuid_) {
      std::shared_ptr<typename ActionType::Result> result =
                                std::make_shared<typename ActionType::Result>();
      if (state_ == WAITING_FOR_ACCEPTED) {
      // This cancel will be called when there is one goal that got canceled
      // before its accepted callback was called. This should be very rare.
        goal_handle->canceled(result);
      } else {
        // This abort will get called if multiple goals were received before
        // the accepted callback was called. It is unlikely that more than one
        // goal will preempt the original of canceled goal. However, this code
        // should be able to handle that case. Also this isn't really an abort
        // but the canceled function documentation makes it sound like the goal
        // had to be canceled by the action client before it can be canceled in
        // the server. Since these cases are very unlikely, output a warning in
        // case debugging is needed.
        FF_DEBUG("Rare abort goal case. This may need some attention.");
        goal_handle->abort(result);
      }
    } else {
      // Check if nominal or preempted case. For preempted, we already checked
      // that this was the most recent goal handle
      if (state_ == WAITING_FOR_ACCEPTED || state_ == PREEMPTED) {
        StartGoal(goal_handle);
      } else if (state_ == CANCELED) {
        // First check to see if the old goal has been successfully canceled. We
        // don't want to start the new goal until the old one was successfully
        // canceled.
        if (!current_goal_handle_) {
          // In this case, the goal was accepted and deferred in the goal
          // callback. Need to set the goal to executing
          goal_handle->execute();
          StartGoal(goal_handle);
        } else {
          // If the old goal still needs to be canceled, save the goal handle
          next_goal_handle_ = goal_handle;
        }
      } else {
        // Handle the case were we get a goal handle and don't know what to do
        FF_ERROR("ff action: Got accepted goal callback in state %d.", state_);
        std::shared_ptr<typename ActionType::Result> result =
                                std::make_shared<typename ActionType::Result>();
        goal_handle->abort(result);
      }
    }
  }

  void StartGoal(const std::shared_ptr<GoalHandle> goal_handle) {
    state_ = WAITING_FOR_RESPONSE;
    current_goal_handle_ = goal_handle;
    if (cb_goal_) {
      std::thread(cb_goal_, current_goal_handle_->get_goal()).detach();
    }
  }

  // Handles request to cancel a goal
  rclcpp_action::CancelResponse CancelCallback(
                              const std::shared_ptr<GoalHandle> goal_handle) {
    // Nominal case
    if (state_ == WAITING_FOR_RESPONSE) {
      FF_INFO("FFS: In waiting for response if.");
      // A result must be sent when a goal is canceled. If not, the action
      // server will throw an exception.
      result_sent_ = false;
      state_ = CANCELED;
      if (cb_cancel_) {
        cb_cancel_();
      }
      // Send result if it wasn't sent
      if (!result_sent_) {
        std::shared_ptr<typename ActionType::Result> result =
                                std::make_shared<typename ActionType::Result>();
        // Weirdly sending a canceled result causes a failure/exception in the
        // action server so send succeeded instead which totally makes sense
        goal_handle->succeed(result);
      }
    } else if (state_ == WAITING_FOR_ACCEPTED) {
      // This assumes actions allow a goal to be canceled before fully being
      // accepted
      // Make latest uuid invalid so we have a way to cancel the goal in the
      // accepted callback
      latest_uuid_ = {};
    } else if (state_ == PREEMPTED) {
      // If the cancel was not called on the latest goal, the goal will be
      // aborted anyway. Do we care if it is aborted instead of cancelled?
      if (latest_uuid_ == goal_handle->get_goal_id()) {
        // Set state to waiting for goal. When the accepted callback for the
        // latest goal is called, the uuid check will fail and the goal will be
        // aborted. Again, do we care that the goal was aborted instead of
        // cancelled?
        state_ = WAITING_FOR_GOAL;
        latest_uuid_ = {};
      }
    } else if (state_ == CANCELED) {
      if (current_goal_handle_->get_goal_id() != goal_handle->get_goal_id()) {
        // Check to make sure the latest goal is being canceled. If we had
        // multiple goals come in while the oldest was being canceled, they will
        // be aborted in the accepted callback. Again, do we care that the goals
        // were aborted instead of canceled?
        if (latest_uuid_ == goal_handle->get_goal_id()) {
          // If accepted was called for the new goal, cancel it and set latest
          // uuid to the current goal uuid if that goal hasn't finished
          if (next_goal_handle_) {
            std::shared_ptr<typename ActionType::Result> result =
                                std::make_shared<typename ActionType::Result>();
            next_goal_handle_->canceled(result);
            latest_uuid_ = current_goal_handle_->get_goal_id();
            next_goal_handle_ = nullptr;
          } else {
            // If accepted wasn't called yet, set the latest uuid to be invalid
            // so that it gets aborted in the accepted callback. Again, do we
            // care that the goal gets aborted instead of canceled?
            latest_uuid_ = {};
          }
        }
      }
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

 protected:
  bool result_sent_;
  State state_;
  rclcpp_action::GoalUUID latest_uuid_;
  typename rclcpp_action::Server<ActionType>::SharedPtr sas_;
  typename std::shared_ptr<GoalHandle> current_goal_handle_;
  typename std::shared_ptr<GoalHandle> next_goal_handle_;
  GoalCallbackType cb_goal_;
  PreemptCallbackType cb_preempt_;
  CancelCallbackType cb_cancel_;
};

///////////////////////////////// ACTION CLIENT CODE ///////////////////////////

/**
 * \brief A wrapper class around the simple action client
 *
 * While useful, the simple action client lacks a few features that prove useful
 * in the context of the freeflyer project. Notably, timeouts are not enforced,
 * which leads to the potential situation where the server disappears, and the
 * client remains waiting for a response. To avoid needing to add timers
 * alongside each client, this class internally creates timers to manage the
 * following situations. Zero-values timeouts (default value) are equivalent to
 * no timeout.
 * - Connected timeout - server cannot be connected to within some time.
 * - Active timeout - server does not accept goal by some time.
 * - Response timeout - feedback / result is not received within some time.
 * - Deadline timeout - result is not received within some time.
 * Preemption and cancellation of ones own task will never result in a response.
 * This class also allows construction prior to initialization for use within
 * nodelets.
  **/
template < typename ActionType >
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
  static constexpr double DEFAULT_TIMEOUT_CONNECTED      = 0.0;
  static constexpr double DEFAULT_TIMEOUT_ACTIVE         = 0.0;
  static constexpr double DEFAULT_TIMEOUT_RESPONSE       = 0.0;
  static constexpr double DEFAULT_TIMEOUT_DEADLINE       = 0.0;
  static constexpr double DEFAULT_POLL_DURATION          = 0.1;

 public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

  // Callback types
  typedef std::function < void (
    std::shared_ptr<const typename ActionType::Feedback>) > FeedbackCallbackType;
  typedef std::function < void (FreeFlyerActionState::Enum,
      std::shared_ptr<const typename ActionType::Result>) > ResultCallbackType;
  typedef std::function < void (void) > ConnectedCallbackType;
  typedef std::function < void (void) > ActiveCallbackType;

  // Constructor
  FreeFlyerActionClient() : state_(WAITING_FOR_CREATE),
    to_connect_(DEFAULT_TIMEOUT_CONNECTED),
    to_active_(DEFAULT_TIMEOUT_ACTIVE),
    to_response_(DEFAULT_TIMEOUT_RESPONSE),
    to_deadline_(DEFAULT_TIMEOUT_DEADLINE),
    to_poll_(DEFAULT_POLL_DURATION) {}

  // Destructor
  ~FreeFlyerActionClient() {}

  // Setters for callbacks
  void SetFeedbackCallback(FeedbackCallbackType cb_feedback) {
    cb_feedback_ = cb_feedback;
  }
  void SetResultCallback(ResultCallbackType cb_result) {
    cb_result_ = cb_result;
  }
  void SetConnectedCallback(ConnectedCallbackType cb_connected) {
    cb_connected_ = cb_connected;
  }
  void SetActiveCallback(ActiveCallbackType cb_active) {
    cb_active_ = cb_active;
  }

  // Setters for timeouts
  void SetConnectedTimeout(double to_connect)   { to_connect_ = to_connect;    }
  void SetActiveTimeout(double to_active)       { to_active_ = to_active;      }
  void SetResponseTimeout(double to_response)   { to_response_ = to_response;  }
  void SetDeadlineTimeout(double to_deadline)   { to_deadline_ = to_deadline;  }
  void SetPollTime(double to_poll)              { to_poll_ = to_poll;          }

  // Initialize the action client and return whether connected by end of call
  bool Create(NodeHandle nh, std::string const& topic) {
    // Initialize all timers, but do not start them.
    timer_connected_.createTimer(to_connect_,
        std::bind(&FreeFlyerActionClient::ConnectedTimeoutCallback, this),
        nh, true, false);
    timer_active_.createTimer(to_active_,
        std::bind(&FreeFlyerActionClient::ActiveTimeoutCallback, this),
        nh, true, false);
    timer_response_.createTimer(to_response_,
        std::bind(&FreeFlyerActionClient::ResponseTimeoutCallback, this),
        nh, true, false);
    timer_deadline_.createTimer(to_deadline_,
        std::bind(&FreeFlyerActionClient::DeadlineTimeoutCallback, this),
        nh, true, false);
    timer_poll_.createTimer(to_poll_,
        std::bind(&FreeFlyerActionClient::ConnectPollCallback, this),
        nh, false, false);
    // Initialize the action client
    sac_ = rclcpp_action::create_client<ActionType>(nh, topic);
    // Set the state
    state_ = WAITING_FOR_GOAL;
    // If we have a non-zero connect timeout, this means that we'd like to have
    // the system poll for server ready in the background and callback
    // when the server is ready
    if (to_connect_ != 0) {
      state_ = WAITING_FOR_CONNECT;
      StartOptionalTimer(timer_connected_, to_connect_);
      StartOptionalTimer(timer_poll_, to_poll_);
      ConnectPollCallback();
    }
    // Return whether the server is ready by the end of this function
    return IsConnected();
  }

  // Check if connected
  bool IsConnected() {
    return (sac_ && sac_->action_server_is_ready());
  }

  // Send a new goal
  bool SendGoal(typename ActionType::Goal const& goal) {
    if (!sac_) return false;

    // If we are preempting another goal, stop all timers.
    if (current_goal_handle_) {
      StopAllTimers();
    }
    // In ros1, if a previous goal is active and send goal is called, the old
    // goal is simply forgotten and the new goal is tracked. To preserve this
    // functionality, set the current goal_handle to null so we ignore any
    // feedback or results received from the old goal. The current goal handle
    // will then be assigned to the new goal once the goal is accepted and the
    // accepted callback is called.
    current_goal_handle_ = nullptr;

    // Start the active timer waiting for goal to be accepted
    StartOptionalTimer(timer_active_, to_active_);
    StartOptionalTimer(timer_deadline_, to_deadline_);
    typename rclcpp_action::Client<ActionType>::SendGoalOptions sgo =
                  typename rclcpp_action::Client<ActionType>::SendGoalOptions();
    sgo.goal_response_callback =
                          std::bind(&FreeFlyerActionClient::ActiveCallback,
                          this,
                          std::placeholders::_1);
    sgo.feedback_callback =
                          std::bind(&FreeFlyerActionClient::FeedbackCallback,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2);
    sgo.result_callback = std::bind(&FreeFlyerActionClient::ResultCallback,
                          this,
                          std::placeholders::_1);
    // Send the goal
    sac_->async_send_goal(goal, sgo);
    // Update the state
    state_ = WAITING_FOR_ACTIVE;
    return true;
  }

  // Cancel the goal that is currently running
  bool CancelGoal() {
    if (!sac_ || !current_goal_handle_) return false;
    // Only cancel a goal if we are in the correct state
    switch (state_) {
      case WAITING_FOR_ACTIVE:
      case WAITING_FOR_RESPONSE:
      case WAITING_FOR_DEADLINE:
        StopAllTimers();
        sac_->async_cancel_goal(current_goal_handle_);
        // Set goal handle to null so we stop tracking the goal
        current_goal_handle_ = nullptr;
        state_ = WAITING_FOR_GOAL;
        return true;
      default:
        break;
    }
    return false;
  }

 protected:
  // Simple wrapper around an optional timer
  void StartOptionalTimer(ff_util::FreeFlyerTimer & timer, double duration) {
    if (duration == 0) return;
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
  void Complete(FreeFlyerActionState::Enum state,
                std::shared_ptr<const typename ActionType::Result> result) {
    // Stop all timers
    StopAllTimers();
    // Send response
    if (cb_result_)
      cb_result_(state, result);
    // Reset state
    state_ = WAITING_FOR_GOAL;
    current_goal_handle_ = nullptr;
  }

  // Called periodically until the server is connected
  void ConnectPollCallback() {
    if (!sac_ || !sac_->action_server_is_ready()) return;
    timer_connected_.stop();
    timer_poll_.stop();
    state_ = WAITING_FOR_GOAL;
    if (cb_connected_)
      cb_connected_();
  }

  // Called when the server cannot be connected to
  void ConnectedTimeoutCallback() {
    Complete(FreeFlyerActionState::TIMEOUT_ON_CONNECT, nullptr);
  }

  // Called when the goal does not go active
  void ActiveTimeoutCallback() {
    FF_WARN("Freeflyer action timed out on going active.");
    CancelGoal();
    Complete(FreeFlyerActionState::TIMEOUT_ON_ACTIVE, nullptr);
  }

  // Called when the task deadline was not met
  void DeadlineTimeoutCallback() {
    CancelGoal();
    Complete(FreeFlyerActionState::TIMEOUT_ON_DEADLINE, nullptr);
  }

  // Called when no feedback/result is received within a certain period of time
  void ResponseTimeoutCallback() {
    CancelGoal();
    Complete(FreeFlyerActionState::TIMEOUT_ON_RESPONSE, nullptr);
  }

  // Goal is now active, restart timer and switch state
  void ActiveCallback(typename GoalHandle::SharedPtr goal_handle) {
    timer_active_.stop();

    if (!sac_) return;

    // In ros2, it is possible for a goal to be rejected. Check that here and
    // save goal handle for later use.
    current_goal_handle_ = goal_handle;
    if (!current_goal_handle_) {
      Complete(FreeFlyerActionState::ABORTED, nullptr);
    } else {
      // Start a responser timer if required
      StartOptionalTimer(timer_response_, to_response_);
      // If we want to know when goal goes active
      if (cb_active_)
        cb_active_();
      // We are now waiting on a response
      state_ = WAITING_FOR_RESPONSE;
    }
  }

  // Feedback received
  void FeedbackCallback(typename GoalHandle::SharedPtr goal_handle,
          const std::shared_ptr<const typename ActionType::Feedback> feedback) {
    timer_response_.stop();
    // Need to check if this is the newest goal being executed. We can ignore
    // the feedback from preempted/canceled goals
    if (current_goal_handle_ &&
        current_goal_handle_->get_goal_id() == goal_handle->get_goal_id()) {
      // Start a responser timer if required
      StartOptionalTimer(timer_response_, to_response_);
      // Forward the feedback
      if (cb_feedback_)
        cb_feedback_(feedback);
    }
  }

  // Called when a result is received
  void ResultCallback(typename GoalHandle::WrappedResult const& result) {
    if (current_goal_handle_ &&
        current_goal_handle_->get_goal_id() == result.goal_id) {
      // The response we send depends on the state
      result_ = result.result;

      // The response we send depends on the state
      // We don't track canceled goals so we don't need to worry about that
      // return code
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        state_response_ = FreeFlyerActionState::SUCCESS;
      } else {
        // Unfortunately the free flyer action server code had to abort
        // preempted goals so these goals could have either been preempted or
        // aborted. After some poking around in the free flyer code, it looks
        // like the code only cares if the goal succeeded or failed. I.e. there
        // is no check to distinguish between preempted or aborted. If the
        // distinction is necessary, one can use the result since the response
        // portion of the result in all free flyer actions is 0 for preemption.
        state_response_ = FreeFlyerActionState::ABORTED;
      }

      // Stop timers now. If another timer times out before the result delay
      // timer, cancel goal will be called which in this case will try to cancel
      // an already completed goal which will result in a very cryptic error
      StopAllTimers();

      // Call the result callback on the client side
      Complete(state_response_, result_);

      // Return to waiting for a goal
      state_ = WAITING_FOR_GOAL;
    }
  }

 protected:
  State state_;
  double to_connect_;
  double to_active_;
  double to_response_;
  double to_deadline_;
  double to_poll_;
  typename rclcpp_action::Client<ActionType>::SharedPtr sac_;
  FeedbackCallbackType cb_feedback_;
  ResultCallbackType cb_result_;
  ConnectedCallbackType cb_connected_;
  ActiveCallbackType cb_active_;
  typename GoalHandle::SharedPtr current_goal_handle_;
  ff_util::FreeFlyerTimer timer_connected_;
  ff_util::FreeFlyerTimer timer_active_;
  ff_util::FreeFlyerTimer timer_response_;
  ff_util::FreeFlyerTimer timer_deadline_;
  ff_util::FreeFlyerTimer timer_poll_;
  // Save response
  FreeFlyerActionState::Enum state_response_;
  std::shared_ptr<const typename ActionType::Result> result_;
};

}  // namespace ff_util

#endif  // FF_UTIL_FF_ACTION_H_
