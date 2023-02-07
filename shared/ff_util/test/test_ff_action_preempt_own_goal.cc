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

// Test action preempt own goal
// In this test the client sends a goal when the server connects. After the
// action is active (ActiveCallback) a timer starts, 2 seconds later it
// sends another goal. The test is successful if the new goal starts and
// another active callback is triggered, showing that it started a new action.
// If 2 seconds pass and the action was not preempted it fails.

// Required for the test framework
#include <gtest/gtest.h>

// Required for the test cases
#include <ff_common/ff_ros.h>

// Action interface
#include <ff_util/ff_action.h>
#include <ff_util/ff_component.h>
#include <ff_util/ff_timer.h>

// Use one of the simplest actions
#include <ff_msgs/action/dock.hpp>

// C++ includes
#include <functional>
#include <memory>

FF_DEFINE_LOGGER("test_ff_action_preempt_own_goal")

// SERVER CALLBACKS
class Server : ff_util::FreeFlyerComponent {
 public:
  explicit Server(const rclcpp::NodeOptions& options) :
      ff_util::FreeFlyerComponent(options, "action_server_test", true) {}

  void Initialize(NodeHandle nh) {
    action_.SetGoalCallback(std::bind(&Server::GoalCallback,
                                      this,
                                      std::placeholders::_1));
    action_.SetPreemptCallback(std::bind(&Server::PreemptCallback, this));
    action_.SetCancelCallback(std::bind(&Server::CancelCallback, this));
    action_.Create(nh, "test_action");
    timer_.createTimer(0.2,
                       std::bind(&Server::TimerCallback, this),
                       nh,
                       false,
                       false);
  }

 protected:
  void GoalCallback(std::shared_ptr<const ff_msgs::action::Dock::Goal> goal) {
    FF_INFO("S:GoalCallback()");
    timer_.start();
  }

  void CancelCallback() {
    FF_INFO("S:CancelCallback()");
  }

  void PreemptCallback() {
    FF_INFO("S:PreemptCallback()");
    ff_msgs::action::Dock::Result::SharedPtr result =
                              std::make_shared<ff_msgs::action::Dock::Result>();
    action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
  }

  void TimerCallback() {
    ff_msgs::action::Dock::Feedback::SharedPtr feedback =
                            std::make_shared<ff_msgs::action::Dock::Feedback>();
    action_.SendFeedback(feedback);
  }

 private:
  ff_util::FreeFlyerActionServer<ff_msgs::action::Dock> action_;
  ff_util::FreeFlyerTimer timer_;
};

// CLIENT CALLBACKS

class Client : ff_util::FreeFlyerComponent {
 public:
  explicit Client(const rclcpp::NodeOptions& options) :
      ff_util::FreeFlyerComponent(options, "action_client_test", true),
      preempted_(false) {}

  void Initialize(NodeHandle nh) {
    // Setters for callbacks
    action_.SetFeedbackCallback(std::bind(&Client::FeedbackCallback,
                                          this,
                                          std::placeholders::_1));
    action_.SetResultCallback(std::bind(&Client::ResultCallback,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
    action_.SetConnectedCallback(std::bind(&Client::ConnectedCallback, this));
    action_.SetActiveCallback(std::bind(&Client::ActiveCallback, this));
    // Setters for timeout values
    action_.SetConnectedTimeout(2.0);
    action_.SetActiveTimeout(4.0);
    action_.SetResponseTimeout(4.0);
    action_.SetDeadlineTimeout(10.0);
    // Call connect
    action_.Create(nh, "test_action");
    // Preempt the goal after 2 seconds
    timer_preempt_.createTimer(2.0,
                               std::bind(&Client::TimerPreemptCallback, this),
                               nh,
                               true,
                               false);
    timer_wait_.createTimer(2.0,
                            std::bind(&Client::TimerWaitCallback, this),
                            nh,
                            true,
                            false);
    timer_shutdown_.createTimer(0.2,
                                std::bind(&Client::TimerShutdownCallback, this),
                                nh,
                                true,
                                false);
  }

 protected:
  void FeedbackCallback(
        const std::shared_ptr<const ff_msgs::action::Dock::Feedback> feedback) {
    FF_INFO("C:FeedbackCallback()");
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum state,
                  std::shared_ptr<const ff_msgs::action::Dock::Result> result) {
    FF_INFO("C:ResultCallback()");
    EXPECT_TRUE(false);
  }

  void ConnectedCallback() {
    FF_INFO("C:ConnectedCallback()");
    ff_msgs::action::Dock::Goal goal = ff_msgs::action::Dock::Goal();
    action_.SendGoal(goal);
  }

  void ActiveCallback() {
    FF_INFO("C:ActiveCallback()");
    // In 2 seconds it will issue a preempt command
    if (preempted_ == false) {
      timer_preempt_.start();
      preempted_ = true;
    } else {
      // It preempted and has the other goal active
      EXPECT_TRUE(true);
      // Cancel the action so the action server doesn't get mad that we shutdown
      // in the middle of goal execution
      action_.CancelGoal();
      timer_shutdown_.start();
    }
  }

  // Timer callback
  void TimerPreemptCallback() {
    FF_INFO("C:TimerPreemptCallback");
    timer_preempt_.stop();
    ff_msgs::action::Dock::Goal goal = ff_msgs::action::Dock::Goal();
    action_.SendGoal(goal);
    timer_wait_.start();
  }

  // Timer callback
  void TimerWaitCallback() {
    FF_INFO("C:TimerWaitCallback");
    // It did not preempt the goal, ending the test
    EXPECT_TRUE(false);
    rclcpp::shutdown();
  }

  // Timer callback
  void TimerShutdownCallback() {
    FF_INFO("C:TimerShutdownCallback");
    rclcpp::shutdown();
  }

 private:
  ff_util::FreeFlyerActionClient<ff_msgs::action::Dock> action_;
  ff_util::FreeFlyerTimer timer_preempt_;
  ff_util::FreeFlyerTimer timer_wait_;
  ff_util::FreeFlyerTimer timer_shutdown_;
  bool preempted_;
};

// Perform a test of the simple action client
TEST(ff_action, preempt_own_goal) {
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr nh =
              std::make_shared<rclcpp::Node>("test_ff_action_preempt_own_goal");
  // Initialize before nodehandle is available
  Server server(node_options);
  Client client(node_options);
  // Initialize the client before the server to make things difficult
  client.Initialize(nh);
  server.Initialize(nh);
  // Wait until shutdown is called
  rclcpp::spin(nh);
}

// Required for the test framework
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
