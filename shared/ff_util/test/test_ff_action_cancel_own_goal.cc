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

// Test action cancel own goal
// In this test the client sends a goal when the server connects. After the
// action is active (ActiveCallback) a timer starts, 2 seconds later it
// sends a cancel goal. The test is successful if the server cancels the goal.
// If 2 seconds pass and the action was not cancelled it fails.

// Required for the test framework
#include <gtest/gtest.h>

// Required for the test cases
#include <ff_common/ff_ros.h>

// Action interface
#include <ff_util/ff_action.h>
#include <ff_util/ff_component.h>
#include <ff_util/ff_timer.h>

// Borrow the example from actionlib
#include <ff_msgs/action/dock.hpp>

// C++ includes
#include <functional>
#include <memory>

FF_DEFINE_LOGGER("test_ff_action_cancel_own_goal")

bool done = false;

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
    // Success
    EXPECT_TRUE(true);
    done = true;

    // ROS2 requires a result when a goal gets canceled. Test that the ff action
    // server sends the result if the calling code forgets to.
    // ff_msgs::action::Dock::Result::SharedPtr result =
    //                        std::make_shared<ff_msgs::action::Dock::Result>();
    // action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    // Can't call shutdown here as it will trigger an interesting error in the
    // ros code. Call it when the timer gets triggered again.
  }

  void PreemptCallback() {
    FF_INFO("S:PreemptCallback()");
  }

  void TimerCallback() {
    FF_INFO("S:TimerCallback()");
    if (!done) {
      ff_msgs::action::Dock::Feedback::SharedPtr feedback =
                            std::make_shared<ff_msgs::action::Dock::Feedback>();
      action_.SendFeedback(feedback);
    } else {
      rclcpp::shutdown();
    }
  }

 private:
  ff_util::FreeFlyerActionServer<ff_msgs::action::Dock> action_;
  ff_util::FreeFlyerTimer timer_;
};

// CLIENT CALLBACKS
class Client : ff_util::FreeFlyerComponent {
 public:
  explicit Client(const rclcpp::NodeOptions& options) :
      ff_util::FreeFlyerComponent(options, "action_client_test", true) {}

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
    // Cancel the goal after 3 seconds
    timer_cancel_.createTimer(2.0,
                              std::bind(&Client::TimerCancelCallback, this),
                              nh,
                              true,
                              false);
    timer_wait_.createTimer(4.0,
                            std::bind(&Client::TimerWaitCallback, this),
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
    EXPECT_TRUE(true);
    rclcpp::shutdown();
  }

  void ConnectedCallback() {
    FF_INFO("C:ConnectedCallback()");
    ff_msgs::action::Dock::Goal goal = ff_msgs::action::Dock::Goal();
    action_.SendGoal(goal);
  }

  void ActiveCallback() {
    FF_INFO("C:ActiveCallback()");
    timer_cancel_.start();
  }

  // Timer callback
  void TimerCancelCallback() {
    timer_cancel_.stop();
    action_.CancelGoal();
    timer_wait_.start();
  }

  // Timer callback
  void TimerWaitCallback() {
    // It did not cancel the goal, ending the test
    EXPECT_TRUE(false);
    rclcpp::shutdown();
  }

 private:
  ff_util::FreeFlyerActionClient<ff_msgs::action::Dock> action_;
  ff_util::FreeFlyerTimer timer_cancel_;
  ff_util::FreeFlyerTimer timer_wait_;
};

// Perform a test of the simple action client
TEST(ff_action, cancel_own_goal) {
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr nh =
              std::make_shared<rclcpp::Node>("test_ff_action_cancel_own_goal");
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
