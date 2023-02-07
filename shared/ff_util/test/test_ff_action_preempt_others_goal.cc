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

// Test action preempt others goal
// Client #1 sends a goal on the server connected callback. Client #2 starts a timer
// on the server connected callback. When the timer triggers, the action is preempted.
// The test is successful if Client #1 receives the result that the action was preempted.

// Required for the test framework
#include <gtest/gtest.h>

// Required for the test cases
#include <ff_common/ff_ros.h>

// Action interface
#include <ff_util/ff_action.h>
#include <ff_util/ff_component.h>
#include <ff_util/ff_timer.h>

// Use one of the simplest action
#include <ff_msgs/action/dock.hpp>

// C++ includes
#include <functional>
#include <memory>

FF_DEFINE_LOGGER("test_ff_action_preempt_others_goal")

// SERVER CALLBACKS
class Server : ff_util::FreeFlyerComponent {
 public:
  explicit Server(const rclcpp::NodeOptions& options) :
      ff_util::FreeFlyerComponent(options, "action_server_test", true),
      messages_(0) {}

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
    messages_ = 0;
    timer_.start();
  }

  void CancelCallback() {
    FF_INFO("S:CancelCallback()");
    EXPECT_TRUE(false);
  }

  void PreemptCallback() {
    FF_INFO("S:PreemptCallback()");
    ff_msgs::action::Dock::Result::SharedPtr result =
                              std::make_shared<ff_msgs::action::Dock::Result>();
    action_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
  }

  void TimerCallback() {
    FF_INFO("S:TimerCallback()");
    if (messages_++ < 12) {
      ff_msgs::action::Dock::Feedback::SharedPtr feedback =
                            std::make_shared<ff_msgs::action::Dock::Feedback>();
      action_.SendFeedback(feedback);
    } else {
      timer_.stop();
      ff_msgs::action::Dock::Result::SharedPtr result =
                              std::make_shared<ff_msgs::action::Dock::Result>();
      action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    }
  }

 private:
  int messages_;
  ff_util::FreeFlyerActionServer<ff_msgs::action::Dock> action_;
  ff_util::FreeFlyerTimer timer_;
};

// CLIENT CALLBACKS
class Client1 : ff_util::FreeFlyerComponent {
 public:
  explicit Client1(const rclcpp::NodeOptions& options) :
      ff_util::FreeFlyerComponent(options, "action_client1_test", true) {}

  void Initialize(NodeHandle nh) {
    // Setters for callbacks
    action_.SetFeedbackCallback(std::bind(&Client1::FeedbackCallback,
                                          this,
                                          std::placeholders::_1));
    action_.SetResultCallback(std::bind(&Client1::ResultCallback,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
    action_.SetConnectedCallback(std::bind(&Client1::ConnectedCallback, this));
    action_.SetActiveCallback(std::bind(&Client1::ActiveCallback, this));
    // Setters for timeout values
    action_.SetConnectedTimeout(2.0);
    action_.SetActiveTimeout(4.0);
    action_.SetResponseTimeout(4.0);
    action_.SetDeadlineTimeout(10.0);
    // Call connect
    action_.Create(nh, "test_action");
  }

 protected:
  void FeedbackCallback(
        const std::shared_ptr<const ff_msgs::action::Dock::Feedback> feedback) {
    FF_INFO("C1:FeedbackCallback()");
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum state,
                  std::shared_ptr<const ff_msgs::action::Dock::Result> result) {
    FF_INFO("C1:ResultCallback()");
    EXPECT_TRUE(state == ff_util::FreeFlyerActionState::ABORTED);
  }

  void ConnectedCallback() {
    FF_INFO("C1:ConnectedCallback()");
    ff_msgs::action::Dock::Goal goal = ff_msgs::action::Dock::Goal();
    action_.SendGoal(goal);
  }

  void ActiveCallback() {
    FF_INFO("C1:ActiveCallback()");
  }

 private:
  ff_util::FreeFlyerActionClient<ff_msgs::action::Dock> action_;
};

class Client2 : ff_util::FreeFlyerComponent {
 public:
  explicit Client2(const rclcpp::NodeOptions& options) :
      ff_util::FreeFlyerComponent(options, "action_client2_test", true) {}

  void Initialize(NodeHandle nh) {
    // Create the timer before creating the action since that calls the
    // connected callback
    timer_.createTimer(1.0,
                       std::bind(&Client2::TimerCallback, this),
                       nh,
                       true,
                       false);

    // Setters for callbacks
    action_.SetFeedbackCallback(std::bind(&Client2::FeedbackCallback,
                                          this,
                                          std::placeholders::_1));
    action_.SetResultCallback(std::bind(&Client2::ResultCallback,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
    action_.SetConnectedCallback(std::bind(&Client2::ConnectedCallback, this));
    action_.SetActiveCallback(std::bind(&Client2::ActiveCallback, this));
    // Setters for timeout values
    action_.SetConnectedTimeout(2.0);
    action_.SetActiveTimeout(4.0);
    action_.SetResponseTimeout(4.0);
    action_.SetDeadlineTimeout(10.0);
    // Call connect
    action_.Create(nh, "test_action");
  }

 protected:
  void FeedbackCallback(
        const std::shared_ptr<const ff_msgs::action::Dock::Feedback> feedback) {
    FF_INFO("C2:FeedbackCallback()");
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum state,
                  std::shared_ptr<const ff_msgs::action::Dock::Result> result) {
    FF_INFO("C2:ResultCallback()");
    EXPECT_TRUE(state == ff_util::FreeFlyerActionState::SUCCESS);
    rclcpp::shutdown();
  }

  void ConnectedCallback() {
    FF_INFO("C2:ConnectedCallback()");
    timer_.start();
  }

  void ActiveCallback() {
    FF_INFO("C2:ActiveCallback()");
  }

  void TimerCallback() {
    FF_INFO("C2:TimerCallback()");
    ff_msgs::action::Dock::Goal goal;
    action_.SendGoal(goal);
  }

 private:
  ff_util::FreeFlyerActionClient<ff_msgs::action::Dock> action_;
  ff_util::FreeFlyerTimer timer_;
};

// Client 2 preempts client 1
TEST(ff_action, preempt_others_goal) {
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr nh =
          std::make_shared<rclcpp::Node>("test_ff_action_preempt_others_goal");
  Server server(node_options);
  Client1 client1(node_options);
  Client2 client2(node_options);
  server.Initialize(nh);
  client1.Initialize(nh);
  client2.Initialize(nh);
  rclcpp::spin(nh);
}

// Required for the test framework
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
