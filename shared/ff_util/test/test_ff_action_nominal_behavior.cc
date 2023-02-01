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

// Test action nominal behaviour
// Client sends a goal, after 5 messages a SUCCESS result is issued.
// Test succeeds if the client receives the success response.

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

FF_DEFINE_LOGGER("test_ff_action_nominal_behavior")

// SERVER CALLBACKS

class Server : ff_util::FreeFlyerComponent {
 public:
  explicit Server(const rclcpp::NodeOptions& options) :
      FreeFlyerComponent(options, "action_server_test", true),
      messages_(ff_msgs::msg::DockState::DOCKING_MAX_STATE) {}

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
    messages_ = ff_msgs::msg::DockState::DOCKING_MAX_STATE;
    EXPECT_EQ(goal->command, ff_msgs::action::Dock::Goal::DOCK);
    EXPECT_EQ(goal->berth, ff_msgs::action::Dock::Goal::BERTH_1);
    EXPECT_FALSE(goal->return_dock);
    timer_.start();
  }

  void CancelCallback() {
    FF_INFO("S:CancelCallback()");
    EXPECT_TRUE(false);
  }

  void PreemptCallback() {
    FF_INFO("S:PreemptCallback()");
    EXPECT_TRUE(false);
  }

  void TimerCallback() {
    FF_INFO("S:TimerCallback");
    if (messages_ > 0) {
      ff_msgs::action::Dock::Feedback::SharedPtr feedback =
                            std::make_shared<ff_msgs::action::Dock::Feedback>();
      feedback->state.state = messages_;
      action_.SendFeedback(feedback);
      messages_--;
    } else {
      timer_.stop();
      ff_msgs::action::Dock::Result::SharedPtr result =
                              std::make_shared<ff_msgs::action::Dock::Result>();
      result->response = ff_msgs::action::Dock::Result::DOCKED;
      result->fsm_result = "Success!";
      action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    }
  }

 private:
  int messages_;
  ff_util::FreeFlyerActionServer<ff_msgs::action::Dock> action_;
  ff_util::FreeFlyerTimer timer_;
};

// CLIENT CALLBACKS

class Client : ff_util::FreeFlyerComponent {
 public:
  explicit Client(const rclcpp::NodeOptions& options) :
      ff_util::FreeFlyerComponent(options, "action_client_test", true),
      messages_(ff_msgs::msg::DockState::DOCKING_MAX_STATE) {}

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
    // Setup a timer to preempt or cancel ones own task
    timer_.createTimer(0.2,
                       std::bind(&Client::TimerCallback, this),
                       nh,
                       true,
                       false);
  }

 protected:
  void FeedbackCallback(
        const std::shared_ptr<const ff_msgs::action::Dock::Feedback> feedback) {
    EXPECT_EQ(feedback->state.state, messages_);
    messages_--;
    FF_INFO("C:FeedbackCallback()");
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum state,
                  std::shared_ptr<const ff_msgs::action::Dock::Result> result) {
    FF_INFO("C:ResultCallback()");
    EXPECT_TRUE(state == ff_util::FreeFlyerActionState::SUCCESS);
    EXPECT_EQ(result->response, ff_msgs::action::Dock::Result::DOCKED);
    EXPECT_EQ(result->fsm_result, "Success!");
    rclcpp::shutdown();
  }

  void ConnectedCallback() {
    FF_INFO("C:ConnectedCallback()");
    ff_msgs::action::Dock::Goal goal = ff_msgs::action::Dock::Goal();
    goal.command = ff_msgs::action::Dock::Goal::DOCK;
    goal.berth = ff_msgs::action::Dock::Goal::BERTH_1;
    goal.return_dock = false;
    action_.SendGoal(goal);
  }

  void ActiveCallback() {
    FF_INFO("C:ActiveCallback()");
  }

  // Timer callback
  void TimerCallback() {
    EXPECT_TRUE(false);
  }

 private:
  int messages_;
  ff_util::FreeFlyerActionClient<ff_msgs::action::Dock> action_;
  ff_util::FreeFlyerTimer timer_;
};

// Perform a test of the freeflyer action
TEST(ff_action, nominal_behaviour) {
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr nh =
              std::make_shared<rclcpp::Node>("test_ff_action_nominal_behavior");
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
