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

// Tests action active timeout
// This tests allows the client to connect to the server, and then kills the
// server. When a goal is sent to the server, after 4 seconds, an active
// timeout result should be triggered showing that it did not receive the goal.

// Required for the test framework
#include <gtest/gtest.h>

// Required for the test cases
#include <ff_common/ff_ros.h>

// Action interface
#include <ff_util/ff_action.h>
#include <ff_util/ff_component.h>

// Use one of the simplest actions
#include <ff_msgs/action/dock.hpp>

// C++ includes
#include <functional>
#include <memory>

FF_DEFINE_LOGGER("test_ff_action_active_timeout")

// SERVER CALLBACKS
class Server : ff_util::FreeFlyerComponent {
 public:
  explicit Server(const rclcpp::NodeOptions& options) :
      ff_util::FreeFlyerComponent(options, "action_server_test", true) {}

  void Initialize(NodeHandle nh) {
    FF_INFO("S:Initialize");
    action_.SetGoalCallback(std::bind(&Server::GoalCallback,
                            this,
                            std::placeholders::_1));
    action_.SetPreemptCallback(std::bind(&Server::PreemptCallback, this));
    action_.SetCancelCallback(std::bind(&Server::CancelCallback, this));
    action_.Create(nh, "test_action");
  }

 protected:
  void GoalCallback(std::shared_ptr<const ff_msgs::action::Dock::Goal> goal) {
    FF_INFO("S:GoalCallback()");
    EXPECT_TRUE(false);
  }

  void CancelCallback() {
    FF_INFO("S:CancelCallback()");
    EXPECT_TRUE(false);
  }

  void PreemptCallback() {
    FF_INFO("S:PreemptCallback()");
    EXPECT_TRUE(false);
  }

 private:
  ff_util::FreeFlyerActionServer<ff_msgs::action::Dock> action_;
};

std::shared_ptr<Server> server = nullptr;

// CLIENT CALLBACKS
class Client : ff_util::FreeFlyerComponent {
 public:
  explicit Client(const rclcpp::NodeOptions& options) :
      ff_util::FreeFlyerComponent(options, "client_test", true) {}

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
  }

 protected:
  void FeedbackCallback(
        const std::shared_ptr<const ff_msgs::action::Dock::Feedback> feedback) {
    FF_INFO("C:FeedbackCallback()");
    EXPECT_TRUE(false);
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum state,
                  std::shared_ptr<const ff_msgs::action::Dock::Result> result) {
    FF_INFO("C:ResultCallback()");
    EXPECT_TRUE(state == ff_util::FreeFlyerActionState::TIMEOUT_ON_ACTIVE);
    rclcpp::shutdown();
  }

  void ConnectedCallback() {
    FF_INFO("C:ConnectedCallback()");
    // Destroy the server
    server = nullptr;
    // Send a goal to the destroyed server!
    ff_msgs::action::Dock::Goal goal = ff_msgs::action::Dock::Goal();
    action_.SendGoal(goal);
  }

  void ActiveCallback() {
    FF_INFO("C:ActiveCallback()");
    EXPECT_TRUE(false);
  }

 private:
  ff_util::FreeFlyerActionClient <ff_msgs::action::Dock> action_;
};

// Perform a test of the simple action client
TEST(ff_action, active_timeout) {
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr nh =
                std::make_shared<rclcpp::Node>("test_ff_action_active_timeout");
  server = std::make_shared<Server>(node_options);
  Client client(node_options);
  server->Initialize(nh);
  client.Initialize(nh);
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
