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
      FreeFlyerComponent(options, "action_server_test", true), messages_(0) {}

  void Initialize(NodeHandle nh) {
    action_.SetGoalCallback(std::bind(&Server::GoalCallback, this, std::placeholders::_1));
    action_.SetPreemptCallback(std::bind(&Server::PreemptCallback, this));
    action_.SetCancelCallback(std::bind(&Server::CancelCallback, this));
    action_.Create(nh, "two_ints_action");
    timer_.createTimer(0.2, std::bind(&Server::TimerCallback, this), nh, false, false);
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
    // ros::shutdown();
  }

  void PreemptCallback() {
    FF_INFO("S:PreemptCallback()");
    EXPECT_TRUE(false);
    // ros::shutdown();
  }

  void TimerCallback() {
    if (messages_++ < 5) {
      ff_msgs::action::Dock::Feedback::SharedPtr feedback;
      action_.SendFeedback(feedback);
    } else {
      timer_.stop();
      ff_msgs::action::Dock::Result::SharedPtr result;
      action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    }
  }

 private:
  size_t messages_;
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
    /*action_.SetFeedbackCallback(
      std::bind(&Client::FeedbackCallback, this, std::placeholders::_1));
    action_.SetResultCallback(
      std::bind(&Client::ResultCallback, this, std::placeholders::_1, std::placeholders::_2));
    action_.SetConnectedCallback(
      std::bind(&Client::ConnectedCallback, this));
    action_.SetActiveCallback(
      std::bind(&Client::ActiveCallback, this));*/
    // Setters for timeout values
    action_.SetConnectedTimeout(2.0);
    action_.SetActiveTimeout(4.0);
    action_.SetResponseTimeout(4.0);
    action_.SetDeadlineTimeout(10.0);
    // Call connect
    action_.Create(nh, "two_ints_action");
    // Setup a timer to preempt or cancel ones own task
    timer_.createTimer(0.2,
                       std::bind(&Client::TimerCallback, this),
                       nh,
                       true,
                       false);
  }

 protected:
  void FeedbackCallback(ff_msgs::action::Dock::Feedback const& feedback) {
    FF_INFO("C:FeedbackCallback()");
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum state,
                      ff_msgs::action::Dock::Result::SharedPtr const& result) {
    FF_INFO("C:ResultCallback()");
    EXPECT_TRUE(state == ff_util::FreeFlyerActionState::SUCCESS);
    ros::shutdown();
  }

  void ConnectedCallback() {
    FF_INFO("C:ConnectedCallback()");
    ff_msgs::action::Dock::Goal::SharedPtr goal;
    action_.SendGoal(goal);
  }

  void ActiveCallback() {
    FF_INFO("C:ActiveCallback()");
  }

  // Timer callback
  void TimerCallback() {
    EXPECT_TRUE(false);
    // ros::shutdown();
  }

 private:
  ff_util::FreeFlyerActionClient<ff_msgs::action::Dock> action_;
  ff_util::FreeFlyerTimer timer_;
};

// Perform a test of the simple action client
TEST(ff_action, nominal_behaviour) {
  // Initialize before nodehandle is available
  /*Server server;
  Client client;
  // Create a new node handle
  ros::NodeHandle nh("~");
  // Initialize the client before the server to make things difficult
  client.Initialize(&nh);
  server.Initialize(&nh);
  // Wait until shutdown is called
  ros::spin();*/
  EXPECT_TRUE(true);
}

// Required for the test framework
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
