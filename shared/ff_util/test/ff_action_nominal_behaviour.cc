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
// Client seds a goal, after 5 messages a SUCCESS result is issued.
// Test succeeds if the client recieves the success response.

// Required for the test framework
#include <gtest/gtest.h>

// Required for the test cases
#include <ros/ros.h>

// Action interface
#include <ff_util/ff_action.h>
#include <ff_util/ff_nodelet.h>

// Borrow the example from actionlib
#include <actionlib/TwoIntsAction.h>

// C++ includes
#include <functional>
#include <memory>

// SERVER CALLBACKS

class Server : ff_util::FreeFlyerNodelet {
 public:
  Server() : ff_util::FreeFlyerNodelet("server_test", true), messages_(0) {}

  void Initialize(ros::NodeHandle *nh) {
    action_.SetGoalCallback(std::bind(&Server::GoalCallback, this, std::placeholders::_1));
    action_.SetPreemptCallback(std::bind(&Server::PreemptCallback, this));
    action_.SetCancelCallback(std::bind(&Server::CancelCallback, this));
    action_.Create(nh, "two_ints_action");
    timer_ = nh->createTimer(ros::Duration(0.2), &Server::TimerCallback, this, false, false);
  }

 protected:
  void GoalCallback(actionlib::TwoIntsGoalConstPtr const& goal) {
    ROS_INFO("S:GoalCallback()");
    messages_ = 0;
    timer_.start();
  }

  void CancelCallback() {
    ROS_INFO("S:CancelCallback()");
    EXPECT_TRUE(false);
    // ros::shutdown();
  }

  void PreemptCallback() {
    ROS_INFO("S:PreemptCallback()");
    EXPECT_TRUE(false);
    // ros::shutdown();
  }

  void TimerCallback(ros::TimerEvent const& event) {
    if (messages_++ < 5) {
      actionlib::TwoIntsFeedback feedback;
      action_.SendFeedback(feedback);
    } else {
      timer_.stop();
      actionlib::TwoIntsResult result;
      action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
    }
  }

 private:
  size_t messages_;
  ff_util::FreeFlyerActionServer < actionlib::TwoIntsAction > action_;
  ros::Timer timer_;
};

// CLIENT CALLBACKS

class Client : ff_util::FreeFlyerNodelet {
 public:
  Client() : ff_util::FreeFlyerNodelet("client_test", true) {}

  void Initialize(ros::NodeHandle *nh) {
    // Setters for callbacks
    action_.SetFeedbackCallback(
      std::bind(&Client::FeedbackCallback, this, std::placeholders::_1));
    action_.SetResultCallback(
      std::bind(&Client::ResultCallback, this, std::placeholders::_1, std::placeholders::_2));
    action_.SetConnectedCallback(
      std::bind(&Client::ConnectedCallback, this));
    action_.SetActiveCallback(
      std::bind(&Client::ActiveCallback, this));
    // Setters for timeout values
    action_.SetConnectedTimeout(2.0);
    action_.SetActiveTimeout(4.0);
    action_.SetResponseTimeout(4.0);
    action_.SetDeadlineTimeout(10.0);
    // Call connect
    action_.Create(nh, "two_ints_action");
    // Setup a timer to preempt or cancel ones own task
    timer_ = nh->createTimer(ros::Duration(0.2), &Client::TimerCallback, this, true, false);
  }

 protected:
  void FeedbackCallback(actionlib::TwoIntsFeedbackConstPtr const& feedback) {
    ROS_INFO("C:FeedbackCallback()");
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum state, actionlib::TwoIntsResultConstPtr const& result) {
    ROS_INFO("C:ResultCallback()");
    EXPECT_TRUE(state == ff_util::FreeFlyerActionState::SUCCESS);
    ros::shutdown();
  }

  void ConnectedCallback() {
    ROS_INFO("C:ConnectedCallback()");
    actionlib::TwoIntsGoal goal;
    action_.SendGoal(goal);
  }

  void ActiveCallback() {
    ROS_INFO("C:ActiveCallback()");
  }

  // Timer callback
  void TimerCallback(ros::TimerEvent const& event) {
    EXPECT_TRUE(false);
    // ros::shutdown();
  }

 private:
  ff_util::FreeFlyerActionClient < actionlib::TwoIntsAction > action_;
  ros::Timer timer_;
};

// Perform a test of the simple action client
TEST(ff_action, nominal_behaviour) {
  // Initialize before nodehandle is available
  Server server;
  Client client;
  // Create a new node handle
  ros::NodeHandle nh("~");
  // Initialize the client before the server to make things difficult
  client.Initialize(&nh);
  server.Initialize(&nh);
  // Wait until shutdown is called
  ros::spin();
}

// Required for the test framework
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ff_action_active_timeout");
  return RUN_ALL_TESTS();
}
