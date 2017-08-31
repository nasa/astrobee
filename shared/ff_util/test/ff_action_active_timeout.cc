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

// Required for the test framework
#include <gtest/gtest.h>

// Required for the test cases
#include <ros/ros.h>

// Action interface
#include <ff_util/ff_action.h>
#include <ff_util/ff_nodelet.h>

// Borrow the example from actionlib
#include <actionlib/TwoIntsAction.h>

// C++11 incliudes
#include <functional>
#include <memory>

// SERVER CALLBACKS

class Server : ff_util::FreeFlyerNodelet {
 public:
  Server() : ff_util::FreeFlyerNodelet("server_test", true) {}

  void Initialize(ros::NodeHandle *nh) {
    action_.SetGoalCallback(std::bind(&Server::GoalCallback, this, std::placeholders::_1));
    action_.SetPreemptCallback(std::bind(&Server::PreemptCallback, this));
    action_.SetCancelCallback(std::bind(&Server::CancelCallback, this));
    action_.Create(nh, "two_ints_action");
  }

 protected:
  void GoalCallback(actionlib::TwoIntsGoalConstPtr const& goal) {
    ROS_INFO("S:GoalCallback()");
    EXPECT_TRUE(false);
  }

  void CancelCallback() {
    ROS_INFO("S:CancelCallback()");
    EXPECT_TRUE(false);
  }

  void PreemptCallback() {
    ROS_INFO("S:PreemptCallback()");
    EXPECT_TRUE(false);
  }

 private:
  ff_util::FreeFlyerActionServer < actionlib::TwoIntsAction > action_;
};

std::shared_ptr<Server> server = nullptr;

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
  }

 protected:
  void FeedbackCallback(actionlib::TwoIntsFeedbackConstPtr const& feedback) {
    ROS_INFO("C:FeedbackCallback()");
    EXPECT_TRUE(false);
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum state, actionlib::TwoIntsResultConstPtr const& result) {
    ROS_INFO("C:ResultCallback()");
    EXPECT_TRUE(state == ff_util::FreeFlyerActionState::TIMEOUT_ON_ACTIVE);
    ros::shutdown();
  }

  void ConnectedCallback() {
    ROS_INFO("C:ConnectedCallback()");
    // Destroy the server
    server = nullptr;
    // Send a goal to the destroyed server!
    actionlib::TwoIntsGoal goal;
    action_.SendGoal(goal);
  }

  void ActiveCallback() {
    ROS_INFO("C:ActiveCallback()");
    EXPECT_TRUE(false);
  }

 private:
  ff_util::FreeFlyerActionClient < actionlib::TwoIntsAction > action_;
};

// Perform a test of the simple action client
TEST(ff_action, active_timeout) {
  server = std::shared_ptr<Server>(new Server);
  Client client;
  ros::NodeHandle nh("~");
  server->Initialize(&nh);
  client.Initialize(&nh);
  ros::spin();
}

// Required for the test framework
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ff_action_active_timeout");
  return RUN_ALL_TESTS();
}
