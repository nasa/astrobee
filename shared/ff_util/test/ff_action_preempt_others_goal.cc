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
  }

  void PreemptCallback() {
    ROS_INFO("S:PreemptCallback()");
  }

  void TimerCallback(ros::TimerEvent const& event) {
    if (messages_++ < 10) {
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

class Client1 : ff_util::FreeFlyerNodelet {
 public:
  Client1() : ff_util::FreeFlyerNodelet("client_test", true) {}

  void Initialize(ros::NodeHandle *nh) {
    // Setters for callbacks
    action_.SetFeedbackCallback(
      std::bind(&Client1::FeedbackCallback, this, std::placeholders::_1));
    action_.SetResultCallback(
      std::bind(&Client1::ResultCallback, this, std::placeholders::_1, std::placeholders::_2));
    action_.SetConnectedCallback(
      std::bind(&Client1::ConnectedCallback, this));
    action_.SetActiveCallback(
      std::bind(&Client1::ActiveCallback, this));
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
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum state, actionlib::TwoIntsResultConstPtr const& result) {
    ROS_INFO("C:ResultCallback()");
    EXPECT_TRUE(state == ff_util::FreeFlyerActionState::PREEMPTED);
  }

  void ConnectedCallback() {
    ROS_INFO("C:ConnectedCallback()");
    actionlib::TwoIntsGoal goal;
    action_.SendGoal(goal);
  }

  void ActiveCallback() {
    ROS_INFO("C:ActiveCallback()");
  }

 private:
  ff_util::FreeFlyerActionClient < actionlib::TwoIntsAction > action_;
};

class Client2 : ff_util::FreeFlyerNodelet {
 public:
  Client2() : ff_util::FreeFlyerNodelet("client_test", true) {}

  void Initialize(ros::NodeHandle *nh) {
    // Setters for callbacks
    action_.SetFeedbackCallback(
      std::bind(&Client2::FeedbackCallback, this, std::placeholders::_1));
    action_.SetResultCallback(
      std::bind(&Client2::ResultCallback, this, std::placeholders::_1, std::placeholders::_2));
    action_.SetConnectedCallback(
      std::bind(&Client2::ConnectedCallback, this));
    action_.SetActiveCallback(
      std::bind(&Client2::ActiveCallback, this));
    // Setters for timeout values
    action_.SetConnectedTimeout(2.0);
    action_.SetActiveTimeout(4.0);
    action_.SetResponseTimeout(4.0);
    action_.SetDeadlineTimeout(10.0);
    // Call connect
    action_.Create(nh, "two_ints_action");
    // Start the timer
    timer_ = nh->createTimer(ros::Duration(1.0), &Client2::TimerCallback, this, true, false);
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
    timer_.start();
  }

  void ActiveCallback() {
    ROS_INFO("C:ActiveCallback()");
  }

  void TimerCallback(ros::TimerEvent const& event) {
    actionlib::TwoIntsGoal goal;
    action_.SendGoal(goal);
  }


 private:
  ff_util::FreeFlyerActionClient < actionlib::TwoIntsAction > action_;
  ros::Timer timer_;
};

// Client 2 preempts client 1
TEST(ff_action, preempt_others_goal) {
  Server server;
  Client1 client1;
  Client2 client2;
  ros::NodeHandle nh("~");
  server.Initialize(&nh);
  client1.Initialize(&nh);
  client2.Initialize(&nh);
  ros::spin();
}
