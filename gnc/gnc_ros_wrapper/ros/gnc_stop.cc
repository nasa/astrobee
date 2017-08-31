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

#include <ros/ros.h>
#include <ros/console.h>

#include <ff_util/ff_names.h>
#include <actionlib/client/simple_action_client.h>
#include <ff_msgs/ControlAction.h>
#include <ff_msgs/ControlCommand.h>
#include <ff_msgs/EkfState.h>
#include <memory>

// Publisher and subscribers
ros::Subscriber sub;
std::shared_ptr<actionlib::SimpleActionClient<ff_msgs::ControlAction>> ac;

// Called once when the goal completes
void Done(const actionlib::SimpleClientGoalState& state, const ff_msgs::ControlResultConstPtr& result) {
  ROS_INFO("Goal just completed");
  ros::shutdown();
}

// Called once when the goal becomes active
void Active() {
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void Feedback(const ff_msgs::ControlFeedbackConstPtr& feedback) {
  // ROS_INFO("Got Feedback");
}

// When new state info is available
void StateCallback(const ff_msgs::EkfStateConstPtr& state) {
  // Only one message wanted
  sub.shutdown();
  // Send the Goal
  ROS_INFO("Sending goal to move to a pose");
  ff_msgs::ControlGoal goal;
  goal.mode = ff_msgs::ControlCommand::MODE_STOP;
  ac->sendGoal(goal, &Done, &Active, &Feedback);
}

// Simple executable wrapper
int main(int argc, char** argv) {
  if (argc < 2) {
    ROS_INFO("Usage: <ns>");
    ROS_INFO("- use <ns> = / for a single-freeflyer without a namespace");
    ROS_INFO("- use <ns> = /alpha for a freeflyer called /alpha");
    return 0;
  }
  std::string ns;
  ROS_INFO("Arguments");
  if (argc > 1) {
    ns  = (std::string)(argv[1]);
    ROS_INFO_STREAM("- ns : " << ns);
  }
  // Initialize
  ros::init(argc, argv, "gnc_stop", ros::init_options::AnonymousName);
  ros::NodeHandle nh(ns);
  ac = std::shared_ptr<actionlib::SimpleActionClient<ff_msgs::ControlAction>>(
    new actionlib::SimpleActionClient<ff_msgs::ControlAction>(nh, ACTION_GNC_CTL_CONTROL, true));
  ROS_INFO("Waiting for action server to start.");
  ac->waitForServer();
  ROS_INFO("Action server started, sending goal.");
  sub = nh.subscribe(TOPIC_GNC_EKF, 1000, &StateCallback);
  ros::spin();
  return 0;
}
