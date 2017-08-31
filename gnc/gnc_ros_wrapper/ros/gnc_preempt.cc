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
ros::Timer timer;
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

void TimerCallback(const ros::TimerEvent & event) {
  ROS_INFO("Preempting goal");
  ff_msgs::ControlGoal goal;
  goal.mode = ff_msgs::ControlCommand::MODE_STOP;
  ac->sendGoal(goal, &Done, &Active, &Feedback);
}

// When new state info is available
void StateCallback(const ff_msgs::EkfStateConstPtr& state) {
  // Only one message wanted
  sub.shutdown();
  // Send the Goal
  ROS_INFO("Sending goal to move to a pose");
  ff_msgs::ControlGoal goal;
  goal.mode = ff_msgs::ControlCommand::MODE_NOMINAL;
  ff_msgs::ControlState t1, t2, t3;
  // T = 1
  t1.when = ros::Time::now() + ros::Duration(1.0);
  t1.pose.position.x = -0.000177093;
  t1.pose.position.y = -0.0119939;
  t1.pose.position.z = -0.81975;
  t1.pose.orientation.x = -0.00328006;
  t1.pose.orientation.y = -0.00451048;
  t1.pose.orientation.z = 0.00196397;
  t1.pose.orientation.w = 0.999983;
  t1.accel.linear.x = 0.00996053;
  t1.accel.linear.y = 0.0100393;
  t1.accel.linear.z = 2.43021e-05;
  goal.segment.push_back(t1);
  // T = 2
  t2.when = t1.when + ros::Duration(7.141628622);
  t2.pose.position.x = 0.253831;
  t2.pose.position.y = 0.244022;
  t2.pose.position.z = -0.819131;
  t2.pose.orientation.x = -0.00328006;
  t2.pose.orientation.y = -0.00451048;
  t2.pose.orientation.z = 0.00196397;
  t2.pose.orientation.w = 0.999983;
  t2.twist.linear.x = 0.0705828;
  t2.twist.linear.y = 0.0716946;
  t2.twist.linear.z = 0.000168587;
  t2.accel.linear.x = -0.00996053;
  t2.accel.linear.y = -0.0100393;
  t2.accel.linear.z = -2.43021e-05;
  goal.segment.push_back(t2);
  // T = 3
  t3.when = t2.when + ros::Duration(7.141628622);
  t3.pose.position.x = 0.503899;
  t3.pose.position.y = 0.500023;
  t3.pose.position.z = -0.818546;
  t3.pose.orientation.x = -0.00328006;
  t3.pose.orientation.y = -0.00451048;
  t3.pose.orientation.z = 0.00196397;
  t3.pose.orientation.w = 0.999983;
  goal.segment.push_back(t3);
  goal.flight_mode = "nominal";
  // Send the goal!
  ac->sendGoal(goal, &Done, &Active, &Feedback);
  // Start the timer
  timer.start();
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
  ros::init(argc, argv, "gnc_preempt", ros::init_options::AnonymousName);
  ros::NodeHandle nh(ns);
  // Wait for the servcer
  ac = std::shared_ptr<actionlib::SimpleActionClient<ff_msgs::ControlAction>>(
    new actionlib::SimpleActionClient<ff_msgs::ControlAction>(nh, ACTION_GNC_CTL_CONTROL, true));
  ROS_INFO("Waiting for action server to start.");
  ac->waitForServer();
  ROS_INFO("Action server started, sending goal.");
  sub = nh.subscribe(TOPIC_GNC_EKF, 1000, &StateCallback);
  timer = nh.createTimer(ros::Duration(3.0), TimerCallback, true, false);
  ros::spin();
  return 0;
}
