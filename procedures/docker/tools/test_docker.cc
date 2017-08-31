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
#include <actionlib/client/simple_action_client.h>

#include <cstring>
#include <iostream>

#include "ff_msgs/DockAction.h"
#include "ff_msgs/UndockAction.h"

void usage() {
  std::cerr << "Usage:" << std::endl;
  std::cerr << "      test_docker {dock | undock}" << std::endl;
}

void doneDCb(const actionlib::SimpleClientGoalState& state,
            const ff_msgs::DockResultConstPtr& result) {
  ROS_INFO("Dock Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result is: %d",
           result->status);
  ros::shutdown();
}

void activeDCb() {
  ROS_INFO("Dock Goal just went active");
}

void feedbackDCb(const ff_msgs::DockFeedback::ConstPtr& feedback) {
  ROS_INFO("Dock Feedback is: %d | %f",
           feedback->status,
           feedback->progress);
}

void doneUCb(const actionlib::SimpleClientGoalState& state,
            const ff_msgs::UndockResultConstPtr& result) {
  ROS_INFO("Undock Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result is: %d",
           result->status);
  ros::shutdown();
}

void activeUCb() {
  ROS_INFO("Dock Goal just went active");
}

void feedbackUCb(const ff_msgs::UndockFeedbackConstPtr& feedback) {
  ROS_INFO("Undock Feedback is: %d | %f",
           feedback->status,
           feedback->progress);
}

int main(int argc, char **argv) {
  bool do_dock = false;
  bool do_undock = false;

  ros::init(argc, argv, "test_docker");

  if ( argc < 2 ) {
    usage();
    return -1;
  }
  if ( strcmp("dock", argv[1]) == 0 ) {
    do_dock = true;
  }
  if ( strcmp("undock", argv[1]) == 0 ) {
    do_undock = true;
  }

  if ( do_dock ) {
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<ff_msgs::DockAction> ac("docking/dock", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    ff_msgs::DockGoal goal;
    goal.header.stamp = ros::Time::now();
    goal.bay = 2;

    ac.sendGoal(goal, &doneDCb, &activeDCb, &feedbackDCb);
    ros::spin();
    return 0;
  }

  if ( do_undock ) {
    actionlib::SimpleActionClient<ff_msgs::UndockAction>
        ac("docking/undock", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    ff_msgs::UndockGoal goal;
    goal.header.stamp = ros::Time::now();

    ac.sendGoal(goal, &doneUCb, &activeUCb, &feedbackUCb);
    ros::spin();
    return 0;
  }
}
