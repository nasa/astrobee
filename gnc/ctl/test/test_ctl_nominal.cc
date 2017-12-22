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
#include <ros/console.h>

#include <ff_util/ff_names.h>
#include <actionlib/client/simple_action_client.h>
#include <ff_msgs/ControlAction.h>
#include <ff_msgs/EkfState.h>
#include <memory>

// Publisher and subscribers
ros::Subscriber sub;
std::shared_ptr<actionlib::SimpleActionClient<ff_msgs::ControlAction>> ac;

// Called once when the goal completes
void Done(const actionlib::SimpleClientGoalState& state,
  const ff_msgs::ControlResultConstPtr& result) {
  EXPECT_EQ(result->response, ff_msgs::ControlResult::SUCCESS);
  ros::shutdown();
}

// Called once when the goal becomes active
void Active() {}

// Called every time feedback is received for the goal
void Feedback(const ff_msgs::ControlFeedbackConstPtr& feedback) {}

// When new state info is available
void StateCallback(const ff_msgs::EkfStateConstPtr& state) {
  // Only one message wanted
  sub.shutdown();
  // Send the Goal
  ROS_INFO("Sending goal to move to a pose");
  ff_msgs::ControlGoal goal;
  goal.command = ff_msgs::ControlGoal::NOMINAL;
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
  // Send the goal!
  ac->sendGoal(goal, &Done, &Active, &Feedback);
}

// Holonomic test
TEST(ctl_nominal, Holonomic) {
  // The default namespace is given by the group hierarchy in the luanch file
  ros::NodeHandle nh;
  // Wait for the servcer
  ac = std::shared_ptr<actionlib::SimpleActionClient<ff_msgs::ControlAction>>(
    new actionlib::SimpleActionClient<ff_msgs::ControlAction>(
      nh, ACTION_GNC_CTL_CONTROL, true));
  ROS_INFO("Waiting for action server to start.");
  ac->waitForServer();
  ROS_INFO("Action server started, sending goal.");
  // Syaty the node handle and wait for the state
  ros::NodeHandle n;
  sub = n.subscribe(TOPIC_GNC_EKF, 1000, &StateCallback);
  ros::spin();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Initialize the gtesttest framework
  testing::InitGoogleTest(&argc, argv);
  // Initialize ROS
  ros::init(argc, argv, "test_stop", ros::init_options::AnonymousName);
  // Run all test procedures
  return RUN_ALL_TESTS();
}
