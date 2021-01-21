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
#include <actionlib/client/simple_action_client.h>

// Listen for transforms
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// FSW includes
#include <ff_util/ff_names.h>
#include <ff_util/ff_action.h>
#include <ff_util/config_client.h>

// FF messages
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/EkfState.h>

// C++ STL includes
#include <iostream>
#include <memory>

// Avoid sending the command multiple times
bool stable_ = false;

// Subscriber Ekf
ros::Subscriber sub_ekf_;
ff_util::FreeFlyerActionClient<ff_msgs::MotionAction> client_m_;

tf2_ros::Buffer tf_buffer_;

// Called once when the goal completes
void MResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
  ff_msgs::MotionResultConstPtr const& result) {
  ASSERT_EQ(result->response, ff_msgs::MotionResult::VIOLATES_KEEP_OUT);
  ROS_DEBUG("Test Completed");
  ros::shutdown();
}

// Called every time feedback is received for the goal
void MFeedbackCallback(ff_msgs::MotionFeedbackConstPtr const& feedback) {}



void StateCallback(const ff_msgs::EkfStateConstPtr& state) {
  if (!client_m_.IsConnected()) return;
  else if (state->confidence != ff_msgs::EkfState::CONFIDENCE_GOOD) return;
  else
    sub_ekf_.shutdown();
  sleep(1);
  stable_ = true;
  return;
}

// Keepout zone test
TEST(choreographer_nominal, ZoneBreach) {
  // Get the node handle
  ros::NodeHandle nh;

  tf2_ros::TransformListener tfListener(tf_buffer_);

  // Setup MOBILITY action
  client_m_.SetConnectedTimeout(30.0);
  client_m_.SetActiveTimeout(30.0);
  client_m_.SetResponseTimeout(30.0);
  client_m_.SetFeedbackCallback(std::bind(
    MFeedbackCallback, std::placeholders::_1));
  client_m_.SetResultCallback(std::bind(
    MResultCallback, std::placeholders::_1, std::placeholders::_2));
  client_m_.Create(&nh, ACTION_MOBILITY_MOTION);

  // Wait for Ekf to start publishing to send the motion goal
  sub_ekf_ = nh.subscribe(TOPIC_GNC_EKF, 1000, &StateCallback);

  // This waits until the simulation is in a stable status
  while (!stable_) ros::spinOnce();  // Mobility

  // Configure the planner
  ff_util::ConfigClient cfg(&nh, NODE_CHOREOGRAPHER);

  cfg.Set<bool>("enable_collision_checking", true);
  cfg.Set<bool>("enable_validation", true);
  cfg.Set<bool>("enable_bootstrapping", true);
  cfg.Set<bool>("enable_immediate", true);
  cfg.Set<bool>("enable_timesync", false);
  cfg.Set<bool>("enable_replanning", true);
  cfg.Set<bool>("enable_faceforward", false);
  cfg.Set<std::string>("planner", "trapezoidal");

  if (!cfg.Reconfigure()) {
    std::cout << "Could not reconfigure the choreographer node " << std::endl;
    ASSERT_EQ(true, false);
  }

  // Setup a new mobility goal
  ff_msgs::MotionGoal goal;
  goal.command = ff_msgs::MotionGoal::MOVE;
  goal.flight_mode = ff_msgs::MotionGoal::NOMINAL;

  // Pose that breaks the keepout/keepin zones condition
  geometry_msgs::PoseStamped pose;
  geometry_msgs::TransformStamped tfs = tf_buffer_.lookupTransform(
          std::string(FRAME_NAME_WORLD),
          "body" ,
          ros::Time(0));
  pose.header = tfs.header;
  // pose.header.frame_id = "world";

  pose.pose.position.x = 10.5;
  pose.pose.position.y = -11.5;
  pose.pose.position.z = 4.0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;

  // Package up and send the move goal
  goal.states.push_back(pose);
  // Try and send the goal
  if (!client_m_.SendGoal(goal)) {
    std::cout << "Mobility client did not accept goal" << std::endl;
    ASSERT_EQ(true, false);
  }

  ros::spin();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Initialize the gtesttest framework
  testing::InitGoogleTest(&argc, argv);
  // Initialize ROS
  ros::init(argc, argv, "test_zones_keepout", ros::init_options::AnonymousName);
  // Run all test procedures
  return RUN_ALL_TESTS();
}
