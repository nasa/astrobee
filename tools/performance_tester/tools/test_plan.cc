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

// Command line flags
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>

// Include RPOS
#include <ros/ros.h>

// FSW includes
#include <ff_common/ff_names.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_action.h>

// Software messages
#include <ff_msgs/EnableRecording.h>
#include <ff_msgs/PlanStatusStamped.h>
#include <ff_msgs/AckStatus.h>

// C++ STL includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <ctime>
#include <cstdlib>

// Gflags
DEFINE_string(ns, "", "Robot namespace");

DEFINE_string(startup_plan, "", "Input file with rows of type: x y z roll pitch yaw (in degrees).");
DEFINE_string(data_to_disk, "fplan", "ll");
DEFINE_string(move_plan, "", "Output file.");

ros::ServiceClient client_;


// Wait for plan to finish
void PlanCallback(ff_msgs::PlanStatusStamped::ConstPtr const& ps) {
  if (ps->status.status == ff_msgs::AckStatus::COMPLETED) {
    // Stop recording
    ff_msgs::EnableRecording srv;
    srv.request.enable = false;
    if (client_.call(srv))
      ROS_ERROR_STREAM("Result: " << srv.response.success);
    ros::shutdown();
  }
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "test_plan", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun performance_tester test_plan <opts>");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Create a node handle
  ros::NodeHandle nh(std::string("/") + FLAGS_ns);
  // Start bagger service client
  client_ = nh.serviceClient<ff_msgs::EnableRecording>(SERVICE_MANAGEMENT_DATA_BAGGER_ENABLE_RECORDING);


  // Run Startup plan
  std::system(("rosrun executive plan_pub " + FLAGS_startup_plan).c_str());

  // Configure recording data
  std::system(("rosrun executive data_to_disk_pub " + FLAGS_data_to_disk).c_str());

  // Start recording data
  ff_msgs::EnableRecording srv;
  srv.request.enable = true;
  if (client_.call(srv))
    ROS_ERROR_STREAM("Result: " << srv.response.success);

  // Start movement plan
  std::system(("rosrun executive plan_pub " + FLAGS_move_plan).c_str());


  // Monitor plan
  ros::Subscriber sub = nh.subscribe(
    TOPIC_MANAGEMENT_EXEC_PLAN_STATUS, 1, &PlanCallback);

  // Synchronous mode
  ros::spin();
  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
