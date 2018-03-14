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

// Include ROS
#include <ros/ros.h>

// Include FSW
#include <ff_util/ff_names.h>

// Config and feedback messages
#include <ff_msgs/ViveConfig.h>

// C++ includes
#include <string>

// Gflags
DEFINE_string(ns, "", "Robot namespace");
DEFINE_bool(start, false, "Start recording");
DEFINE_bool(stop, false, "Stop recording and calibrate");

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "vive_tool", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun vive vive_tool <opts>");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Some simple checks
  uint8_t mode = 0;
  if (FLAGS_start) {
    // ROS_INFO_STREAM("STARTING");
    mode++;
  }
  if (FLAGS_stop) {
    // ROS_INFO_STREAM("STOPPING");
    mode++;
  }
  if (mode != 1) {
    std::cerr << "You must specify exactly one of -start, -stop" << std::endl;
    return 1;
  }
  // Create a node handle
  ros::NodeHandle nh(std::string("/") + FLAGS_ns);
  // Service for calling calibration
  ros::ServiceClient service = nh.serviceClient<ff_msgs::ViveConfig>(
    SERVICE_LOCALIZATION_VIVE_CONFIG);
  // Synchronous mode
  ff_msgs::ViveConfig msg;
  if (FLAGS_start) msg.request.action = ff_msgs::ViveConfig::Request::START;
  if (FLAGS_stop) msg.request.action = ff_msgs::ViveConfig::Request::STOP;
  bool status = service.call(msg);
  if (!status) {
    std::cerr << "Service call failed" << std::endl;
    return 1;
  }
  // Success and exit
  google::ShutDownCommandLineFlags();
  std::cout << "Service call succeeded" << std::endl;
  return 0;
}
