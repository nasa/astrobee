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
#include <ros/console.h>
#include <rosgraph_msgs/Log.h>


// C++ STL includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <cmath>

// Robot namespace
DEFINE_string(ns, "", "Robot namespace");

// Toggle based commands
DEFINE_bool(debug, false, "Debug is minimum importance level to print");
DEFINE_bool(info, false, "Info is minimum importance level to print");
DEFINE_bool(warn, false, "Warning is minimum importance level to print");
DEFINE_bool(error, false, "Error is minimum importance level to print");
DEFINE_bool(fatal, false, "Fatal is minimum importance level to print");
DEFINE_bool(only, false, "Listen to only this importance level");

void RosoutCallback(rosgraph_msgs::Log const& msg) {
  // Check that the message does not come from this node
  if (msg.file.find("log_monitor") != std::string::npos)
    return;

  // Print the message if the printing level is right
  if ((msg.level == msg.DEBUG) && FLAGS_debug &&
          (!FLAGS_only | (FLAGS_only && FLAGS_debug)))
    std::cout << "\033[1;34m [DEBUG][" << msg.name << "]:\033[0m\033[34m " << msg.msg << "\033[0m" << std::endl;
  else if ((msg.level == msg.INFO) && (FLAGS_debug | FLAGS_info) &&
          (!FLAGS_only | (FLAGS_only && FLAGS_info)))
    std::cout << "\033[1;32m [INFO][" << msg.name << "]:\033[0m\033[32m  " << msg.msg << "\033[0m" << std::endl;
  else if ((msg.level == msg.WARN) && (FLAGS_debug | FLAGS_info | FLAGS_warn) &&
          (!FLAGS_only | (FLAGS_only && FLAGS_warn)))
    std::cout << "\033[1;33m [WARNING][" << msg.name << "]:\033[0m\033[33m  " << msg.msg << "\033[0m"  << std::endl;
  else if ((msg.level == msg.ERROR) && (FLAGS_debug | FLAGS_info | FLAGS_warn | FLAGS_error) &&
          (!FLAGS_only | (FLAGS_only && FLAGS_error)))
    std::cout << "\033[1;31m [ERROR][" << msg.name << "]:\033[0m\033[31m  " << msg.msg << "\033[0m"  << std::endl;
  else if ((msg.level == msg.FATAL) && (FLAGS_debug | FLAGS_info | FLAGS_warn | FLAGS_error | FLAGS_fatal) &&
          (!FLAGS_only | (FLAGS_only && FLAGS_fatal)))
    std::cout << "\033[1;31m [FATAL][" << msg.name << "]:\033[0m\033[31m  " << msg.msg << "\033[0m"  << std::endl;
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "log_tool", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun log_monito log_tool <opts>");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Some simple checks
  uint8_t cmd = 0;
  if (FLAGS_debug)  cmd++;
  if (FLAGS_info)   cmd++;
  if (FLAGS_warn)   cmd++;
  if (FLAGS_error)  cmd++;
  if (FLAGS_fatal)  cmd++;
  if (cmd != 1) {
    std::cerr << "You must specify one of -debug or -info or -warn or -error or -fatal as threshold" << std::endl;
    return 1;
  }
  if (FLAGS_only == true) {
    std::cerr << "You've chosen to look at only this importance level" << std::endl;;
  }

  // Create a node handle
  ros::NodeHandle nh(std::string("/") + FLAGS_ns);
  // Subscriber to the /rosout topic
  ros::Subscriber rosout_sub  = nh.subscribe("/rosout", 10, RosoutCallback);
  // Synchronous mode
  ros::spin();
  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
