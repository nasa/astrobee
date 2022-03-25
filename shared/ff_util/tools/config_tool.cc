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
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_serialization.h>
#include <ff_util/config_client.h>

// C++ STL includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <memory>

// Gflags
DEFINE_string(ns, "", "Robot namespace");
DEFINE_string(node, "", "Localization pipeline (no, ml, ar, hr)");
DEFINE_string(parameter, "nominal", "Flight mode");
DEFINE_string(type, "", "Path planning algorithm");
DEFINE_string(value, "", "Plan in face-forward mode");

// Avoid sending the command multiple times
bool sent_ = false;

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "teleop", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun ff_util config_client <opts>");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);


  // Create a node handle
  ros::NodeHandle nh(std::string("/") + FLAGS_ns);

  // Config client
  ff_util::ConfigClient cfg(&nh, FLAGS_node);
  if (FLAGS_type == "bool") {
    bool b;
    std::istringstream(FLAGS_value) >> std::boolalpha >> b;
    cfg.Set<bool>(FLAGS_parameter, b);
  } else if (FLAGS_type == "str") {
    cfg.Set<std::string>(FLAGS_parameter, FLAGS_value);
  } else if (FLAGS_type == "double") {
    cfg.Set<double>(FLAGS_parameter, std::stod(FLAGS_value));
  } else {
    std::cout << "You must specify either"
      << "-type bool, -type str, or -type double" << std::endl;
    return 1;
  }

  if (!cfg.Reconfigure()) {
    std::cout << "Could not reconfigure the choreographer node " << std::endl;
    ros::shutdown();
  }

  // Synchronous mode
  ros::spinOnce();
  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
