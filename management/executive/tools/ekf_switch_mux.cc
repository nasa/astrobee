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

// A node to switch the EKF input from landmarks to sparse mapping,
// or something else. This is needed because both landmarks and registration
// need to be switched at the same time.

#include <ros/ros.h>
#include <ros/console.h>

#include <topic_tools/MuxSelect.h>

#include <ff_common/init.h>

#include <iostream>
#include <string>

const std::string kRegistrationNode = "mux_registration";
const std::string kLandmarksNode    = "mux_landmarks";

int main(int argc, char **argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  ros::init(argc, argv, "ekf_switch_mux");

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " [sparse_mapping|marker_tracking]"
              << std::endl;
    return 1;
  }

  ros::NodeHandle n("~");


  if (!ros::service::waitForService("mux_registration/select", 10000)) {
    std::cerr << "Error: no mux services? is mux_registration running?"
              << std::endl;
    return 1;
  }

  topic_tools::MuxSelect reg_sel;
  reg_sel.request.topic = std::string(argv[1]) + "/registration";

  if (!ros::service::call("/mux_registration/select", reg_sel)) {
    std::cerr << "Error: error calling mux_registration/select"
              << std::endl;
    return 1;
  }

  topic_tools::MuxSelect land_sel;
  land_sel.request.topic = std::string(argv[1]) + "/landmarks";

  if (!ros::service::call("/mux_landmarks/select", land_sel)) {
    std::cerr << "Error: error calling mux_landmarks/select"
              << std::endl;
    std::cerr << "NOTE: mux_registration and mux_landmarks are inconsistent!"
              << std::endl;
    return 1;
  }

  return 0;
}
