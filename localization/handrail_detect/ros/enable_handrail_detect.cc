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
#include <ff_msgs/SetBool.h>
#include <ff_util/ff_names.h>

// Simple executable wrapper
int main(int argc, char** argv) {
  // Setup ROS and a NodeHandle pointing to the current group namespace
  ros::init(argc, argv, "enable_handrail_detect", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  // Create a handle to handrail localization enable
  ros::ServiceClient client_e_ = nh.serviceClient<ff_msgs::SetBool>(
    SERVICE_LOCALIZATION_HR_ENABLE);
  // Wait for the enable service to become available
  if (!client_e_.waitForExistence(ros::Duration(5.0)))
    ROS_FATAL("Could not find the handrail enable service");
  // Enable localization
  ff_msgs::SetBool msg;
  msg.request.enable = true;
  if (!client_e_.call(msg))
    ROS_FATAL("Could not call the handrail enable service");
  ros::spin();
  return 0;
}
