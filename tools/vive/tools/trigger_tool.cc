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

// ROS stuff
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

// ROS messages
#include <geometry_msgs/TransformStamped.h>

// Get a transform the trackers with ID passed as argument
int main(int argc, char** argv) {
  ros::init(argc, argv, "trigger_tool");
  ros::NodeHandle node;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  for (int i = 1; i < argc; i++) {
    try {
      static geometry_msgs::TransformStamped tf;
      tf = tfBuffer.lookupTransform("world", argv[i],
        ros::Time(0), ros::Duration(5.0));
      ROS_INFO_STREAM(tf);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
    }
  }
  return 0;
}
