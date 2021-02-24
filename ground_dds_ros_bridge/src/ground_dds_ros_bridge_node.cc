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

#include "ground_dds_ros_bridge/ground_dds_ros_bridge.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ground_dds_ros_bridge");

  ros::NodeHandle nh("~");
  ground_dds_ros_bridge::GroundDdsRosBridge gdrb;
  if (gdrb.Initialize(nh)) {
    ros::spin();
  }

  return 0;
}
