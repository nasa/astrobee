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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

// C++ stuff
#include <map>
#include <string>

// Get linear 1D speed for trackers with ID passed in argument
int main(int argc, char** argv) {
  ros::init(argc, argv, "speed_tool");
  ros::NodeHandle node;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate rate(1.0);
  static std::map<std::string, geometry_msgs::TransformStamped> old;
  while (node.ok()) {
    rate.sleep();
    for (int i = 1; i < argc; i++) {
      geometry_msgs::TransformStamped tf;
      try {
        tf = tfBuffer.lookupTransform("vive", argv[i], ros::Time(0));
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        continue;
      }
      if (old.find(tf.child_frame_id) != old.end()) {
        if ((tf.header.stamp - old[tf.child_frame_id].header.stamp).toSec() == 0)
          continue;
        double speed = 0;
        speed += pow(old[tf.child_frame_id].transform.translation.x - tf.transform.translation.x, 2);
        speed += pow(old[tf.child_frame_id].transform.translation.y - tf.transform.translation.y, 2);
        speed += pow(old[tf.child_frame_id].transform.translation.z - tf.transform.translation.z, 2);
        speed = sqrt(speed) / (tf.header.stamp - old[tf.child_frame_id].header.stamp).toSec();
        ROS_INFO_STREAM("Tracker " << tf.child_frame_id << " speed: " << speed * 1000 << " mm/s");
      }
      old[tf.child_frame_id] = tf;
    }
  }
  return 0;
}
