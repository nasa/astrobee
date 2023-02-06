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
#include <tf2_ros/transform_broadcaster.h>

#include <ff_common/init.h>
#include <ff_common/utils.h>
#include <ff_common/ff_ros.h>
#include <gflags/gflags.h>

#if ROS1
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#else
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
namespace geometry_msgs {
typedef msg::PoseStamped PoseStamped;
typedef msg::PoseStamped::SharedPtr PoseStampedPtr;
typedef msg::TransformStamped TransformStamped;
}  // namespace geometry_msgs

#endif

DEFINE_string(input_topic, "/loc/ground_truth",
              "The ground truth topic name.");
DEFINE_string(input_frame, "world",
              "The frame to input to tf2.");
DEFINE_string(output_frame, "ground_truth",
              "The frame to output to tf2.");

std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
NodeHandle node;

void odometry_callback(geometry_msgs::PoseStampedPtr const odometry) {
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = node->get_clock()->now();
  transform.header.frame_id = FLAGS_input_frame;
  transform.child_frame_id = FLAGS_output_frame;
  transform.transform.translation.x = odometry->pose.position.x;
  transform.transform.translation.y = odometry->pose.position.y;
  transform.transform.translation.z = odometry->pose.position.z;
  transform.transform.rotation = odometry->pose.orientation;
  tf_broadcaster->sendTransform(transform);
}

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  rclcpp::init(argc, argv);   \
  node = rclcpp::Node::make_shared("pose_stamped_msg_cnv");

  auto odometry_sub = node->create_subscription<geometry_msgs::PoseStamped>(FLAGS_input_topic, 5, odometry_callback);

  // Initialize the transform broadcaster
  tf_broadcaster =
    std::make_unique<tf2_ros::TransformBroadcaster>(*node);

  rclcpp::spin(node);

  return 0;
}
