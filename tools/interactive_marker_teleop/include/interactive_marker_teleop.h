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

#ifndef INTERACTIVE_MARKER_TELEOP_H_
#define INTERACTIVE_MARKER_TELEOP_H_

#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_util/ff_names.h>

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

class InteractiveMarkerTeleop {
 public:
  explicit InteractiveMarkerTeleop(ros::NodeHandle& nh);

  visualization_msgs::Marker makeMarker(const std::string marker_type);

  visualization_msgs::InteractiveMarkerControl& makeBoxControl(visualization_msgs::InteractiveMarker& msg);

  void sendMobilityCommand(std::string command, const geometry_msgs::Pose& desired_pose);

  void sendMobilityCommand(std::string command);

  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  void make6DofMarker(unsigned int interaction_mode, const geometry_msgs::Pose& position);

 private:
  ros::NodeHandle nh;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

  ros::Publisher cmd_publisher_;
  ros::Subscriber ack_subscriber_;

  tf2_ros::Buffer tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
};

#endif  // INTERACTIVE_MARKER_TELEOP_H_
