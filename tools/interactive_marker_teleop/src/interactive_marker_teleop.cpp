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

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_util/ff_names.h>

#include <string>

using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using visualization_msgs::Marker;

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

ros::Publisher cmd_publisher;
ros::Subscriber ack_subscriber;

tf2_ros::Buffer tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener;

Marker makeMarker(const std::string marker_type) {
  Marker marker;

  // make it yellowish and see through
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.6;
  marker.color.a = 0.8;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // return a different marker using meshes from astrobee_media
  if (marker_type == "cube") {
    marker.type = Marker::CUBE;
    marker.scale.x = 0.3175;
    marker.scale.y = 0.3175;
    marker.scale.z = 0.3175;
  } else if (marker_type == "body") {
    marker.type = Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://astrobee_freeflyer/meshes/body.dae";
  } else if (marker_type == "pmc_left") {
    marker.type = Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://astrobee_freeflyer/meshes/pmc_skin_.dae";
  } else if (marker_type == "pmc_right") {
    marker.type = Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://astrobee_freeflyer/meshes/pmc_skin_.dae";
    marker.pose.orientation.z = 1.0;
  }

  return marker;
}

InteractiveMarkerControl& makeBoxControl(InteractiveMarker& msg) {
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeMarker("body"));
  control.markers.push_back(makeMarker("pmc_left"));
  control.markers.push_back(makeMarker("pmc_right"));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void sendMobilityCommand(std::string command, const geometry_msgs::Pose& desired_pose) {
  if (command == ff_msgs::CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF) {
    // Make ros command message to send to the executive. see `simple_move.cc`
    ff_msgs::CommandStamped move_cmd;
    move_cmd.header.stamp = ros::Time::now();
    move_cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF;
    move_cmd.cmd_id = "interactive_marker" + std::to_string(move_cmd.header.stamp.sec);
    move_cmd.cmd_src = "interactive_marker";
    move_cmd.subsys_name = "Astrobee";

    // Move command has 4 arguements; frame, xyz, xyz tolerance, and rotation
    move_cmd.args.resize(4);
    move_cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
    move_cmd.args[0].s = "world";

    // Set location where you want Astrobee to go to
    move_cmd.args[1].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
    move_cmd.args[1].vec3d[0] = desired_pose.position.x;  // x
    move_cmd.args[1].vec3d[1] = desired_pose.position.y;  // y
    move_cmd.args[1].vec3d[2] = desired_pose.position.z;  // z

    // "Tolerance not used!"
    move_cmd.args[2].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
    move_cmd.args[2].vec3d[0] = 0;
    move_cmd.args[2].vec3d[1] = 0;
    move_cmd.args[2].vec3d[2] = 0;

    // Target attitude, quaternion, only the first 4 values are used
    move_cmd.args[3].data_type = ff_msgs::CommandArg::DATA_TYPE_MAT33f;
    move_cmd.args[3].mat33f[0] = desired_pose.orientation.x;
    move_cmd.args[3].mat33f[1] = desired_pose.orientation.y;
    move_cmd.args[3].mat33f[2] = desired_pose.orientation.z;
    move_cmd.args[3].mat33f[3] = desired_pose.orientation.w;
    move_cmd.args[3].mat33f[4] = 0;
    move_cmd.args[3].mat33f[5] = 0;
    move_cmd.args[3].mat33f[6] = 0;
    move_cmd.args[3].mat33f[7] = 0;
    move_cmd.args[3].mat33f[8] = 0;

    // Send command
    cmd_publisher.publish(move_cmd);
  } else {
    ROS_ERROR("only simplemove supported with pose");
  }
}

void sendMobilityCommand(std::string command) {
  ff_msgs::CommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.subsys_name = "Astrobee";
  cmd.cmd_name = command;
  cmd.cmd_id = command;

  if (command == ff_msgs::CommandConstants::CMD_NAME_DOCK) {
    // Dock has one argument
    cmd.args.resize(1);
    cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_INT;
    cmd.args[0].i = 1;  // TODO(jdekarske) support other berth
  }

  cmd_publisher.publish(cmd);
}

void processFeedback(const InteractiveMarkerFeedbackConstPtr& feedback) {
  switch (feedback->event_type) {
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      switch (feedback->menu_entry_id) {
        case 1:  // snap to astrobee
        {
          // get astrobee position
          geometry_msgs::TransformStamped t;
          t = tfBuffer.lookupTransform("world", "body", ros::Time(0));
          // set marker pose
          geometry_msgs::Pose p;
          p.position.x = t.transform.translation.x;  // Help! is there a better way to do this?
          p.position.y = t.transform.translation.y;
          p.position.z = t.transform.translation.z;
          p.orientation = t.transform.rotation;
          server->setPose("astrobee_teleop", p);
          server->applyChanges();
          break;
        }
        case 2:  // move to desired position
        {
          sendMobilityCommand(ff_msgs::CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF, feedback->pose);
          break;
        }
        case 3:  // dock
        {
          sendMobilityCommand(ff_msgs::CommandConstants::CMD_NAME_DOCK);
          break;
        }
        case 4:  // undock
        {
          sendMobilityCommand(ff_msgs::CommandConstants::CMD_NAME_UNDOCK);
          break;
        }
        default:
          break;
      }
      break;

      // TODO(jdekarske) check for keep out zones here
      // case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      //   ROS_INFO_STREAM(s.str() << ": pose changed"
      //                           << "\nposition = "
      //                           << feedback->pose.position.x
      //                           << ", " << feedback->pose.position.y
      //                           << ", " << feedback->pose.position.z
      //                           << "\norientation = "
      //                           << feedback->pose.orientation.w
      //                           << ", " << feedback->pose.orientation.x
      //                           << ", " << feedback->pose.orientation.y
      //                           << ", " << feedback->pose.orientation.z
      //                           << "\nframe: " << feedback->header.frame_id
      //                           << " time: " << feedback->header.stamp.sec << "sec, "
      //                           << feedback->header.stamp.nsec << " nsec");
      //   break;
  }
}

void make6DofMarker(unsigned int interaction_mode, const geometry_msgs::Pose& position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  int_marker.pose = position;
  int_marker.scale = 1;

  int_marker.name = "astrobee_teleop";
  int_marker.description = "Astrobee Command";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
  orien.normalize();
  tf2::convert(orien, control.orientation);
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf2::convert(orien, control.orientation);
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
  orien.normalize();
  tf2::convert(orien, control.orientation);
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler.apply(*server, int_marker.name);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "interactive_marker_teleop");
  ros::NodeHandle nh;

  // Start tf listener
  tfListener = std::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tfBuffer));

  // Make the marker
  server.reset(new interactive_markers::InteractiveMarkerServer("interactive_marker_teleop", "", false));
  ros::Duration(0.1).sleep();
  menu_handler.insert("Snap to Astrobee", &processFeedback);
  menu_handler.insert("Go to Position", &processFeedback);
  menu_handler.insert("Dock", &processFeedback);
  menu_handler.insert("Undock", &processFeedback);

  // menu_handler.insert("Add to plan", &processFeedback);  // TODO(jdekarske) would be cool to add a new marker for
  // each station
  // menu_handler.insert("Remove from plan", &processFeedback);

  geometry_msgs::Pose position;
  make6DofMarker(visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position);
  server->applyChanges();

  // publish and acknowledge command actions
  cmd_publisher = nh.advertise<ff_msgs::CommandStamped>(TOPIC_COMMAND, 10, true);
  // ack_subscriber = nh.subscribe(TOPIC_MANAGEMENT_ACK, 10, &AckCallback); // TODO(jdekarske) status goes in the marker
  // text

  ros::spin();

  server.reset();
}
