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

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using visualization_msgs::Marker;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

Marker makeBox(InteractiveMarker& msg) {
  Marker marker;

  marker.type = Marker::CUBE;  // TODO(jdekarske) change to urdf or mesh?
  marker.scale.x = msg.scale * 0.3175;
  marker.scale.y = msg.scale * 0.3175;
  marker.scale.z = msg.scale * 0.3175;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.6;
  marker.color.a = 0.8;

  return marker;
}

InteractiveMarkerControl& makeBoxControl(InteractiveMarker& msg) {
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void processFeedback(const InteractiveMarkerFeedbackConstPtr& feedback) {
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid) {
    mouse_point_ss << " at " << feedback->mouse_point.x << ", " << feedback->mouse_point.y << ", "
                   << feedback->mouse_point.z << " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type) {
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM(s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str()
                              << ".");
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

  server->applyChanges();
}

////////////////////////////////////////////////////////////////////////////////////

void make6DofMarker(unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "simple_6dof";
  int_marker.description = "Astrobee Command";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if (show_6dof) {
    tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != InteractiveMarkerControl::NONE) menu_handler.apply(*server, int_marker.name);
}

void makeMenuMarker(const tf::Vector3& position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "context_menu";
  int_marker.description = "Context Menu\n(Right Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  Marker marker = makeBox(int_marker);
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler.apply(*server, int_marker.name);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "position_command");
  ros::NodeHandle n;

  server.reset(new interactive_markers::InteractiveMarkerServer("position_command", "", false));

  ros::Duration(0.1).sleep();

  menu_handler.insert("Go to Position", &processFeedback);
  menu_handler.insert("Add to plan", &processFeedback);  // would be cool to add a new marker for each station
  menu_handler.insert("Remove from plan", &processFeedback);
  menu_handler.insert("Dock", &processFeedback);
  menu_handler.insert("Undock", &processFeedback);

  tf::Vector3 position;
  position = tf::Vector3(0, 0, 0);
  make6DofMarker(visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true);
  server->applyChanges();

  ros::spin();

  server.reset();
}
