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

#include <string>
#include <vector>
#include <set>
#include "mapper/visualization_functions.h"

namespace visualization_functions {

// Overwrites the given properties of the marker array.
void SetMarkerProperties(const std_msgs::Header &header,
                         const double &life_time,
                         visualization_msgs::MarkerArray* markers) {
  int count = 0;
  for (visualization_msgs::Marker& marker : markers->markers) {
    marker.header = header;
    marker.id = count;
    marker.lifetime = ros::Duration(life_time);
    ++count;
  }
}


void DeleteMarkersTemplate(const std::string &frame_id,
                           visualization_msgs::MarkerArray *marker_array) {
  visualization_msgs::Marker deleteMarker;
  deleteMarker.action = deleteMarker.DELETEALL;
  deleteMarker.scale.x = 0.1;
  deleteMarker.scale.y = 0.1;
  deleteMarker.scale.z = 0.1;
  deleteMarker.header.frame_id = frame_id;
  deleteMarker.ns = "";
  marker_array->markers.push_back(deleteMarker);
}

void SetMarkersForDeletion(visualization_msgs::MarkerArray* marker_array) {
  for (uint i = 0; i < marker_array->markers.size(); i++) {
    marker_array->markers[i].action = visualization_msgs::Marker::DELETE;
  }
}

void DrawObstacleNodes(const std::vector<Eigen::Vector3d> &points,
                       const std::string &frame_id,
                       const double &resolution,
                       visualization_msgs::MarkerArray* marker_array) {
  DrawNodes(points, frame_id, "obstacle", resolution, Color::Blue(), 0.1, marker_array);
}

void DrawTreeNodes(const std::vector<Eigen::Vector3d> &points,
                   const std::string &frame_id,
                   const double &resolution,
                   visualization_msgs::MarkerArray* marker_array) {
  DrawNodes(points, frame_id, "path", resolution, Color::Orange(), 0.1, marker_array);
}

void DrawCollidingNodes(const std::vector<Eigen::Vector3d> &points,
                        const std::string &frame_id,
                        const double &resolution,
                        visualization_msgs::MarkerArray* marker_array) {
  DrawNodes(points, frame_id, "collision", resolution, Color::Red(), 0.9, marker_array);
}

void DrawCollidingNodes(const std::vector<octomap::point3d> &points,
                        const std::string &frame_id,
                        const double &resolution,
                        visualization_msgs::MarkerArray *marker_array) {
  DrawNodes(points, frame_id, "collision", resolution, Color::Red(), 0.9, marker_array);
}

void DrawNodes(const std::vector<Eigen::Vector3d> &points,
               const std::string &frame_id,
               const std::string &ns,  // namespace
               const double &resolution,
               const std_msgs::ColorRGBA &color,
               const double &transparency,  // 0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray *marker_array) {
  // marker_array->markers.clear();

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.color = color;
  marker.color.a = transparency;
  marker.scale.x = resolution;
  marker.scale.y = resolution;
  marker.scale.z = resolution;
  marker.ns = ns;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.pose.orientation.w = 1.0;
  marker.header.seq = 0;
  marker.id = 0;

  // Get the number of requested waypoints
  uint n_w = points.size();

  if (n_w == 0) {
    marker.action = visualization_msgs::Marker::DELETE;
  } else {
    marker.action = visualization_msgs::Marker::ADD;
  }

  for (size_t i = 0; i < n_w; ++i) {
    geometry_msgs::Point NewPoint;
    NewPoint.x = points[i](0);
    NewPoint.y = points[i](1);
    NewPoint.z = points[i](2);
    marker.points.push_back(NewPoint);
    // marker.pose.position = NewPoint;
    // marker_array->markers.push_back(marker);
    // i = i + 1;
  }
  marker_array->markers.push_back(marker);

  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  SetMarkerProperties(header, 0.0, marker_array);
}

void DrawNodes(const std::vector<octomap::point3d> &points,
               const std::string &frame_id,
               const std::string &ns,  // namespace
               const double &resolution,
               const std_msgs::ColorRGBA &color,
               const double &transparency,  // 0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray *marker_array) {
  // marker_array->markers.clear();

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.color = color;
  marker.color.a = transparency;
  marker.scale.x = resolution;
  marker.scale.y = resolution;
  marker.scale.z = resolution;
  marker.ns = ns;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.pose.orientation.w = 1.0;
  marker.header.seq = 0;
  marker.id = 0;

  // Get the number of requested waypoints
  uint n_w = points.size();

  if (n_w == 0) {
    marker.action = visualization_msgs::Marker::DELETE;
  } else {
    marker.action = visualization_msgs::Marker::ADD;
  }

  for (size_t i = 0; i < n_w; ++i) {
    geometry_msgs::Point NewPoint;
    NewPoint.x = points[i].x();
    NewPoint.y = points[i].y();
    NewPoint.z = points[i].z();
    marker.points.push_back(NewPoint);
    // marker.pose.position = NewPoint;
    // marker_array->markers.push_back(marker);
    // i = i + 1;
  }
  marker_array->markers.push_back(marker);

  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  SetMarkerProperties(header, 0.0, marker_array);
}

void MarkerNode(const Eigen::Vector3d &point,
                const std::string &frame_id,
                const std::string &ns,  // namespace
                const double &resolution,
                const std_msgs::ColorRGBA &color,
                const double &transparency,  // 0 -> transparent, 1 -> opaque
                const int &seqNumber,
                visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::CUBE;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;
  marker->color.a = transparency;
  marker->scale.x = resolution;
  marker->scale.y = resolution;
  marker->scale.z = resolution;
  marker->ns = ns;
  marker->header.frame_id = frame_id;
  marker->header.stamp = ros::Time::now();
  marker->pose.orientation.w = 1.0;

  geometry_msgs::Point position_msg;
  position_msg.x = point(0);
  position_msg.y = point(1);
  position_msg.z = point(2);
  marker->pose.position = position_msg;
  marker->header.seq = seqNumber;
}

void PathVisualization(const std::vector<Eigen::Vector3d> &total_path,
                       const std::vector<Eigen::Vector3d> &waypoints,
                       visualization_msgs::MarkerArray *markers) {
    PathVisualization(total_path, waypoints, Color::Green(), "/Path", markers);
}

void PathVisualization(const std::vector<Eigen::Vector3d> &total_path,
                       const std::vector<Eigen::Vector3d> &waypoints,
                       const std_msgs::ColorRGBA &color,
                       const std::string &ns,  // namespace
                       visualization_msgs::MarkerArray *markers) {
    // Initialize edges marker
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = ns;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.id = 0;
    line_list.scale.x = 0.02;  // Line width
    line_list.pose.orientation.w = 1.0;
    line_list.color = color;

    // Populate edges
    uint path_n_points = total_path.size();
    geometry_msgs::Point node;
    if (path_n_points >= 2) {
        node = msg_conversions::eigen_to_ros_point(total_path[0]);
        line_list.points.push_back(node);
        for (uint i = 1; i < total_path.size()-1; i++) {
            node = msg_conversions::eigen_to_ros_point(total_path[i]);
            line_list.points.push_back(node);
            line_list.points.push_back(node);
        }
        node = msg_conversions::eigen_to_ros_point(total_path[total_path.size()-1]);
        line_list.points.push_back(node);
    }

    // Populate the waypoints
    visualization_msgs::Marker endpoints;
    endpoints.header.frame_id = "/world";
    endpoints.header.stamp = ros::Time::now();
    endpoints.ns = ns;
    endpoints.action = visualization_msgs::Marker::ADD;
    endpoints.type = visualization_msgs::Marker::SPHERE_LIST;
    endpoints.id = 1;
    endpoints.scale.x = 0.1;
    endpoints.scale.y = 0.1;
    endpoints.scale.z = 0.1;
    endpoints.pose.orientation.w = 1.0;
    endpoints.color = Color::Blue();
    for (uint i = 0; i < waypoints.size(); i++) {
        node = msg_conversions::eigen_to_ros_point(waypoints[i]);
        endpoints.points.push_back(node);
    }

    markers->markers.push_back(endpoints);
    markers->markers.push_back(line_list);
}

void DrawArrowPoints(const Eigen::Vector3d& p1,
                     const Eigen::Vector3d& p2,
                     const std_msgs::ColorRGBA &color,
                     const double &diameter,
                     visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::ARROW;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;

  marker->points.resize(2);
  marker->points[0] = msg_conversions::eigen_to_ros_point(p1);
  marker->points[1] = msg_conversions::eigen_to_ros_point(p2);
  // EigenPoint2RosPoint(p1, &marker->points[0]);
  // EigenPoint2RosPoint(p2, &marker->points[1]);

  marker->scale.x = diameter * 0.1;
  marker->scale.y = diameter * 2 * 0.1;
  marker->scale.z = 0.1;
  marker->pose.orientation.w = 1.0;
}

}  // namespace visualization_functions
