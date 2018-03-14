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

#ifndef MAPPER_VISUALIZATION_FUNCTIONS_H_
#define MAPPER_VISUALIZATION_FUNCTIONS_H_

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <visualization_msgs/MarkerArray.h>
#include <ff_msgs/Zone.h>
#include <ff_msgs/SetZones.h>
#include <ff_msgs/GetZones.h>
#include <msg_conversions/msg_conversions.h>
#include <Eigen/Core>
#include <string>
#include <vector>
#include <set>

namespace visualization_functions {

// Some colors for visualization markers
class Color : public std_msgs::ColorRGBA {
 public:
  Color() : std_msgs::ColorRGBA() {}
  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
  Color(double red, double green, double blue, double alpha) : Color() {
    r = red;
    g = green;
    b = blue;
    a = alpha;
  }

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 1.0, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

// Overwrites the given properties of the marker array.
void SetMarkerProperties(const std_msgs::Header &header,
                         const double &life_time,
                         visualization_msgs::MarkerArray *markers);

// Delete markers from a given array
void SetMarkersForDeletion(visualization_msgs::MarkerArray *marker_array);

void DeleteMarkersTemplate(  // Delete all markers
  const std::string &frame_id,
  visualization_msgs::MarkerArray *marker_array);

void DrawObstacleNodes(const std::vector<Eigen::Vector3d> &points,
                       const std::string &frame_id,
                       const double &resolution,
                       visualization_msgs::MarkerArray *marker_array);

void DrawTreeNodes(
    const std::vector<Eigen::Vector3d> points,
    const std::string& frame_id,
    const double resolution,
    visualization_msgs::MarkerArray *marker_array);

void DrawCollidingNodes(const std::vector<Eigen::Vector3d> &points,
                        const std::string &frame_id,
                        const double &resolution,
                        visualization_msgs::MarkerArray *marker_array);

void DrawCollidingNodes(const std::vector<octomap::point3d> &points,
                        const std::string &frame_id,
                        const double &resolution,
                        visualization_msgs::MarkerArray *marker_array);

void DrawNodes(const std::vector<Eigen::Vector3d> &points,
               const std::string &frame_id,
               const std::string &ns,  // namespace
               const double &resolution,
               const std_msgs::ColorRGBA &color,
               const double &transparency,  // 0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray *marker_array);

void DrawNodes(const std::vector<octomap::point3d> &points,
               const std::string &frame_id,
               const std::string &ns,  // namespace
               const double &resolution,
               const std_msgs::ColorRGBA &color,
               const double &transparency,  // 0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray *marker_array);

void MarkerNode(const Eigen::Vector3d &point,
                const std::string &frame_id,
                const std::string &ns,  // namespace
                const double &resolution,
                const std_msgs::ColorRGBA &color,
                const double &transparency,  // 0 -> transparent, 1 -> opaque
                const int &seqNumber,
                visualization_msgs::Marker *marker);

void PathVisualization(const std::vector<Eigen::Vector3d> &total_path,
                       const std::vector<Eigen::Vector3d> &waypoints,
                       visualization_msgs::MarkerArray* markers);

void PathVisualization(const std::vector<Eigen::Vector3d> &total_path,
                       const std::vector<Eigen::Vector3d> &waypoints,
                       const std_msgs::ColorRGBA &color,
                       const std::string &ns,  // namespace
                       visualization_msgs::MarkerArray* markers);

void DrawArrowPoints(const Eigen::Vector3d &p1,
                     const Eigen::Vector3d &p2,
                     const std_msgs::ColorRGBA &color,
                     const double &diameter,
                     visualization_msgs::Marker* marker);

}  // namespace visualization_functions

#endif  // MAPPER_VISUALIZATION_FUNCTIONS_H_
