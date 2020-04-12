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

// ROS includes
#include <ros/ros.h>

// Rosbag API
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Common messages
#include <geometry_msgs/PoseStamped.h>
#include <ff_hw_msgs/ViveLight.h>
#include <ff_hw_msgs/ViveLighthouses.h>
#include <ff_hw_msgs/ViveTrackers.h>

// Our general code
#include "../vive_solver.h"

// Main entry point of applications
int main(int argc, char **argv) {
  ros::init(argc, argv, "vive_offline");
  ros::NodeHandle nh("~");
  ros::Publisher pub_vive = nh.advertise<nav_msgs::Path>("/vive", 10, true);
  ros::Publisher pub_truth = nh.advertise<nav_msgs::Path>("/ekf", 10, true);

  // Get the bag file
  std::string bagfile;
  if (!nh.getParam("bagfile", bagfile))
    ROS_FATAL("Failed to get the bagfile file.");

  // Get the bag file
  std::string perfile;
  if (!nh.getParam("perfile", perfile))
    ROS_FATAL("Failed to get the perfile file.");

  // Create an offline solver using the parameters passed into the node
  vive_localization::Solver solver(nh);

  // Store trackers and lighthouses
  std::map<std::string, size_t> trackers;
  std::map<std::string, size_t> lighthouses;

  // Tallies
  size_t cnt_light = 0;
  size_t cnt_correction = 0;

  // Store registration pulses
  std::map<uint32_t, ros::Time> nav_cam;
  std::map<uint32_t, ros::Time> dock_cam;

  // Open the bagfile for reading
  ROS_INFO("Adding all measurements");

  rosbag::Bag bag(bagfile);
  for (rosbag::MessageInstance const m : rosbag::View(bag)) {
    std::string const& type = m.getDataType();
    std::string const& topic = m.getTopic();
    // Lighthouses
    if (type == "ff_hw_msgs/ViveLighthouses" && topic == "/hw/vive/lighthouses") {
      ff_hw_msgs::ViveLighthouses::ConstPtr ptr =
        m.instantiate<ff_hw_msgs::ViveLighthouses>();
      std::vector<ff_hw_msgs::ViveLighthouse>::const_iterator i;
      for (i = ptr->lighthouses.begin(); i != ptr->lighthouses.end(); i++)
        lighthouses[i->serial]++;
      solver.Add(ptr, m.getTime());
      continue;
    }
    // Trackers
    if (type == "ff_hw_msgs/ViveTrackers" && topic == "/hw/vive/trackers") {
      ff_hw_msgs::ViveTrackers::ConstPtr ptr =
        m.instantiate<ff_hw_msgs::ViveTrackers>();
      std::vector<ff_hw_msgs::ViveTracker>::const_iterator i;
      for (i = ptr->trackers.begin(); i != ptr->trackers.end(); i++)
        trackers[i->serial]++;
      solver.Add(ptr, m.getTime());
      continue;
    }
    // Light
    if (type == "ff_hw_msgs/ViveLight" && topic == "/hw/vive/light") {
      solver.Add(m.instantiate<ff_hw_msgs::ViveLight>(), m.getTime());
      cnt_light++;
      continue;
    }
    // Corrections
    if (type == "geometry_msgs/PoseStamped" && topic == "/loc/pose") {
      solver.Add(m.instantiate<geometry_msgs::PoseStamped>(), m.getTime());
      cnt_correction++;
      continue;
    }
  }
  bag.close();

  // Data summary
  ROS_INFO_STREAM("- Trackers:");
  std::map<std::string, size_t>::iterator tt;
  for (tt = trackers.begin(); tt != trackers.end(); tt++)
    ROS_INFO_STREAM("  - " << tt->first << ": " << tt->second);
  ROS_INFO_STREAM("- Lighthouses:");
  std::map<std::string, size_t>::iterator lt;
  for (lt = lighthouses.begin(); lt != lighthouses.end(); lt++)
    ROS_INFO_STREAM("  - " << lt->first << ": " << lt->second);
  ROS_INFO_STREAM("- Light: " << cnt_light);
  ROS_INFO_STREAM("- Correction: " << cnt_correction);

  // Try and solve the problem
  ROS_INFO("Solving problem");
  solver.Solve();

  // Get the results
  nav_msgs::Path path_vive;
  solver.GetVive(path_vive);
  pub_vive.publish(path_vive);
  nav_msgs::Path path_truth;
  solver.GetTruth(path_truth);
  pub_truth.publish(path_truth);

  // Write the results to file
  std::ofstream pfile(perfile.c_str());
  if (!pfile.is_open())
    ROS_FATAL("Pose solution file could not be opened for writing");
  std::map<ros::Time, geometry_msgs::Pose> map_vive;
  std::map<ros::Time, geometry_msgs::Pose> map_truth;
  for (auto const& it : path_vive.poses)
    map_vive[it.header.stamp] = it.pose;
  for (auto const& it : path_truth.poses)
    map_truth[it.header.stamp] = it.pose;
  // Work out the poses that overlap
  ros::Time t0 = map_truth.begin()->first;
  for (auto const& truth : map_truth) {
    auto vive = map_vive.find(truth.first);
    if (vive == map_vive.end())
      continue;
    pfile << std::setprecision(5) << (truth.first - t0).toSec()   << ","
      << vive->second.position.x                                  << ","
      << vive->second.position.y                                  << ","
      << vive->second.position.z                                  << ","
      << vive->second.orientation.x                               << ","
      << vive->second.orientation.y                               << ","
      << vive->second.orientation.z                               << ","
      << vive->second.orientation.w                               << ","
      << truth.second.position.x                                  << ","
      << truth.second.position.y                                  << ","
      << truth.second.position.z                                  << ","
      << truth.second.orientation.x                               << ","
      << truth.second.orientation.y                               << ","
      << truth.second.orientation.z                               << ","
      << truth.second.orientation.w                               << std::endl;
  }
  pfile.close();

  // Don't exit or we will lose messages
  ros::spin();

  // Success!
  return 0;
}
