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

#include <common/init.h>
#include <config_reader/config_reader.h>
#include <msg_conversions/msg_conversions.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

static ros::Publisher pub_vis;
visualization_msgs::MarkerArray marks;

void PublishMarkers() {
  for (unsigned int i = 0; i < marks.markers.size(); i++)
    marks.markers[i].header.stamp = ros::Time();

  pub_vis.publish(marks);
}

void ReadParams(config_reader::ConfigReader *config) {
  // Read config files into lua
  if (!config->ReadFiles()) {
    ROS_ERROR("Error loading rviz visualizer parameters.");
    return;
  }

  marks.markers.clear();
  auto boxes_table = std::make_shared<config_reader::ConfigReader::Table>(config, "boxes");
  int table_size = boxes_table->GetSize();
  // Go through boxes until there are no more, lua table index starts at 1
  for (int i = 1; i < (table_size + 1); ++i) {
    visualization_msgs::Marker marker;
    auto box = std::make_shared<config_reader::ConfigReader::Table>(boxes_table.get(), i);
    auto color = std::make_shared<config_reader::ConfigReader::Table>(box.get(), "color");
    auto scale = std::make_shared<config_reader::ConfigReader::Table>(box.get(), "scale");
    auto pose = std::make_shared<config_reader::ConfigReader::Table>(box.get(), "pose");

    marker.header.frame_id = "world";
    marker.ns = "table_visualization";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = i-1;
    marker.frame_locked = true;

    if (!color->GetReal("r", &marker.color.r))
      marker.color.r = 1.0;
    if (!color->GetReal("g", &marker.color.g))
      marker.color.g = 0.0;
    if (!color->GetReal("b", &marker.color.b))
      marker.color.b = 0.0;
    if (!color->GetReal("a", &marker.color.a))
      marker.color.a = 1.0;

    if (!msg_conversions::config_read_vector(scale.get(), &marker.scale)) {
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
    }

    if (!msg_conversions::config_read_vector(pose.get(), &marker.pose.position)) {
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
    }

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marks.markers.push_back(marker);
  }
}

int main(int argc, char **argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  ros::init(argc, argv, "visualization_node");

  ros::NodeHandle nh("~");

  // Initialize lua config reader
  config_reader::ConfigReader config;
  config.AddFile("tools/visualization_node.config");
  ReadParams(&config);

  ros::Timer config_timer = nh.createTimer(ros::Duration(1), [&config](ros::TimerEvent e) {
      config.CheckFilesUpdated(std::bind(&ReadParams, &config));}, false, true);

  // Use a ltached topic to prevent unecessary chatter on the messaging backbone
  pub_vis = nh.advertise<visualization_msgs::MarkerArray>("table", 1, true);

  // Publish the table markers
  PublishMarkers();

  // Keep spinning unti termination
  ros::spin();

  return 0;
}

