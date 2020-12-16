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

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>
#include <sensor_msgs/PointCloud2.h>

#include <glog/logging.h>

#include "sparse_mapping_display.h"  // NOLINT

namespace localization_rviz_plugins {
SparseMappingDisplay::SparseMappingDisplay() {
  int ff_argc = 1;
  char* argv = "sparse_mapping_display";
  char** argv_ptr = &argv;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv_ptr);
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
//  config.AddFile("localization.config");
  if (!config.ReadFiles()) {
    LOG(FATAL) << "Failed to read config files.";
  }
  std::string map_file;
  if (!config.GetStr("world_vision_map_filename", &map_file))
    LOG(FATAL) << "SparseMappingDisplay: Failed to load map file.";
  map_.reset(new sparse_mapping::SparseMap(map_file, true));
  map_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("sparse_mapping/map_cloud", 1, true);
}

void SparseMappingDisplay::onInitialize() { drawMap(); }

void SparseMappingDisplay::drawMap() {
  // TODO(rsoussan): Use pcl point cloud when pcl_ros dependency added
  sensor_msgs::PointCloud2 map_cloud;
  map_cloud.header = std_msgs::Header();
  map_cloud.header.frame_id = "world";
  map_cloud.height = 1;
  map_cloud.width = (map_.get())->GetNumLandmarks();
  map_cloud.fields.resize(3);
  map_cloud.fields[0].name = "x";
  map_cloud.fields[0].offset = 0;
  map_cloud.fields[0].datatype = 7;
  map_cloud.fields[0].count = 1;
  map_cloud.fields[1].name = "y";
  map_cloud.fields[1].offset = 4;
  map_cloud.fields[1].datatype = 7;
  map_cloud.fields[1].count = 1;
  map_cloud.fields[2].name = "z";
  map_cloud.fields[2].offset = 8;
  map_cloud.fields[2].datatype = 7;
  map_cloud.fields[2].count = 1;
  map_cloud.is_bigendian = false;
  map_cloud.point_step = 12;
  map_cloud.row_step = map_cloud.point_step * map_cloud.width;
  map_cloud.is_dense = true;
  map_cloud.data.resize(map_cloud.row_step);

  for (int i = 0; i < static_cast<int>(map_cloud.width); ++i) {
    const Eigen::Vector3f point = ((map_.get())->GetLandmarkPosition(i).cast<float>());
    memcpy(&map_cloud.data[map_cloud.point_step * i + 0], &point.x(), 4);
    memcpy(&map_cloud.data[map_cloud.point_step * i + 4], &point.y(), 4);
    memcpy(&map_cloud.data[map_cloud.point_step * i + 8], &point.z(), 4);
  }

  map_cloud.header.stamp = ros::Time::now();
  // TODO(rsoussan): Use ros point_cloud_common instead of publishing when rviz_default_plugin linker issue fixed
  map_cloud_publisher_.publish(map_cloud);
}

void SparseMappingDisplay::reset() {}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::SparseMappingDisplay, rviz::Display)
