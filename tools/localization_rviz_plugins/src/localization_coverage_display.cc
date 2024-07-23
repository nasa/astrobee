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

#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

#include "localization_coverage_display.h"  // NOLINT

namespace localization_rviz_plugins {
LocalizationCoverageDisplay::LocalizationCoverageDisplay() {
  int ff_argc = 1;
  char argv[] = "localization_coverage_display";
  char* argv_ptr = &argv[0];
  char** argv_ptr_ptr = &argv_ptr;
  cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("localization_coverage/cloud", 1);
  // Load positions from bagfile
  const std::string coverage_bag = std::getenv("COVERAGE_BAG");
  rosbag::Bag bag(coverage_bag, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery("/sparse_mapping/pose"));
  for (const rosbag::MessageInstance msg : view) {
    geometry_msgs::PoseStampedPtr pose_msg = msg.instantiate<geometry_msgs::PoseStamped>();
    if (pose_msg) {
      positions_.emplace_back(localization_common::PoseFromMsg(*pose_msg).translation().cast<float>());
    }
  }
}

void LocalizationCoverageDisplay::onInitialize() { drawMap(); }

void LocalizationCoverageDisplay::drawMap() {
  // TODO(rsoussan): Use pcl point cloud when pcl_ros dependency added
  sensor_msgs::PointCloud2 cloud;
  cloud.header = std_msgs::Header();
  cloud.header.frame_id = "world";
  cloud.height = 1;
  cloud.width = positions_.size();
  cloud.fields.resize(3);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = 7;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = 7;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = 7;
  cloud.fields[2].count = 1;
  cloud.is_bigendian = false;
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.is_dense = true;
  cloud.data.resize(cloud.row_step);

  for (int i = 0; i < static_cast<int>(cloud.width); ++i) {
    const Eigen::Vector3f& point = positions_[i];
    memcpy(&cloud.data[cloud.point_step * i + 0], &point.x(), 4);
    memcpy(&cloud.data[cloud.point_step * i + 4], &point.y(), 4);
    memcpy(&cloud.data[cloud.point_step * i + 8], &point.z(), 4);
  }

  cloud.header.stamp = ros::Time::now();
  // TODO(rsoussan): Use ros point_cloud_common instead of publishing when rviz_default_plugin linker issue fixed
  cloud_publisher_.publish(cloud);
}

void LocalizationCoverageDisplay::reset() {}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::LocalizationCoverageDisplay, rviz::Display)
