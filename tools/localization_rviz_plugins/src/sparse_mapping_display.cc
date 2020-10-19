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

#include <config_reader/config_reader.h>
#include <localization_common/utilities.h>
#include <localization_measurements/matched_projections_measurement.h>
#include <localization_measurements/measurement_conversions.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <glog/logging.h>

#include <string>

#include "sparse_mapping_display.h"  // NOLINT
#include "utilities.h"               // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;
namespace lm = localization_measurements;

SparseMappingDisplay::SparseMappingDisplay() {
  pose_axes_size_.reset(new rviz::FloatProperty("Pose Axes Size", 0.1, "Pose axes size.", this));

  // TODO(rsoussan): avoid this and pass config path directly to config reader!!!!
  // make sure body_T_nav cam is correct!!!! (bsharp not bumble)
  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  char* argv = "script";
  char** argv_ptr = &argv;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv_ptr);
  const std::string config_path = "/home/rsoussan/astrobee/astrobee";
  const std::string world = "granite";
  const std::string robot_config_file = "config/robots/bsharp.config";
  lc::SetEnvironmentConfigs(config_path, world, robot_config_file);
  config_reader::ConfigReader config;
  config.AddFile("geometry.config");
  if (!config.ReadFiles()) {
    LOG(FATAL) << "Failed to read config files.";
    exit(0);
  }

  nav_cam_T_body_ = (lc::LoadTransform(config, "nav_cam_transform")).inverse();
}

void SparseMappingDisplay::onInitialize() { MFDClass::onInitialize(); }

void SparseMappingDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

void SparseMappingDisplay::clearDisplay() { sparse_mapping_pose_axes_.clear(); }

void SparseMappingDisplay::processMessage(const ff_msgs::VisualLandmarks::ConstPtr& msg) {
  clearDisplay();
  const auto projections_measurement = lm::MakeMatchedProjectionsMeasurement(*msg);
  const float scale = pose_axes_size_->getFloat();
  const gtsam::Pose3 global_T_body = projections_measurement.global_T_cam * nav_cam_T_body_;
  addPoseAsAxis(global_T_body, scale, sparse_mapping_pose_axes_, context_->getSceneManager(), scene_node_);
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::SparseMappingDisplay, rviz::Display)
