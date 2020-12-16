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

#include <localization_common/utilities.h>

#include <geometry_msgs/PoseStamped.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <glog/logging.h>

#include <string>

#include "pose_display.h"  // NOLINT
#include "utilities.h"     // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;

PoseDisplay::PoseDisplay() {
  pose_axes_size_.reset(new rviz::FloatProperty("Pose Axes Size", 0.1, "Pose axes size.", this));
  number_of_poses_.reset(new rviz::IntProperty("Number of Poses", 10, "Number of poses to display.", this));
}

void PoseDisplay::onInitialize() { MFDClass::onInitialize(); }

void PoseDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

void PoseDisplay::clearDisplay() { pose_axes_.clear(); }

void PoseDisplay::processMessage(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  pose_axes_.set_capacity(number_of_poses_->getInt());
  const float scale = pose_axes_size_->getFloat();
  const gtsam::Pose3 world_T_body = lc::PoseFromMsg(*msg);
  const auto timestamp = lc::RosTimeFromHeader(msg->header);
  const auto current_frame_T_world = currentFrameTFrame("world", timestamp, *context_);
  if (!current_frame_T_world) {
    LOG(ERROR) << "processMessage: Failed to get current_frame_T_world.";
    return;
  }
  auto axis = axisFromPose((*current_frame_T_world) * world_T_body, scale, context_->getSceneManager(), scene_node_);
  axis->setXColor(Ogre::ColourValue(0.5, 0, 0, 0.3));
  axis->setYColor(Ogre::ColourValue(0, 0.5, 0, 0.3));
  axis->setZColor(Ogre::ColourValue(0, 0, 0.5, 0.3));
  pose_axes_.push_back(std::move(axis));
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::PoseDisplay, rviz::Display)
