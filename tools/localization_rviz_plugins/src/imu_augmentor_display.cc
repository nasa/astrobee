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
#include <msg_conversions/msg_conversions.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <utility>

#include "imu_augmentor_display.h"  // NOLINT
#include "utilities.h"              // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;
namespace mc = msg_conversions;

ImuAugmentorDisplay::ImuAugmentorDisplay() {
  show_pose_axes_.reset(new rviz::BoolProperty("Show Pose Axes", true, "Show graph poses as axes.", this));
  pose_axes_size_.reset(new rviz::FloatProperty("Pose Axes Size", 0.1, "Pose axes size.", this));
  number_of_poses_.reset(new rviz::IntProperty("Number of Poses", 10, "Number of Poses.", this));
  show_imu_acceleration_arrow_.reset(
    new rviz::BoolProperty("Show Imu Acceleration Arrow", true, "Show imu acceleration arrow.", this));
  imu_acceleration_arrow_scale_.reset(new rviz::FloatProperty("Imu Arrow Scale", 0.1, "Imu arrow scale.", this));
}

void ImuAugmentorDisplay::onInitialize() { MFDClass::onInitialize(); }

void ImuAugmentorDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

void ImuAugmentorDisplay::clearDisplay() {
  imu_augmentor_pose_axes_.clear();
  imu_acceleration_arrow_.reset();
}

void ImuAugmentorDisplay::processMessage(const ff_msgs::EkfState::ConstPtr& msg) {
  const auto world_T_body = lc::PoseFromMsg(msg->pose);
  const auto timestamp = lc::RosTimeFromHeader(msg->header);
  const auto current_frame_T_world = currentFrameTFrame("world", timestamp, *context_);
  if (!current_frame_T_world) {
    LogError("processMessage: Failed to get current_frame_T_world.");
    return;
  }
  const gtsam::Pose3 current_frame_T_body = *current_frame_T_world * world_T_body;

  if (show_pose_axes_->getBool()) {
    imu_augmentor_pose_axes_.set_capacity(number_of_poses_->getInt());
    const float scale = pose_axes_size_->getFloat();
    auto axis = axisFromPose(current_frame_T_body, scale, context_->getSceneManager(), scene_node_);
    axis->setXColor(Ogre::ColourValue(0.5, 0, 0, 0.3));
    axis->setYColor(Ogre::ColourValue(0, 0.5, 0, 0.3));
    axis->setZColor(Ogre::ColourValue(0, 0, 0.5, 0.3));
    imu_augmentor_pose_axes_.push_back(std::move(axis));
  } else {
    imu_augmentor_pose_axes_.clear();
  }

  if (show_imu_acceleration_arrow_->getBool()) {
    imu_acceleration_arrow_.reset(new rviz::Arrow(context_->getSceneManager(), scene_node_));
    imu_acceleration_arrow_->setPosition(ogrePosition(current_frame_T_body));
    const gtsam::Vector3 body_F_acceleration = mc::VectorFromMsg<gtsam::Vector3, geometry_msgs::Vector3>(msg->accel);
    const gtsam::Vector3 current_frame_F_acceleration = current_frame_T_body.rotation() * body_F_acceleration;
    const float scale = imu_acceleration_arrow_scale_->getFloat();
    const auto orientation_and_length =
      getOrientationAndLength(gtsam::Vector3::Zero(), scale * current_frame_F_acceleration);
    const float diameter = scale / 50.0;
    imu_acceleration_arrow_->setOrientation(orientation_and_length.first);
    imu_acceleration_arrow_->set(3.0 * orientation_and_length.second / 4.0, 0.5 * diameter,
                                 orientation_and_length.second / 4.0, diameter);
    imu_acceleration_arrow_->setColor(1, 1, 0, 1);
  } else {
    imu_acceleration_arrow_.reset();
  }
  // TODO(rsoussan): add ang vel visual?
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::ImuAugmentorDisplay, rviz::Display)
