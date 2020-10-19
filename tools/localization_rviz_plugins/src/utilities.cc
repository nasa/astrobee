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

#include "utilities.h"  // NOLINT

namespace localization_rviz_plugins {
Ogre::Vector3 OgrePosition(const gtsam::Pose3& pose) {
  return Ogre::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z());
}

Ogre::Quaternion OgreQuaternion(const gtsam::Pose3& pose) {
  const auto quaternion = pose.rotation().toQuaternion();
  return Ogre::Quaternion(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
}

void addPoseAsAxis(const gtsam::Pose3& pose, const double scale, std::vector<std::unique_ptr<rviz::Axes>>& axes,
                   Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node) {
  auto axis = std::unique_ptr<rviz::Axes>(new rviz::Axes(scene_manager, scene_node));
  axis->setPosition(OgrePosition(pose));
  axis->setOrientation(OgreQuaternion(pose));
  axis->setScale(Ogre::Vector3(scale, scale, scale));
  axes.emplace_back(std::move(axis));
}
}  // namespace localization_rviz_plugins
