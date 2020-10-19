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

#ifndef LOCALIZATION_RVIZ_PLUGINS_UTILITIES_H_  // NOLINT
#define LOCALIZATION_RVIZ_PLUGINS_UTILITIES_H_  // NOLINT

#include <gtsam/geometry/Pose3.h>

#include <rviz/ogre_helpers/axes.h>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <memory>
#include <vector>

namespace localization_rviz_plugins {
Ogre::Vector3 OgrePosition(const gtsam::Pose3& pose);

Ogre::Quaternion OgreQuaternion(const gtsam::Pose3& pose);

void addPoseAsAxis(const gtsam::Pose3& pose, const double scale, std::vector<std::unique_ptr<rviz::Axes>>& axes,
                   Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node);
}  // namespace localization_rviz_plugins
#endif  // LOCALIZATION_RVIZ_PLUGINS_UTILITIES_H_  NOLINT
