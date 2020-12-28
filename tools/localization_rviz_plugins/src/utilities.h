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

#include <graph_localizer/graph_localizer.h>
#include <imu_integration/utilities.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/time.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/axes.h>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <memory>
#include <utility>
#include <string>
#include <vector>

namespace localization_rviz_plugins {
Ogre::Vector3 ogrePosition(const gtsam::Pose3& pose);

Ogre::Vector3 ogrePosition(const gtsam::Point3& point);

Ogre::Quaternion ogreQuaternion(const gtsam::Pose3& pose);

boost::optional<gtsam::Pose3> currentFrameTFrame(const std::string& frame_id, const ros::Time timestamp,
                                                 const rviz::DisplayContext& context);

std::unique_ptr<rviz::Axes> axisFromPose(const gtsam::Pose3& pose, const double scale,
                                         Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node);

boost::optional<localization_common::CombinedNavState> firstCombinedNavState(
  const graph_localizer::GraphLocalizer& graph_localizer, const gtsam::CombinedImuFactor* const imu_factor);

boost::optional<localization_common::CombinedNavState> pimPredict(
  const graph_localizer::GraphLocalizer& graph_localizer, const gtsam::CombinedImuFactor* const imu_factor);

std::pair<Ogre::Quaternion, double> getOrientationAndLength(const gtsam::Point3& point_a, const gtsam::Point3& point_b);
}  // namespace localization_rviz_plugins
#endif  // LOCALIZATION_RVIZ_PLUGINS_UTILITIES_H_  NOLINT
