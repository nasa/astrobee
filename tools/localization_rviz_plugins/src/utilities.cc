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

#include <rviz/frame_manager.h>

#include "utilities.h"  // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;
namespace ii = imu_integration;

Ogre::Vector3 ogrePosition(const gtsam::Pose3& pose) {
  return Ogre::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z());
}

Ogre::Quaternion ogreQuaternion(const gtsam::Pose3& pose) {
  const auto quaternion = pose.rotation().toQuaternion();
  return Ogre::Quaternion(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
}

boost::optional<gtsam::Pose3> currentFrameTFrame(const std::string& frame_id, const ros::Time timestamp,
                                                 const rviz::DisplayContext& context) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context.getFrameManager()->getTransform(frame_id, timestamp, position, orientation)) {
    LogError("currentFrameTFrame: Failed to get transform for " << frame_id);
    return boost::none;
  }
  return gtsam::Pose3(gtsam::Rot3(orientation.w, orientation.x, orientation.y, orientation.z),
                      gtsam::Point3(position.x, position.y, position.z));
}

std::unique_ptr<rviz::Axes> axisFromPose(const gtsam::Pose3& pose, const double scale,
                                         Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node) {
  auto axis = std::unique_ptr<rviz::Axes>(new rviz::Axes(scene_manager, scene_node));
  axis->setPosition(ogrePosition(pose));
  axis->setOrientation(ogreQuaternion(pose));
  axis->setScale(Ogre::Vector3(scale, scale, scale));
  return axis;
}

boost::optional<lc::CombinedNavState> firstCombinedNavState(const graph_localizer::GraphLocalizer& graph_localizer,
                                                            const gtsam::CombinedImuFactor* const imu_factor) {
  const auto pose = graph_localizer.graph_values().at<gtsam::Pose3>(imu_factor->key1());
  if (!pose) {
    LogError("pimPredict: Failed to get pose.");
    return boost::none;
  }

  const auto velocity = graph_localizer.graph_values().at<gtsam::Velocity3>(imu_factor->key2());
  if (!velocity) {
    LogError("pimPredict: Failed to get velocity.");
    return boost::none;
  }

  // TODO(rsoussan): is this correct bias to use???
  const auto bias = graph_localizer.graph_values().at<gtsam::imuBias::ConstantBias>(imu_factor->key5());
  if (!bias) {
    LogError("pimPredict: Failed to get bias.");
    return boost::none;
  }

  return lc::CombinedNavState(*pose, *velocity, *bias, 0 /*Dummy Timestamp*/);
}

boost::optional<lc::CombinedNavState> pimPredict(const graph_localizer::GraphLocalizer& graph_localizer,
                                                 const gtsam::CombinedImuFactor* const imu_factor) {
  const auto combined_nav_state(firstCombinedNavState(graph_localizer, imu_factor));
  if (!combined_nav_state) return boost::none;
  const auto& pim = imu_factor->preintegratedMeasurements();
  return ii::PimPredict(*combined_nav_state, pim);
}

std::pair<Ogre::Quaternion, double> getOrientationAndLength(const gtsam::Point3& point_a,
                                                            const gtsam::Point3& point_b) {
  // Ogre identity vector is along negative z axis
  const Eigen::Vector3d negative_z(0, 0, 1);
  const auto normalized_difference_vector = (point_b - point_a).normalized();
  const double norm = (point_b - point_a).norm();
  const auto orientation = Eigen::Quaterniond().setFromTwoVectors(negative_z, normalized_difference_vector);
  return {Ogre::Quaternion(orientation.w(), orientation.x(), orientation.y(), orientation.z()), norm};
}
}  // namespace localization_rviz_plugins
