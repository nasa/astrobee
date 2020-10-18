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

#include "localization_graph_display.h"  // NOLINT

#include <imu_integration/utilities.h>
#include <localization_common/combined_nav_state.h>

#include <gtsam/base/serialization.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <glog/logging.h>

namespace localization_rviz_plugins {
namespace ii = imu_integration;
namespace lc = localization_common;

namespace {
Ogre::Vector3 OgrePosition(const gtsam::Pose3& pose) {
  return Ogre::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z());
}  // anonymous ns
}  // namespace

LocalizationGraphDisplay::LocalizationGraphDisplay() {}

void LocalizationGraphDisplay::onInitialize() { MFDClass::onInitialize(); }

void LocalizationGraphDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

void LocalizationGraphDisplay::clearDisplay() {
  graph_pose_axes_.clear();
  imu_factor_lines_.clear();
}

void LocalizationGraphDisplay::addImuVisual(const graph_localizer::GraphLocalizer& graph_localizer,
                                            const gtsam::CombinedImuFactor* const imu_factor) {
  const auto pose = graph_localizer.graph_values().at<gtsam::Pose3>(imu_factor->key1());
  if (!pose) {
    LOG(ERROR) << "ProcessMessage: Failed to get pose.";
    return;
  }
  addPose(*pose);

  const auto velocity = graph_localizer.graph_values().at<gtsam::Velocity3>(imu_factor->key2());
  if (!velocity) {
    LOG(ERROR) << "ProcessMessage: Failed to get velocity.";
    return;
  }

  // TODO(rsoussan): is this correct bias to use???
  const auto bias = graph_localizer.graph_values().at<gtsam::imuBias::ConstantBias>(imu_factor->key5());
  if (!bias) {
    LOG(ERROR) << "ProcessMessage: Failed to get bias.";
    return;
  }

  const lc::CombinedNavState combined_nav_state(*pose, *velocity, *bias, 0 /*Dummy Timestamp*/);
  const auto& pim = imu_factor->preintegratedMeasurements();
  const auto imu_predicted_combined_nav_state = ii::PimPredict(combined_nav_state, pim);
  auto imu_factor_line = std::unique_ptr<rviz::Line>(new rviz::Line(context_->getSceneManager(), scene_node_));
  imu_factor_line->setPoints(OgrePosition(*pose), OgrePosition(imu_predicted_combined_nav_state.pose()));
  imu_factor_lines_.emplace_back(std::move(imu_factor_line));
}

void LocalizationGraphDisplay::addPose(const gtsam::Pose3& pose) {
  auto graph_pose_axis = std::unique_ptr<rviz::Axes>(new rviz::Axes(context_->getSceneManager(), scene_node_));
  graph_pose_axis->setPosition(OgrePosition(pose));
  // TODO(rsoussan): set orientation!!!!
  graph_pose_axes_.emplace_back(std::move(graph_pose_axis));
}

void LocalizationGraphDisplay::processMessage(const ff_msgs::LocalizationGraph::ConstPtr& msg) {
  // TODO(rsoussan): put these somewhere else!
  using Calibration = gtsam::Cal3_S2;
  using Camera = gtsam::PinholeCamera<Calibration>;
  using SmartFactor = gtsam::SmartProjectionPoseFactor<Calibration>;

  // TODO(rsoussan): cleaner way to do this, serialize/deserialize properly
  clearDisplay();
  graph_localizer::GraphLocalizerParams params;
  graph_localizer::GraphLocalizer graph_localizer(params);
  gtsam::deserializeBinary(msg->serialized_graph, graph_localizer);
  for (const auto factor : graph_localizer.factor_graph()) {
    const auto smart_factor = dynamic_cast<const SmartFactor*>(factor.get());
    if (smart_factor) {
      std::cout << "smart factor!" << std::endl;
    }
    const auto imu_factor = dynamic_cast<gtsam::CombinedImuFactor*>(factor.get());
    if (imu_factor) {
      addImuVisual(graph_localizer, imu_factor);
    }
  }
}

}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::LocalizationGraphDisplay, rviz::Display)
