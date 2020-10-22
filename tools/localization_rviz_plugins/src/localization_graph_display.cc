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

#include <localization_common/combined_nav_state.h>

#include <gtsam/base/serialization.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <glog/logging.h>

#include <utility>

#include "localization_graph_display.h"  // NOLINT
#include "utilities.h"                   // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;

LocalizationGraphDisplay::LocalizationGraphDisplay() {
  show_pose_axes_.reset(new rviz::BoolProperty("Show Pose Axes", true, "Show graph poses as axes.", this));
  pose_axes_size_.reset(new rviz::FloatProperty("Pose Axes Size", 0.1, "Pose axes size.", this));
  show_imu_factor_arrows_.reset(
      new rviz::BoolProperty("Show Imu Factor Arrows", true, "Show imu factors as arrows.", this));
  imu_factor_arrows_diameter_.reset(
      new rviz::FloatProperty("Imu Factor Arrows Diameter", 0.01, "Imu factor arrows diameter.", this));
}

void LocalizationGraphDisplay::onInitialize() { MFDClass::onInitialize(); }

void LocalizationGraphDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

void LocalizationGraphDisplay::clearDisplay() {
  graph_pose_axes_.clear();
  imu_factor_arrows_.clear();
}

void LocalizationGraphDisplay::addImuVisual(const graph_localizer::GraphLocalizer& graph_localizer,
                                            const gtsam::CombinedImuFactor* const imu_factor) {
  const auto pose = graph_localizer.graph_values().at<gtsam::Pose3>(imu_factor->key1());
  if (!pose) {
    LOG(ERROR) << "ProcessMessage: Failed to get pose.";
    return;
  }
  if (show_pose_axes_->getBool()) {
    const float scale = pose_axes_size_->getFloat();
    auto axis = axisFromPose(*pose, scale, context_->getSceneManager(), scene_node_);
    graph_pose_axes_.emplace_back(std::move(axis));
  }

  if (show_imu_factor_arrows_->getBool()) {
    const auto imu_predicted_combined_nav_state = pimPredict(graph_localizer, imu_factor);
    if (!imu_predicted_combined_nav_state) {
      LOG(ERROR) << "AddImuVisual: Failed to get pim predicted nav state.";
      return;
    }
    auto imu_factor_arrow = std::unique_ptr<rviz::Arrow>(new rviz::Arrow(context_->getSceneManager(), scene_node_));
    imu_factor_arrow->setPosition(ogrePosition(*pose));
    const auto orientation_and_length =
        getOrientationAndLength(pose->translation(), imu_predicted_combined_nav_state->pose().translation());
    imu_factor_arrow->setOrientation(orientation_and_length.first);
    const float diameter = imu_factor_arrows_diameter_->getFloat();
    imu_factor_arrow->set(3.0 * orientation_and_length.second / 4.0, 0.5 * diameter,
                          orientation_and_length.second / 4.0, diameter);
    imu_factor_arrow->setColor(1, 1, 0, 1);
    imu_factor_arrows_.emplace_back(std::move(imu_factor_arrow));
  }
}

void LocalizationGraphDisplay::processMessage(const ff_msgs::LocalizationGraph::ConstPtr& msg) {
  // TODO(rsoussan): put these somewhere else!
  using Calibration = gtsam::Cal3_S2;
  using Camera = gtsam::PinholeCamera<Calibration>;
  using SmartFactor = gtsam::SmartProjectionPoseFactor<Calibration>;

  clearDisplay();
  graph_localizer::GraphLocalizer graph_localizer;
  gtsam::deserializeBinary(msg->serialized_graph, graph_localizer);
  for (const auto factor : graph_localizer.factor_graph()) {
    const auto smart_factor = dynamic_cast<const SmartFactor*>(factor.get());
    if (smart_factor) {
    }
    const auto imu_factor = dynamic_cast<gtsam::CombinedImuFactor*>(factor.get());
    if (imu_factor) {
      addImuVisual(graph_localizer, imu_factor);
    }
  }
}

}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::LocalizationGraphDisplay, rviz::Display)
