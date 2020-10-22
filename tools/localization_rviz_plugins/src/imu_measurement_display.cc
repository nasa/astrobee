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

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <glog/logging.h>

#include <utility>

#include "imu_measurement_display.h"  // NOLINT
#include "utilities.h"                // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;

ImuMeasurementDisplay::ImuMeasurementDisplay() {
  imu_acceleration_arrow_scale_.reset(new rviz::FloatProperty("Imu Arrow Scale", 0.1, "Imu arrow scale.", this));
}

void ImuMeasurementDisplay::onInitialize() { MFDClass::onInitialize(); }

void ImuMeasurementDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

void ImuMeasurementDisplay::clearDisplay() { imu_acceleration_arrow_.reset(); }

void ImuMeasurementDisplay::processMessage(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  imu_acceleration_arrow_.reset(new rviz::Arrow(context_->getSceneManager(), scene_node_));
  // TODO(rsoussan): use tf to get world_T_imu
  imu_acceleration_arrow_->setPosition(ogrePosition(gtsam::Pose3::identity()));
  const gtsam::Vector3 imu_F_acceleration =
      lc::VectorFromMsg<gtsam::Vector3, geometry_msgs::Vector3>(imu_msg->linear_acceleration);
  const float scale = imu_acceleration_arrow_scale_->getFloat();
  const auto orientation_and_length = getOrientationAndLength(gtsam::Vector3::Zero(), scale * imu_F_acceleration);
  const float diameter = scale / 50.0;
  imu_acceleration_arrow_->setOrientation(orientation_and_length.first);
  imu_acceleration_arrow_->set(3.0 * orientation_and_length.second / 4.0, 0.5 * diameter,
                               orientation_and_length.second / 4.0, diameter);
  imu_acceleration_arrow_->setColor(1, 1, 0, 1);
}
// TODO(rsoussan): add ang vel visual?
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::ImuMeasurementDisplay, rviz::Display)
