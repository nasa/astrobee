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

#include <ff_util/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <cv_bridge/cv_bridge.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include "depth_odometry_display.h"  // NOLINT
#include "utilities.h"               // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;

DepthOdometryDisplay::DepthOdometryDisplay() {
  //  correspondence_slider_.reset(new rviz::SliderProperty(
  //   "Select Correspondence", 0, "Selecto Correspondence.", this, SLOT(addSmartFactorsProjectionVisual())));

  image_transport::ImageTransport image_transport(nh_);
  const std::string image_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                  static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                  static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_DEPTH_IMAGE);
  image_sub_ = image_transport.subscribe(image_topic, 10, &DepthOdometryDisplay::imageCallback, this);
  correspondence_image_pub_ = image_transport.advertise("/depth_odom/correspondence_image", 1);
}

void DepthOdometryDisplay::onInitialize() { MFDClass::onInitialize(); }

void DepthOdometryDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

void DepthOdometryDisplay::clearDisplay() {}

void DepthOdometryDisplay::imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
  img_buffer_.emplace(lc::TimeFromHeader(image_msg->header), image_msg);
}

sensor_msgs::ImageConstPtr DepthOdometryDisplay::getImage(const localization_common::Time time) {
  const auto img_it = img_buffer_.find(time);
  if (img_it == img_buffer_.end()) return nullptr;
  return img_it->second;
}

void DepthOdometryDisplay::clearImageBuffer(const localization_common::Time oldest_graph_time) {
  const auto img_it = img_buffer_.find(oldest_graph_time);
  if (img_it == img_buffer_.end()) return;
  img_buffer_.erase(img_buffer_.begin(), img_it);
}

void DepthOdometryDisplay::processMessage(const ff_msgs::DepthCorrespondences::ConstPtr& correspondences_msg) {
  clearDisplay();
  const lc::Time previous_time = lc::TimeFromRosTime(correspondences_msg->previous_time);
  const lc::Time latest_time = lc::TimeFromRosTime(correspondences_msg->latest_time);
  const auto previous_image = getImage(previous_time);
  if (previous_image) correspondence_image_pub_.publish(previous_image);
  clearImageBuffer(previous_time);
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::DepthOdometryDisplay, rviz::Display)
