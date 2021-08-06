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

// Header file must go in src directory for Qt/Rviz plugin
#ifndef LOCALIZATION_RVIZ_PLUGINS_DEPTH_ODOMETRY_DISPLAY_H_  // NOLINT
#define LOCALIZATION_RVIZ_PLUGINS_DEPTH_ODOMETRY_DISPLAY_H_  // NOLINT

// Required for Qt
#ifndef Q_MOC_RUN
#include <ff_msgs/DepthCorrespondences.h>
#include <localization_common/time.h>
#include <image_transport/image_transport.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <rviz/message_filter_display.h>
#include "slider_property.h"  // NOLINT
#endif

// Forward declarations for ogre and rviz
namespace Ogre {
class SceneNode;
}

namespace localization_rviz_plugins {
class DepthOdometryDisplay : public rviz::MessageFilterDisplay<ff_msgs::DepthCorrespondences> {
  Q_OBJECT    // NOLINT
    public :  // NOLINT
              DepthOdometryDisplay();
  ~DepthOdometryDisplay() = default;

  // private:
 protected:
  void onInitialize() final;
  void reset() final;

 private Q_SLOTS:  // NOLINT
                   // void addSmartFactorsProjectionVisual();

 private:
  void processMessage(const ff_msgs::DepthCorrespondences::ConstPtr& correspondences_msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);
  void clearImageBuffer(const localization_common::Time oldest_allowed_time);
  void clearDisplay();
  void createCorrespondencesImage();
  sensor_msgs::ImageConstPtr getImage(const localization_common::Time time);

  std::unique_ptr<rviz::SliderProperty> correspondence_index_slider_;
  ff_msgs::DepthCorrespondences::ConstPtr latest_correspondences_msg_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher correspondence_image_pub_;
  ros::NodeHandle nh_;
  // TODO(rsoussan): Create seperate class for image buffer, unify with loc graph display
  std::map<localization_common::Time, sensor_msgs::ImageConstPtr> img_buffer_;
};
}  // namespace localization_rviz_plugins
#endif  // LOCALIZATION_RVIZ_PLUGINS_DEPTH_ODOMETRY_DISPLAY_H_ NOLINT
