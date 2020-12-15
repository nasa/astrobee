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
#ifndef LOCALIZATION_RVIZ_PLUGINS_POSE_DISPLAY_H_  // NOLINT
#define LOCALIZATION_RVIZ_PLUGINS_POSE_DISPLAY_H_  // NOLINT

// Required for Qt
#ifndef Q_MOC_RUN
#include <geometry_msgs/PoseStamped.h>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <boost/circular_buffer.hpp>
#endif

#include <vector>

// Forward declarations for ogre and rviz
namespace Ogre {
class SceneNode;
}

namespace localization_rviz_plugins {

class PoseDisplay : public rviz::MessageFilterDisplay<geometry_msgs::PoseStamped> {
  Q_OBJECT    // NOLINT
    public :  // NOLINT
              PoseDisplay();
  ~PoseDisplay() = default;

 protected:
  void onInitialize() final;
  void reset() final;

 private Q_SLOTS:  // NOLINT

 private:
  void processMessage(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void clearDisplay();

  boost::circular_buffer<std::unique_ptr<rviz::Axes>> pose_axes_;
  std::unique_ptr<rviz::FloatProperty> pose_axes_size_;
  std::unique_ptr<rviz::IntProperty> number_of_poses_;
};
}  // namespace localization_rviz_plugins
#endif  // LOCALIZATION_RVIZ_PLUGINS_POSE_DISPLAY_H_ NOLINT
