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
#ifndef LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_COVERAGE_DISPLAY_H_  // NOLINT
#define LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_COVERAGE_DISPLAY_H_  // NOLINT

// Required for Qt
#ifndef Q_MOC_RUN
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <rviz/display.h>
#endif

#include <rosbag/view.h>
#include <Eigen/Geometry>
#include <vector>

// Forward declarations for ogre and rviz
namespace Ogre {
class SceneNode;
}

namespace localization_rviz_plugins {

class LocalizationCoverageDisplay : public rviz::Display {
  Q_OBJECT    // NOLINT
    public :  // NOLINT
              LocalizationCoverageDisplay();
  ~LocalizationCoverageDisplay() = default;

 protected:
  void onInitialize() final;
  void reset() final;

 private:
  void drawMap();

  // TODO(rosussan): Remove publishing and use point_cloud_common from rviz/default_plugins
  // when linking error is fixed
  std::vector<Eigen::Vector3f> positions_;
  ros::Publisher cloud_publisher_;
  ros::NodeHandle nh_;
};
}  // namespace localization_rviz_plugins
#endif  // LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_COVERAGE_DISPLAY_H_ NOLINT
