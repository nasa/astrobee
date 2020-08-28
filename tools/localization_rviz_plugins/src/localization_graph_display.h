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

#ifndef LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_GRAPH_DISPLAY_H_ // NOLINT
#define LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_GRAPH_DISPLAY_H_ // NOLINT

#ifndef Q_MOC_RUN
#include <ff_msgs/LocalizationGraph.h>
#include <rviz/message_filter_display.h>
#include <boost/circular_buffer.hpp>
#endif

// Forward declarations for ogre and rviz
namespace Ogre {
class SceneNode;
}

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
}  // namespace rviz

namespace localization_rviz_plugins {

class LocalizationGraphDisplay : public rviz::MessageFilterDisplay<ff_msgs::LocalizationGraph> {
  Q_OBJECT
 public:
  LocalizationGraphDisplay();
  virtual ~LocalizationGraphDisplay();  // {}//= default;

  // private:
 protected:
  virtual void onInitialize();
  virtual void reset();

 private Q_SLOTS: // NOLINT
  void updateColorAndAlpha();
  void updateHistoryLength();

 private:
  void processMessage(const ff_msgs::LocalizationGraph::ConstPtr& graph_msg);
  // boost::circular_buffer<boost::shared_ptr<ImuVisual> > visuals_;

  // User-editable property variables.
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::IntProperty* history_length_property_;
};
}  // namespace localization_rviz_plugins
#endif  // LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_GRAPH_DISPLAY_H_ NOLINT
