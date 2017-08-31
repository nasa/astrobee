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

#ifndef MOBILITY_PLANNER_QP_TRAJ_OPT_ROS_SRC_TRAJECTORY_DISPLAY_H_
#define MOBILITY_PLANNER_QP_TRAJ_OPT_ROS_SRC_TRAJECTORY_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <traj_opt_msgs/Trajectory.h>
#include <boost/circular_buffer.hpp>
#endif

namespace Ogre {
class SceneNode;
}

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class BoolProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace traj_opt {

class TrajectoryVisual;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// TrajectoryDisplay will show a 3D arrow showing the direction and magnitude
// of the TRAJECTORY acceleration vector.  The base of the arrow will be at
// the frame listed in the header of the Trajectory message, and the
// direction of the arrow will be relative to the orientation of that
// frame.  It will also optionally show a history of recent
// acceleration vectors, which will be stored in a circular buffer.
//
// The TrajectoryDisplay class itself just implements the circular buffer,
// editable parameters, and Display subclass machinery.  The visuals
// themselves are represented by a separate class, TrajectoryVisual.  The
// idiom for the visuals is that when the objects exist, they appear
// in the scene, and when they are deleted, they disappear.
class TrajectoryDisplay
    : public rviz::MessageFilterDisplay<traj_opt_msgs::Trajectory> {
  Q_OBJECT  // NOLINT
      public
      :  // NOLINT
         // Constructor.  pluginlib::ClassLoader creates instances by calling
         // the default constructor, so make sure you have one.
         TrajectoryDisplay();
  virtual ~TrajectoryDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
 protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties.
 private Q_SLOTS:  // NOLINT
  void updateColorAndAlpha();
  void randomizeColor();
  void updateHistoryLength();
  void updateScale();
  void updateStyle();
  void updateSampleLength();
  // Function to handle an incoming ROS message.
 private:
  void processMessage(const traj_opt_msgs::Trajectory::ConstPtr& msg);

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<TrajectoryVisual> > visuals_;

  // User-editable property variables.
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::ColorProperty* color_property_v_;
  rviz::ColorProperty* color_property_a_;

  rviz::BoolProperty* use_v_property_;
  rviz::BoolProperty* use_a_property_;

  rviz::FloatProperty* thickness_property_;
  rviz::IntProperty* history_length_property_;
  rviz::IntProperty* traj_samples_property_;
  rviz::IntProperty* tangent_samples_property_;
  rviz::EnumProperty* style_property_;
};
// END_TUTORIAL

}  // end namespace traj_opt

#endif  // MOBILITY_PLANNER_QP_TRAJ_OPT_ROS_SRC_TRAJECTORY_DISPLAY_H_
// %EndTag(FULL_SOURCE)%
