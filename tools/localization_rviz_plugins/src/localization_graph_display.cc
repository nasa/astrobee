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

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>

#include <glog/logging.h>

namespace localization_rviz_plugins {

LocalizationGraphDisplay::LocalizationGraphDisplay() {
  color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204), "Color to draw the acceleration arrows.",
                                            this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty("Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
                                            SLOT(updateColorAndAlpha()));

  history_length_property_ = new rviz::IntProperty("History Length", 1, "Number of prior measurements to display.",
                                                   this, SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

void LocalizationGraphDisplay::onInitialize() {
  MFDClass::onInitialize();
  updateHistoryLength();
}

void LocalizationGraphDisplay::reset() {
  MFDClass::reset();
  // visuals_.clear();
}

void LocalizationGraphDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  // for (size_t i = 0; i < visuals_.size(); i++) {
  // visuals_[ i ]->setColor( color.r, color.g, color.b, alpha );
  //}
}

void LocalizationGraphDisplay::updateHistoryLength() {
  // visuals_.rset_capacity(history_length_property_->getInt());
}

void LocalizationGraphDisplay::processMessage(const ff_msgs::LocalizationGraph::ConstPtr& msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation)) {
    // TODO(rsoussan): remove q printable???
    LOG(ERROR) << "Error transforming from frame " << msg->header.frame_id << " to frame " << qPrintable(fixed_frame_);
    return;
  }

  /*// We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<ImuVisual> visual;
  if( visuals_.full() )
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new ImuVisual( context_->getSceneManager(), scene_node_ ));
  }

  // Now set or update the contents of the chosen visual.
  visual->setMessage( msg );
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor( color.r, color.g, color.b, alpha );

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);*/
}

}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::LocalizationGraphDisplay, rviz::Display)
