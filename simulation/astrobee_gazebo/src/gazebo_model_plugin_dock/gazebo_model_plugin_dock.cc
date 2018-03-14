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

// ROS includes
#include <ros/ros.h>

// Astrobee simulation API
#include <astrobee_gazebo/astrobee_gazebo.h>

// STL includes
#include <string>
#include <thread>

namespace gazebo {

// Provides a simple dock plugin
class GazeboModelPluginDock : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginDock() : FreeFlyerModelPlugin("gazebo_dock") {}

  // Destructor
  virtual ~GazeboModelPluginDock() {}

 protected:
  // Called when the plugin is loaded into the simulator
  virtual void LoadCallback(ros::NodeHandle *nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) {
    // Defer the extrinsics setup to allow plugins to load
    timer_ = nh->createTimer(ros::Duration(1.0),
      &GazeboModelPluginDock::ExtrinsicsCallback, this);
  }

  // Manage the extrinsics based on the sensor type
  void ExtrinsicsCallback(ros::TimerEvent const& event) {
    // Create a buffer and listener for TF2 transforms
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener listener(buffer);
    // Get extrinsics from framestore
    try {
      // Lookup the transform for this sensor
      geometry_msgs::TransformStamped tf = buffer.lookupTransform(
        "world", "dock/body", ros::Time(0));
      // Handle the transform for all sensor types
      ignition::math::Pose3d pose(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z);
      // Set the pose
      GetModel()->SetWorldPose(pose);
    } catch (tf2::TransformException &ex) {}
  }

 private:
  physics::ModelPtr model_;
  ros::Timer timer_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginDock)

}   // namespace gazebo
