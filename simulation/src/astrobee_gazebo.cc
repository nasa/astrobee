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

#include <astrobee_gazebo/astrobee_gazebo.h>

// Transformation helper code
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace gazebo {

// Memory allocation for static buffer
tf2_ros::Buffer FreeFlyerPlugin::buffer_;

// Constructor
FreeFlyerPlugin::FreeFlyerPlugin(std::string const& plugin_name,
  std::string const& plugin_frame, bool send_heartbeats) :
    ff_util::FreeFlyerNodelet(plugin_name, send_heartbeats),
      robot_name_("/"), plugin_name_(plugin_name),
        plugin_frame_(plugin_frame), parent_frame_() {}

// Destructor
FreeFlyerPlugin::~FreeFlyerPlugin() {}

// Some plugins might want the world as the parent frame
void FreeFlyerPlugin::SetParentFrame(std::string const& parent) {
  parent_frame_ = parent;
}

// Load function
void FreeFlyerPlugin::InitializePlugin(std::string const& robot_name) {
  robot_name_ = robot_name;
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
  ROS_DEBUG_STREAM("Loading " << plugin_name_ << " on robot " << robot_name_);

  // Intiialize the transform listener
  static tf2_ros::TransformListener listener(buffer_);

  // Get nodehandle based on the model name.
  nh_ = ros::NodeHandle(robot_name_);
  nh_mt_ = ros::NodeHandle(robot_name_);

  // Init freeflyer nodelet to start heartbeat and faults, using the
  // nodehandles that have the correct namespace
  Setup(nh_, nh_mt_);

  // If we have a frame then defer chainloading until we receive them
  timer_ = nh_.createTimer(ros::Duration(1.0),
    &FreeFlyerSensorPlugin::SetupExtrinsics, this);
}

// Poll for extrinsics until found
void FreeFlyerPlugin::SetupExtrinsics(const ros::TimerEvent& event) {
  // If we don't need extrinsics, then don't bother looking...
  if (plugin_frame_.empty()) {
    if (ExtrinsicsCallback(nullptr))
      timer_.stop();
    return;
  }
  // Get the parent and child frame
  if (parent_frame_.empty())
    parent_frame_ = GetFrame(FRAME_NAME_BODY);
  // Keep trying to find the frame transform
  try {
    geometry_msgs::TransformStamped tf =
      buffer_.lookupTransform(parent_frame_, GetFrame(), ros::Time(0));
    if (ExtrinsicsCallback(&tf)) {
      OnExtrinsicsReceived(&nh_);
      timer_.stop();
    }
  } catch (tf2::TransformException &ex) {}
}

// Get the extrinsics frame
std::string FreeFlyerPlugin::GetFrame(std::string target, std::string delim) {
  std::string frame = (target.empty() ? plugin_frame_ : target);
  return (robot_name_ == "/" ? frame : robot_name_ + delim + frame);
}

// Model plugin

// Constructor
FreeFlyerModelPlugin::FreeFlyerModelPlugin(std::string const& plugin_name,
  std::string const& plugin_frame, bool send_heartbeats) :
    FreeFlyerPlugin::FreeFlyerPlugin(
      plugin_name, plugin_frame, send_heartbeats) {}

// Destructor
FreeFlyerModelPlugin::~FreeFlyerModelPlugin() {}

// Auto-called when Gazebo loads the plugin
void FreeFlyerModelPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  sdf_   = sdf;
  link_  = model->GetLink();
  world_ = model->GetWorld();
  model_ = model;
  // Initialize the FreeFlyerPlugin
  InitializePlugin(model_->GetName());
  // Now load the rest of the plugin
  LoadCallback(GetPlatformHandle(), model_, sdf_);
}

// Get the model link
physics::LinkPtr FreeFlyerModelPlugin::GetLink() {
  return link_;
}

// Get the model world
physics::WorldPtr FreeFlyerModelPlugin::GetWorld() {
  return world_;
}

// Get the model
physics::ModelPtr FreeFlyerModelPlugin::GetModel() {
  return model_;
}

// Manage the extrinsics based on the sensor type
bool FreeFlyerModelPlugin::ExtrinsicsCallback(
  geometry_msgs::TransformStamped const* tf) {
  // A tf nullptr means no transform is required
  if (tf) {
    // Handle the transform for all sensor types
    ignition::math::Pose3d pose(
      tf->transform.translation.x,
      tf->transform.translation.y,
      tf->transform.translation.z,
      tf->transform.rotation.w,
      tf->transform.rotation.x,
      tf->transform.rotation.y,
      tf->transform.rotation.z);
    // Set the model pose
    model_->SetWorldPose(pose);
  }
  // Success
  return true;
}

// Sensor plugin

// Constructor
FreeFlyerSensorPlugin::FreeFlyerSensorPlugin(std::string const& plugin_name,
  std::string const& plugin_frame, bool send_heartbeats) :
    FreeFlyerPlugin::FreeFlyerPlugin(
      plugin_name, plugin_frame, send_heartbeats) {}

// Destructor
FreeFlyerSensorPlugin::~FreeFlyerSensorPlugin() {}

// Sensor plugin load callback
void FreeFlyerSensorPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
  sensor_ = sensor;
  sdf_ = sdf;
  world_ = gazebo::physics::get_world(sensor->WorldName());
  model_ = boost::static_pointer_cast<physics::Link>(
    world_->GetEntity(sensor->ParentName()))->GetModel();
  // Initialize the FreeFlyerPlugin
  InitializePlugin(model_->GetName());
  // Now load the rest of the plugin
  LoadCallback(GetPlatformHandle(), sensor_, sdf_);
}

// Get the sensor world
physics::WorldPtr FreeFlyerSensorPlugin::GetWorld() {
  return world_;
}

// Get the sensor model
physics::ModelPtr FreeFlyerSensorPlugin::GetModel() {
  return model_;
}

// Manage the extrinsics
bool FreeFlyerSensorPlugin::ExtrinsicsCallback(
  geometry_msgs::TransformStamped const* tf) {
  // A tf nullptr means no transform is required
  if (tf) {
    // Handle the transform for all sensor types
    ignition::math::Pose3d pose(
      tf->transform.translation.x,
      tf->transform.translation.y,
      tf->transform.translation.z,
      tf->transform.rotation.w,
      tf->transform.rotation.x,
      tf->transform.rotation.y,
      tf->transform.rotation.z);

    // Set the sensor pose
    if (sensor_)
      sensor_->SetPose(pose);
    else
      return false;

    // Convert to a world pose
    Eigen::Quaterniond rot_90_x(0.70710678, 0.70710678, 0, 0);
    Eigen::Quaterniond rot_90_z(0.70710678, 0, 0, 0.70710678);
    Eigen::Quaterniond pose_temp(
      tf->transform.rotation.w,
      tf->transform.rotation.x,
      tf->transform.rotation.y,
      tf->transform.rotation.z);
    pose_temp = pose_temp * rot_90_x;
    pose_temp = pose_temp * rot_90_z;
    pose = ignition::math::Pose3d(
      tf->transform.translation.x,
      tf->transform.translation.y,
      tf->transform.translation.z,
      pose_temp.w(), pose_temp.x(), pose_temp.y(), pose_temp.z());
    math::Pose tf_bs = pose;
    math::Pose tf_wb = model_->GetWorldPose();
    math::Pose tf_ws = tf_bs + tf_wb;
    ignition::math::Pose3d world_pose(tf_ws.pos.x, tf_ws.pos.y,
      tf_ws.pos.z, tf_ws.rot.w, tf_ws.rot.x, tf_ws.rot.y, tf_ws.rot.z);

    // In the case of a camera update the camera world pose
    if (sensor_->Type() == "camera") {
      sensors::CameraSensorPtr sensor
        = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor_);
      if (sensor && sensor->Camera())
        sensor->Camera()->SetWorldPose(world_pose);
      else
        return false;
    }

    // In the case of a wide angle camera update the camera world pose
    if (sensor_->Type() == "wideanglecamera") {
      // TODO(oalexan1): Figure out why the line below fails on spheresgoat
      sensors::WideAngleCameraSensorPtr sensor =
        std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor_);
      if (sensor && sensor->Camera())
        sensor->Camera()->SetWorldPose(world_pose);
      else
        return false;
    }

    // In the case of a depth camera update the depth camera pose
    if (sensor_->Type() == "depth") {
      sensors::DepthCameraSensorPtr sensor =
        std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor_);
      if (sensor && sensor->DepthCamera())
        sensor->DepthCamera()->SetWorldPose(world_pose);
      else
        return false;
    }
  }
  // Success
  return true;
}

}  // namespace gazebo
