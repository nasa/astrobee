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

// TF2 eigen bindings
#include <tf2_eigen/tf2_eigen.h>

namespace gazebo {

// Model plugin

FreeFlyerModelPlugin::FreeFlyerModelPlugin(std::string const& name,
  std::string const& frame, bool hb) :
    ff_util::FreeFlyerNodelet(name, hb), name_(name), frame_(frame) {}

FreeFlyerModelPlugin::~FreeFlyerModelPlugin() {}

void FreeFlyerModelPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  gzmsg << "[FF] Loading plugin for model with name " << name_ << "\n";
  // Get usefule properties
  sdf_   = sdf;
  link_  = model->GetLink();
  world_ = model->GetWorld();
  model_ = model;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");

  // Get a nodehandle based on the model name and use a different default queue
  // We have to do this to avoid gazebo main thread blocking the ROS queue.
  nh_ = ros::NodeHandle(model_->GetName());

  // Call initialize on the freeflyer nodelet to start heartbeat and faults
  Setup(nh_, nh_);

  // Pass the new callback queue
  LoadCallback(&nh_, model_, sdf_);

  // Try and set the extrinsics after the world has loaded.
  if (!frame_.empty())
    timer_ = nh_.createTimer(ros::Duration(1.0),
      &FreeFlyerModelPlugin::SetupExtrinsics, this);
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

// Get the extrinsics frame
std::string FreeFlyerModelPlugin::GetFrame(std::string target) {
  std::string frame = (target.empty() ? frame_ : target);
  if (GetModel()->GetName() == "/")
    return frame;
  return GetModel()->GetName() + "/" + frame;
}

// Manage the extrinsics based on the sensor type
void FreeFlyerModelPlugin::SetupExtrinsics(const ros::TimerEvent& event) {
  // Create a buffer and listener for TF2 transforms
  static tf2_ros::Buffer buffer;
  static tf2_ros::TransformListener listener(buffer);
  // Get extrinsics from framestore
  try {
    // Lookup the transform for this sensor
    geometry_msgs::TransformStamped tf = buffer.lookupTransform(
      FRAME_NAME_WORLD, GetFrame(), ros::Time(0));
    // Handle the transform for all sensor types
    ignition::math::Pose3d pose(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z,
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);
    // Set the sensor pose
    model_->SetWorldPose(pose);
  } catch (tf2::TransformException &ex) {}
}

// Sensor plugin

FreeFlyerSensorPlugin::FreeFlyerSensorPlugin(
  std::string const& name, std::string const& frame, bool hb) :
    ff_util::FreeFlyerNodelet(name, hb), name_(name), frame_(frame),
      extrinsics_found_(false) {}

FreeFlyerSensorPlugin::~FreeFlyerSensorPlugin() {}

void FreeFlyerSensorPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
  gzmsg << "[FF] Loading plugin for sensor with name " << name_ << "\n";
  // Get the world in which this sensor exists, and the link it is attached to
  sensor_ = sensor;
  sdf_ = sdf;
  world_ = gazebo::physics::get_world(sensor->WorldName());
  model_ = boost::static_pointer_cast < physics::Link > (
    world_->GetEntity(sensor->ParentName()))->GetModel();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");

  // Get a nodehandle based on the model name and use a different default queue
  // We have to do this to avoid gazebo main thread blocking the ROS queue.
  nh_ = ros::NodeHandle(model_->GetName());

  // Call initialize on the freeflyer nodelet to start heartbeat and faults
  Setup(nh_, nh_);

  // Pass the new callback queue
  LoadCallback(&nh_, sensor_, sdf_);

  // Try and set the extrinsics after the world has loaded.
  if (!frame_.empty())
    timer_ = nh_.createTimer(ros::Duration(1.0),
      &FreeFlyerSensorPlugin::SetupExtrinsics, this);
}

// Get the sensor world
physics::WorldPtr FreeFlyerSensorPlugin::GetWorld() {
  return world_;
}

// Get the sensor model
physics::ModelPtr FreeFlyerSensorPlugin::GetModel() {
  return model_;
}

// Get the extrinsics frame
std::string FreeFlyerSensorPlugin::GetFrame(std::string target) {
  std::string frame = (target.empty() ? frame_ : target);
  if (GetModel()->GetName() == "/")
    return frame;
  return GetModel()->GetName() + "/" + frame;
}

// Get the type of the sensor
std::string FreeFlyerSensorPlugin::GetRotationType() {
  return rotation_type_;
}

// Were extrinsics found
bool FreeFlyerSensorPlugin::ExtrinsicsFound() {
  return extrinsics_found_;
}

// Manage the extrinsics based on the sensor type
void FreeFlyerSensorPlugin::SetupExtrinsics(const ros::TimerEvent& event) {
  // Create a buffer and listener for TF2 transforms
  static tf2_ros::Buffer buffer;
  static tf2_ros::TransformListener listener(buffer);
  // Get extrinsics from framestore
  try {
    // Lookup the transform for this sensor
    geometry_msgs::TransformStamped tf = buffer.lookupTransform(
      GetFrame("body"), GetFrame(), ros::Time(0));

    // Handle the transform for all sensor types
    ignition::math::Pose3d pose(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z,
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);

    // Set the sensor pose
    if (sensor_)
      sensor_->SetPose(pose);
    else
      return;

    // Convert to a world pose
    Eigen::Quaterniond rot_90_x(0.70710678, 0.70710678, 0, 0);
    Eigen::Quaterniond rot_90_z(0.70710678, 0, 0, 0.70710678);
    Eigen::Quaterniond pose_temp(tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);
    pose_temp = pose_temp * rot_90_x;
    pose_temp = pose_temp * rot_90_z;
    pose = ignition::math::Pose3d(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
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
        return;
    }

    // In the case of a wide angle camera update the camera world pose
    if (sensor_->Type() == "wideanglecamera") {
      // TODO(oalexan1): Figure out why the line below fails on spheresgoat
      sensors::WideAngleCameraSensorPtr sensor =
        std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor_);
      if (sensor && sensor->Camera())
        sensor->Camera()->SetWorldPose(world_pose);
      else
        return;
    }

    // In the case of a depth camera update the depth camera pose
    if (sensor_->Type() == "depth") {
      sensors::DepthCameraSensorPtr sensor =
        std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor_);
      if (sensor && sensor->DepthCamera())
        sensor->DepthCamera()->SetWorldPose(world_pose);
      else
        return;
    }
    // Mark that the extrinsics were found
    extrinsics_found_ = true;
  } catch (tf2::TransformException &ex) {}
}



}  // namespace gazebo
