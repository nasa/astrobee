/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <gazebo/sensors/WideAngleCameraSensor.hh>
#include <astrobee_gazebo/astrobee_gazebo.h>

// Transformation helper code
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace gazebo {

// Constructor
FreeFlyerPlugin::FreeFlyerPlugin(std::string const& plugin_name,
  std::string const& plugin_frame, bool send_heartbeats) :
    ff_util::FreeFlyerNodelet(plugin_name, send_heartbeats),
      robot_name_("/"), plugin_name_(plugin_name),
        plugin_frame_(plugin_frame), parent_frame_() {}

// Destructor
FreeFlyerPlugin::~FreeFlyerPlugin() {
  nh_ff_.shutdown();
  thread_.join();
}

// Some plugins might want the world as the parent frame
void FreeFlyerPlugin::SetParentFrame(std::string const& parent) {
  parent_frame_ = parent;
}

// Load function
void FreeFlyerPlugin::InitializePlugin(std::string const& robot_name, std::string const& plugin_name) {
  robot_name_ = robot_name;
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
  ROS_DEBUG_STREAM("Loading " << plugin_name_ << " on robot " << robot_name_);

  // Get nodehandle based on the model name.
  nh_ = ros::NodeHandle(robot_name_);
  nh_.setCallbackQueue(&callback_queue_);
  thread_ = std::thread(&FreeFlyerPlugin::CallbackThread, this);
  listener_.reset(new tf2_ros::TransformListener(buffer_, nh_));

  // Assign special node handles that use custom callback queues to avoid
  // Gazebo locking up heartbeats from being sent to the system monitor.
  nh_ff_ = ros::NodeHandle(robot_name_);
  Setup(nh_ff_, nh_ff_, plugin_name);

  // If we have a frame then defer chainloading until we receive them
  timer_ = nh_.createTimer(ros::Duration(5.0),
    &FreeFlyerSensorPlugin::SetupExtrinsics, this);
}

// Service the callback thread
void FreeFlyerPlugin::CallbackThread() {
  while (nh_.ok())
    callback_queue_.callAvailable(ros::WallDuration(0.01));
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

  // Read namespace
  std::string ns = model_->GetName();
  if (ns == "bsharp")
    ns = "/";

  // Read plugin custom name if specified
  std::string plugin_name = "";
  if (sdf->HasElement("plugin_name"))
    plugin_name = sdf->Get<std::string>("plugin_name");
  // Read plugin custom frame if specified
  if (sdf->HasElement("plugin_frame"))
    plugin_frame_ = sdf->Get<std::string>("plugin_frame");

  // Initialize the FreeFlyerPlugin
  InitializePlugin(ns, plugin_name);

  // Now load the rest of the plugin
  LoadCallback(&nh_, model_, sdf_);
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
  #if GAZEBO_MAJOR_VERSION > 7
  model_ = boost::static_pointer_cast < physics::Link >(
    world_->EntityByName(sensor->ParentName()))->GetModel();
  #else
  model_ = boost::static_pointer_cast<physics::Link>(
    world_->GetEntity(sensor->ParentName()))->GetModel();
  #endif

  // Read namespace
  std::string ns = model_->GetName();
  if (ns == "bsharp")
    ns = "/";

  // Read plugin custom name if specified
  std::string plugin_name = "";
  if (sdf->HasElement("plugin_name"))
    plugin_name = sdf->Get<std::string>("plugin_name");
  // Read plugin custom frame if specified
  if (sdf->HasElement("plugin_frame"))
    plugin_frame_ = sdf->Get<std::string>("plugin_frame");

  // Initialize the FreeFlyerPlugin
  InitializePlugin(ns, plugin_name);

  // Now load the rest of the plugin
  LoadCallback(&nh_, sensor_, sdf_);
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
    #if GAZEBO_MAJOR_VERSION > 7
    ignition::math::Pose3d tf_bs = pose;
    ignition::math::Pose3d tf_wb = model_->WorldPose();
    ignition::math::Pose3d tf_ws = tf_bs + tf_wb;
    ignition::math::Pose3d world_pose(tf_bs + tf_wb);
    #else
    math::Pose tf_bs = pose;
    math::Pose tf_wb = model_->GetWorldPose();
    math::Pose tf_ws = tf_bs + tf_wb;
    ignition::math::Pose3d world_pose(tf_ws.pos.x, tf_ws.pos.y,
      tf_ws.pos.z, tf_ws.rot.w, tf_ws.rot.x, tf_ws.rot.y, tf_ws.rot.z);
    #endif

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
      std::shared_ptr<sensors::WideAngleCameraSensor> sensor =
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

// Compute the transform from sensor to world coordinates
#if GAZEBO_MAJOR_VERSION > 7
Eigen::Affine3d SensorToWorld(ignition::math::Pose3d const& world_pose,
                              ignition::math::Pose3d const& sensor_pose) {
    Eigen::Affine3d body_to_world
      = (Eigen::Translation3d(world_pose.Pos().X(),
                              world_pose.Pos().Y(),
                              world_pose.Pos().Z()) *
         Eigen::Quaterniond(world_pose.Rot().W(),
                            world_pose.Rot().X(),
                            world_pose.Rot().Y(),
                            world_pose.Rot().Z()));
#else
Eigen::Affine3d SensorToWorld(gazebo::math::Pose const& world_pose,
                              ignition::math::Pose3d const& sensor_pose) {
    Eigen::Affine3d body_to_world
      = (Eigen::Translation3d(world_pose.pos.x,
                              world_pose.pos.y,
                              world_pose.pos.z) *
         Eigen::Quaterniond(world_pose.rot.w,
                            world_pose.rot.x,
                            world_pose.rot.y,
                            world_pose.rot.z));
#endif
    Eigen::Affine3d sensor_to_body
      = (Eigen::Translation3d(sensor_pose.Pos().X(),
                              sensor_pose.Pos().Y(),
                              sensor_pose.Pos().Z()) *
         Eigen::Quaterniond(sensor_pose.Rot().W(),
                            sensor_pose.Rot().X(),
                            sensor_pose.Rot().Y(),
                            sensor_pose.Rot().Z()));
    return body_to_world * sensor_to_body;
}

void FillCameraInfo(rendering::CameraPtr camera, sensor_msgs::CameraInfo & msg) {
    msg.width = camera->ImageWidth();
    msg.height = camera->ImageHeight();

    double hfov = camera->HFOV().Radian();  // horizontal field of view in radians
    double focal_length = camera->ImageWidth()/(2.0 * tan(hfov/2.0));
    double opitcal_center_x = msg.width/2.0;
    double optical_center_y = msg.height/2.0;

    // Intrinsics matrix
    msg.K = {focal_length, 0, opitcal_center_x,
             0, focal_length, optical_center_y,
             0, 0, 1};

    // Projection matrix. We won't use this, but initalize it to something.
    msg.P = {1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0};

    // Rotation matrix. We won't use it.
    msg.R = {1, 0, 0,
             0, 1, 0,
             0, 0, 1};

    rendering::DistortionPtr dPtr = camera->LensDistortion();

    // sensor_msgs::CameraInfo can manage only a few distortion
    // models. Here we assume plumb_bob just to pass along the
    // coefficients. Out of all the simulated cameras, nav_cam is
    // fisheye, which uses only K1, and all others have zero
    // distortion. Hence this code was not tested in the most general
    // setting.
    msg.distortion_model = "plumb_bob";
    if (dPtr) {
      #if GAZEBO_MAJOR_VERSION > 8
      msg.D = {camera->LensDistortion()->K1(),
               camera->LensDistortion()->K2(),
               camera->LensDistortion()->K3(),
               camera->LensDistortion()->P1(),
               camera->LensDistortion()->P2()};
      #else
      msg.D = {camera->LensDistortion()->GetK1(),
               camera->LensDistortion()->GetK2(),
               camera->LensDistortion()->GetK3(),
               camera->LensDistortion()->GetP1(),
               camera->LensDistortion()->GetP2()};
      #endif
    } else {
      msg.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    }
  }

}  // namespace gazebo
