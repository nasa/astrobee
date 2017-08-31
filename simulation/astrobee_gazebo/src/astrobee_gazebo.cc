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

// TF2 eigen bindings
#include <tf2_eigen/tf2_eigen.h>

namespace gazebo {

// Model plugin

FreeFlyerModelPlugin::FreeFlyerModelPlugin(std::string const& name, bool heartbeats) :
  ff_util::FreeFlyerNodelet(name, heartbeats), name_(name) {}

FreeFlyerModelPlugin::~FreeFlyerModelPlugin() {
  thread_.join();
}

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
  nh_.setCallbackQueue(&queue_);

  // Start a custom queue thread for messages
  thread_ = boost::thread(
    boost::bind(&FreeFlyerModelPlugin::QueueThread, this));

  // Call initialize on the freeflyer nodelet to start heartbeat and faults
  Setup(nh_);

  // Pass the new callback queue
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

// Get the extrinsics frame
std::string FreeFlyerModelPlugin::GetFrame(std::string target) {
  std::string frame = (target.empty() ? "body" : target);
  if (GetModel()->GetName() == "/")
    return frame;
  return GetModel()->GetName() + "/" + frame;
}

// Put laser data to the interface
void FreeFlyerModelPlugin::QueueThread() {
  while (nh_.ok())
    queue_.callAvailable(ros::WallDuration(0.1));
}

// Sensor plugin

FreeFlyerSensorPlugin::FreeFlyerSensorPlugin(std::string const& name, bool heartbeats) :
  ff_util::FreeFlyerNodelet(name, heartbeats), name_(name) {}

FreeFlyerSensorPlugin::~FreeFlyerSensorPlugin() {
  thread_.join();
}

void FreeFlyerSensorPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
  gzmsg << "[FF] Loading plugin for sensor with name " << name_ << "\n";

  // Get the world in which this sensor exists, and the link it is attached to
  sensor_ = sensor;
  sdf_ = sdf;
  world_ = gazebo::physics::get_world(sensor->WorldName());
  model_ = boost::static_pointer_cast < physics::Link > (
    world_->GetEntity(sensor->ParentName()))->GetModel();

  // If we specify a frame name different to our sensor tag name
  if (sdf->HasElement("frame"))
    frame_ = sdf->Get<std::string>("frame");
  else
    frame_ =  sensor_->Name();

  // If we specify a different sensor type
  if (sdf->HasElement("rotation_type"))
    rotation_type_ = sdf->Get<std::string>("rotation_type");
  else
    rotation_type_ =  sensor_->Type();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");

  // Get a nodehandle based on the model name and use a different default queue
  // We have to do this to avoid gazebo main thread blocking the ROS queue.
  nh_ = ros::NodeHandle(model_->GetName());
  nh_.setCallbackQueue(&queue_);

  // Start a custom queue thread for messages
  thread_ = boost::thread(
    boost::bind(&FreeFlyerSensorPlugin::QueueThread, this));

  // Call initialize on the freeflyer nodelet to start heartbeat and faults
  Setup(nh_);

  // Pass the new callback queue
  LoadCallback(&nh_, sensor_, sdf_);

  // Defer the extrinsics setup to allow plugins to load
  timer_ = nh_.createTimer(ros::Duration(1.0),
    &FreeFlyerSensorPlugin::SetupExtrinsics, this, true, true);
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

// Put laser data to the interface
void FreeFlyerSensorPlugin::QueueThread() {
  while (nh_.ok())
    queue_.callAvailable(ros::WallDuration(0.1));
}

// Manage the extrinsics based on the sensor type
void FreeFlyerSensorPlugin::SetupExtrinsics(const ros::TimerEvent&) {
  // Create a buffer and listener for TF2 transforms
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  // Get extrinsics from framestore
  try {
    // Lookup the transform for this sensor
    geometry_msgs::TransformStamped tf = buffer.lookupTransform(
      GetFrame("body"), GetFrame(), ros::Time(0), ros::Duration(60.0));

    // Handle the transform for all sensor types
    ignition::math::Pose3d pose(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z,
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);
    sensor_->SetPose(pose);
    gzmsg << "Extrinsics set for sensor " << name_ << "\n";

    // Get the pose as an Eigen type
    Eigen::Quaterniond pose_temp = Eigen::Quaterniond(tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);

    // define the two rotations needed to transform from the flight software camera pose to the
    // gazebo camera pose
    Eigen::Quaterniond rot_90_x = Eigen::Quaterniond(0.70710678, 0.70710678, 0, 0);
    Eigen::Quaterniond rot_90_z = Eigen::Quaterniond(0.70710678, 0, 0, 0.70710678);
    pose_temp = pose_temp * rot_90_x;
    pose_temp = pose_temp * rot_90_z;

    pose = ignition::math::Pose3d(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
        pose_temp.w(), pose_temp.x(), pose_temp.y(), pose_temp.z());

    // transform the pose into the world frame and set the camera world pose
    math::Pose tf_bs = pose;
    math::Pose tf_wb = model_->GetWorldPose();
    math::Pose tf_ws = tf_bs + tf_wb;
    ignition::math::Pose3d world_pose = ignition::math::Pose3d(tf_ws.pos.x, tf_ws.pos.y,
      tf_ws.pos.z, tf_ws.rot.w, tf_ws.rot.x, tf_ws.rot.y, tf_ws.rot.z);

    ////////////////////////////
    // SPECIAL CASE 1: CAMERA //
    ////////////////////////////

    if (sensor_->Type() == "camera") {
      // Dynamically cast to the correct sensor
      sensors::CameraSensorPtr sensor
        = std::dynamic_pointer_cast < sensors::CameraSensor > (sensor_);
      if (!sensor)
        gzerr << "Extrinsics requires a camera sensor as a parent.\n";
      // set the sensor pose to the pose from tf2 static
      sensor_->SetPose(pose);
      sensor->Camera()->SetWorldPose(world_pose);
      gzmsg << "Extrinsics update for camera " << name_ << "\n";
    }

    //////////////////////////////////////
    // SPECIAL CASE 2: WIDEANGLE CAMERA //
    //////////////////////////////////////

    if (sensor_->Type() == "wideanglecamera") {
      // Dynamically cast to the correct sensor
      sensors::WideAngleCameraSensorPtr sensor =
        std::dynamic_pointer_cast < sensors::WideAngleCameraSensor > (sensor_);
      if (!sensor)
        gzerr << "Extrinsics requires a wideanglecamera sensor as a parent.\n";
      // set the sensor pose to the pose from tf2 static
      sensor_->SetPose(pose);
      sensor->Camera()->SetWorldPose(world_pose);
      gzmsg << "Extrinsics update for wideanglecamera " << name_ << "\n";
    }

    //////////////////////////////////
    // SPECIAL CASE 3: DEPTH CAMERA //
    //////////////////////////////////

    if (sensor_->Type() == "depth") {
      // Dynamically cast to the correct sensor
      sensors::DepthCameraSensorPtr sensor =
        std::dynamic_pointer_cast < sensors::DepthCameraSensor > (sensor_);
      if (!sensor)
        gzerr << "Extrinsics requires a depth camera sensor as a parent.\n";

      sensor->DepthCamera()->SetWorldPose(world_pose);
      gzmsg << "Extrinsics update for depth camera " << name_ << "\n";
    }
  } catch (tf2::TransformException &ex) {
    gzmsg << "[FF] No extrinsics for sensor " << name_ << "\n";
  }
}

}  // namespace gazebo
