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

#ifndef ASTROBEE_GAZEBO_ASTROBEE_GAZEBO_H_
#define ASTROBEE_GAZEBO_ASTROBEE_GAZEBO_H_

// ROS includes
#include <ff_common/ff_ros.h>
// #include <ros/callback_queue.h>

// Transformation helper code
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// FSW includes
#include <ff_util/ff_component.h>

// General messages
#include <geometry_msgs/msg/transform_stamped.hpp>
namespace geometry_msgs {
typedef msg::TransformStamped TransformStamped;
}  // namespace geometry_msgs
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
namespace sensor_msgs {
typedef msg::PointCloud2 PointCloud2;
typedef msg::Image Image;
typedef msg::CameraInfo CameraInfo;
}  // namespace sensor_msgs

// Gazebo includes
#include <gazebo_ros/node.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#if GAZEBO_MAJOR_VERSION <= 7
#include <gazebo/math/gzmath.hh>
#endif
#include <gazebo/rendering/rendering.hh>

// Eigen includes
#include <Eigen/Geometry>

// STL includes
#include <string>
#include <thread>
#include <memory>

namespace gazebo {

// Convenience wrapper around a model plugin
class FreeFlyerPlugin : public ff_util::FreeFlyerComponent {
 public:
  // Constructor
  explicit FreeFlyerPlugin(std::string const& plugin_name,
    std::string const& plugin_frame, bool send_heartbeats = false);

  // Destructor
  virtual ~FreeFlyerPlugin();

 protected:
  // Initialize the plugin
  void InitializePlugin(std::string const& robot_name, std::string const& plugin_name, sdf::ElementPtr sdf);

  // Some plugins might want the world as the parent frame
  void SetParentFrame(std::string const& parent);

  // Get the robot-prefixed frame name for the given traget frame
  std::string GetFrame(std::string target = "", std::string delim = "/");

  // Called when extrinsics become available
  virtual bool ExtrinsicsCallback(
    geometry_msgs::TransformStamped const* tf) = 0;

  // Optional callback for nodes to know when extrinsics were received
  virtual void OnExtrinsicsReceived(NodeHandle node) {}

  // Manage the extrinsics based on the sensor type
  void SetupExtrinsics();

  // Custom callback queue to avoid contention between the global callback
  // queue and gazebo update work.
  // void CallbackThread();

  // Child classes need access
  std::string robot_name_, plugin_name_, plugin_frame_, parent_frame_;
  NodeHandle nh_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  // ros::CallbackQueue callback_queue_;
  std::thread thread_;
  ff_util::FreeFlyerTimer timer_;
};

// Convenience wrapper around a model plugin
class FreeFlyerModelPlugin : public FreeFlyerPlugin, public ModelPlugin {
 public:
  // Constructor
  explicit FreeFlyerModelPlugin(std::string const& plugin_name,
    std::string const& plugin_frame, bool send_heartbeats = false);

  // Destructor
  virtual ~FreeFlyerModelPlugin();

 protected:
  // Called when the model is loaded
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

  // Get the model link
  physics::LinkPtr GetLink();

  // Get the model world
  physics::WorldPtr GetWorld();

  // Get the model
  physics::ModelPtr GetModel();

  // Callback when the model has loaded
  virtual void LoadCallback(NodeHandle &nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) = 0;

  // Manage the extrinsics based on the sensor type
  virtual bool ExtrinsicsCallback(geometry_msgs::TransformStamped const* tf);

 private:
  sdf::ElementPtr sdf_;
  physics::LinkPtr link_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;
};


// Convenience wrapper around a sensor plugin
class FreeFlyerSensorPlugin : public FreeFlyerPlugin, public SensorPlugin {
 public:
  // Constructor
  explicit FreeFlyerSensorPlugin(std::string const& plugin_name,
    std::string const& plugin_frame, bool send_heartbeats = false);

  // Destructor
  virtual ~FreeFlyerSensorPlugin();

 protected:
  // Called when the sensor is loaded
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

  // Get the sensor world
  physics::WorldPtr GetWorld();

  // Get the sensor model
  physics::ModelPtr GetModel();

  // Get the type of the sensor
  std::string GetRotationType();

  // Callback when the sensor has loaded
  virtual void LoadCallback(NodeHandle &nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) = 0;

  // Manage the extrinsics based on the sensor type
  virtual bool ExtrinsicsCallback(geometry_msgs::TransformStamped const* tf);

 private:
  sensors::SensorPtr sensor_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
};

// Utility functions

// Find the transform from the sensor to the world
#if GAZEBO_MAJOR_VERSION > 7
Eigen::Affine3d SensorToWorld(ignition::math::Pose3d const& world_pose,
                              ignition::math::Pose3d const& sensor_pose);
#else
Eigen::Affine3d SensorToWorld(gazebo::math::Pose const& world_pose,
                              ignition::math::Pose3d const& sensor_pose);
#endif

// Read the camera info
void FillCameraInfo(rendering::CameraPtr camera, sensor_msgs::CameraInfo & info_msg);

}  // namespace gazebo

#endif  // ASTROBEE_GAZEBO_ASTROBEE_GAZEBO_H_
