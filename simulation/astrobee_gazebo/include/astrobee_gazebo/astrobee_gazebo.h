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

#ifndef ASTROBEE_GAZEBO_ASTROBEE_GAZEBO_H_
#define ASTROBEE_GAZEBO_ASTROBEE_GAZEBO_H_

// ROS includes
#include <ros/ros.h>
#include <ros/callback_queue.h>

// General messages
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Transformation helper code
#include <tf2_ros/transform_listener.h>

// FSW includes
#include <ff_util/ff_nodelet.h>

// Gazebo includes
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/rendering/rendering.hh>

// STL includes
#include <string>
#include <limits>

namespace gazebo {

// Convenience wrapper around a model plugin
class FreeFlyerModelPlugin : public ModelPlugin, public ff_util::FreeFlyerNodelet {
 public:
  explicit FreeFlyerModelPlugin(std::string const& name, bool heartbeats = true);

  virtual ~FreeFlyerModelPlugin();

  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

  // Get the model link
  physics::LinkPtr GetLink();

  // Get the model world
  physics::WorldPtr GetWorld();

  // Get the model
  physics::ModelPtr GetModel();

  // Get the extrinsics frame
  std::string GetFrame(std::string target = "");

  // Callback when the model has loaded
  virtual void LoadCallback(ros::NodeHandle *nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) = 0;

 protected:
  // Put laser data to the interface
  void QueueThread();

 private:
  std::string name_;
  sdf::ElementPtr sdf_;
  physics::LinkPtr link_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  boost::thread thread_;
};


// Convenience wrapper around a sensor plugin
class FreeFlyerSensorPlugin : public SensorPlugin, public ff_util::FreeFlyerNodelet {
 public:
  explicit FreeFlyerSensorPlugin(std::string const& name, bool heartbeats = true);

  virtual ~FreeFlyerSensorPlugin();

 protected:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

  // Get the sensor world
  physics::WorldPtr GetWorld();

  // Get the sensor model
  physics::ModelPtr GetModel();

  // Get the extrinsics frame
  std::string GetFrame(std::string target = "");

  // Get the type of the sensor
  std::string GetRotationType();

  // Callback when the sensor has loaded
  virtual void LoadCallback(ros::NodeHandle *nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) = 0;

 protected:
  // Put laser data to the interface
  void QueueThread();

  // Manage the extrinsics based on the sensor type
  void SetupExtrinsics(const ros::TimerEvent&);

 private:
  std::string name_;
  std::string frame_;
  std::string rotation_type_;
  sensors::SensorPtr sensor_;
  sdf::ElementPtr sdf_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  ros::Timer timer_;
  boost::thread thread_;
};

}  // namespace gazebo

#endif  // ASTROBEE_GAZEBO_ASTROBEE_GAZEBO_H_
