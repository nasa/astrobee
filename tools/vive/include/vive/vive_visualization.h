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

#ifndef VIVE_VIVE_VISUALIZATION_H_
#define VIVE_VIVE_VISUALIZATION_H_

// Vive
#include <vive/vive.h>

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

// ROS.msgs
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>

// FF.msgs
#include <ff_msgs/ViveLight.h>
#include <ff_msgs/ViveLightSample.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// C++.std
#include <string>
#include <map>

/**
 * \ingroup tools
 */
namespace vive {

// In class tools
typedef std::map<int, Light> LightMap;

struct LightDataSingle {
  LightMap horizontal_lights;
  LightMap vertical_lights;
  bool status;
};

typedef std::map<std::string, LightDataSingle> LightDataVisual;

typedef std::map<std::string, int> LighthouseId;

struct ImuDataVisual {
  Imu imu;
  ros::Time stamp;
  bool status;
};

struct TrackerStamped {
  Tracker tracker;
  int id;
};

class Visualization {
 public:
  // Constructor
  Visualization();

  // Destroyer
  ~Visualization();

  // Initialize the system
  void Initialize(Tracker const& tracker, int id);

  // // Set all necessary components for visualization
  // void Set(geometry_msgs::TransformStamped & tf);

  // Add Imu data
  void AddImu(const sensor_msgs::Imu::ConstPtr& msg);

  // Add Light data
  void AddLight(const ff_msgs::ViveLight::ConstPtr& msg);

  // Get an arrow that shows the IMU data
  bool GetImu(visualization_msgs::Marker * arrow);

  // Get list of lines that show how the sensors are located
  bool GetLight(visualization_msgs::MarkerArray * directions);

  // Get the list of positions of all the sensors
  bool GetSensors(visualization_msgs::MarkerArray * sensors);

 private:
  tf2_ros::TransformListener * tf_listener_;
  TrackerStamped tracker_stamped_;
  LightDataVisual light_data_;
  LighthouseId lighthouse_id_;
  ImuDataVisual imu_data_;
  tf2_ros::Buffer buffer_;
  PoseVM pose_tracker_;
};

}  // namespace vive

#endif  // VIVE_VIVE_VISUALIZATION_H_
