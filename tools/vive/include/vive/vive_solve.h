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

#ifndef VIVE_VIVE_SOLVE_H_
#define VIVE_VIVE_SOLVE_H_

#include <ros/ros.h>

#include <vive/vive.h>

// Incoming measurements
#include <sensor_msgs/Imu.h>
#include <ff_msgs/ViveLight.h>
#include <geometry_msgs/TransformStamped.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// STD C includes
#include <math.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// STD C++ includes
#include <map>
#include <mutex>
#include <vector>
#include <thread>
#include <string>

/**
 * \ingroup tools
 */
namespace vive {

  extern double start_pose[6];

// Internal types

struct LightVecStamped {
  LightVec lights;
  ros::Time stamp;
};

struct AxisLightVec {
  std::string lighthouse;
  std::map<uint8_t, LightVecStamped> axis;
};

typedef std::map<std::string, AxisLightVec> LightData;

struct Extrinsics {
  double positions[3 * TRACKER_SENSORS_NUMBER];
  double normals[3 * TRACKER_SENSORS_NUMBER];
  double radius;
  size_t size;
};

struct SolvedPose {
  double transform[6];
  std::string lighthouse;
  bool valid;
  ros::Time stamp;
};

// A class to solve for the position of a single tracker
class ViveSolve {
 public:
  // Constructor
  ViveSolve();

  // Destructor
  ~ViveSolve();

  // Process an IMU measurement
  void ProcessImu(const sensor_msgs::Imu::ConstPtr& msg);

  // Process a light measurement
  void ProcessLight(const ff_msgs::ViveLight::ConstPtr& msg);

  // Get the current pose according to the solver
  bool GetTransform(geometry_msgs::TransformStamped &msg);

  // Set all the necessary parameters
  bool Initialize(Tracker const& tracker);

  // Set all the necessary parameters
  bool Initialize(Environment const& environment,
    Tracker const& tracker);

  // Updates the calibration file in case it gets updated
  bool Update(Environment const& environment);

 private:
  std::map<std::string, SolvedPose> poses_;
  SolvedPose tracker_pose_;
  Environment environment_;
  LightData observations_;
  Extrinsics extrinsics_;
  std::mutex solveMutex_;
  Tracker tracker_;
};

// Computes the full pose of a tracker for each lighthouse
bool ComputeTransform(AxisLightVec observations,
  SolvedPose * pose_tracker,
  std::string * last_lh_pose,
  Extrinsics * extrinsics,
  std::mutex * solveMutex);

// Computes the full pose of a tracker for all lighthouse - handles poses in the vive frame
bool ComputeTransformBundle(LightData observations,
  SolvedPose * pose_tracker,
  Extrinsics * extrinsics,
  Environment * environment,
  std::mutex * solveMutex);

}  // namespace vive

#endif  // VIVE_VIVE_SOLVE_H_
