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

#ifndef VIVE_VIVE_CALIBRATE_H_
#define VIVE_VIVE_CALIBRATE_H_

#include <ros/ros.h>

#include <vive/vive_solve.h>
#include <vive/vive.h>

// Incoming measurements
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <ff_msgs/ViveLight.h>
#include <ff_msgs/ViveCalibration.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// C++11 includes
#include <utility>
#include <vector>
#include <map>
#include <functional>
#include <thread>
#include <mutex>
#include <string>

// Calibration callback function
typedef std::function<void(vive::Calibration const&)> CallbackFn;

/**
 * \ingroup tools
 */
namespace vive {

// Internal datatypes
struct Sweep {
  LightVec lights;
  std::string lighthouse;
  uint8_t axis;
};
typedef std::vector<Sweep> SweepVec;
typedef std::pair<SweepVec, ImuVec> DataPair;         // pair of Light data and Imu data - change imu
typedef std::map<std::string, DataPair> DataPairMap;         // map of trackers


class ViveCalibrate {
 public:
  explicit ViveCalibrate(CallbackFn cb);

  // Reset the solver
  bool Reset();

  // Process an IMU measurement
  bool AddImu(const sensor_msgs::Imu::ConstPtr& msg);

  // Process a light measurement
  bool AddLight(const ff_msgs::ViveLight::ConstPtr& msg);

  // Send all the necessary information in a single structure
  bool Initialize(Calibration & calibration);

  // Solve the problem based on the assumption tge body of tracker's is still.
  bool Solve();

  // Thread that solves
  static void WorkerThread(CallbackFn cb,
    std::mutex * calibration_mutex,
    DataPairMap data_pair_map,
    Calibration calibration);

 private:
  DataPairMap data_pair_map_;   // Input data
  CallbackFn cb_;               // Solution callback
  std::mutex mutex_;            // Mutex for data access
  bool active_;                 // If the calibration procedure is active
  Calibration calibration_;     // Structure that saves all the data
};

}  // namespace vive

#endif  // VIVE_VIVE_CALIBRATE_H_
