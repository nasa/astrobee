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
#ifndef ROS_GRAPH_VIO_IMU_BIAS_INITIALIZER_H_
#define ROS_GRAPH_VIO_IMU_BIAS_INITIALIZER_H_

#include <imu_integration/dynamic_imu_filter.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/imu_measurement.h>
#include <msg_conversions/msg_conversions.h>

#include <gtsam/navigation/ImuBias.h>

#include <vector>

namespace ros_graph_vio {
// Buffers IMU measurements (and flight speed mode if available to help filter speed specific vibration noise) and
// estimates IMU biases. Assumes standstill behavior so the estimated bias is simply the average of the buffered
// measurements. Saves biases to a file and optionally loads from a file as well.
class ImuBiasInitializer {
 public:
  // Add flight speed mode measurement for IMU bias filter.
  void AddFlightSpeedModeMeasurement(const lm::FanSpeedMode fan_speed_mode);

  // Add IMU measurement. Estimate biases if enough measurements have been received and
  // save this to a file.
  void AddImuMeasurement(const localization_measurements::ImuMeasurement& imu_measurement);

  // Returns bias if it is available.
  boost::optional<gtsam::imuBias::ConstantBias> Bias() const;

  // Clears measurement buffer, filter, and estimated bias.
  void ResetBiases();

  // Loads the IMU bias from a file (filename set in params).
  bool LoadFromFile();

  // Save biases to a file (filename set in params).
  bool SaveToFile() const;

 private:
  boost::optional<gtsam::imuBias::ConstantBias> imu_bias_;
  std::unique_ptr<imu_integration::DynamicImuFilter> imu_bias_filter_;
  std::vector<localization_measurements::ImuMeasurement> imu_bias_measurements_;
  ImuBiasInitializerParams params_;
};
}  // namespace ros_graph_vio

#endif  // ROS_GRAPH_VIO_IMU_BIAS_INITIALIZER_H_
