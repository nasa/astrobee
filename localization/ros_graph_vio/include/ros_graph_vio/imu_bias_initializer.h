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
#include <ros_graph_vio/imu_bias_initializer_params.h>

#include <gtsam/navigation/ImuBias.h>

#include <vector>

namespace ros_graph_vio {
struct ImuBiasWithStddev {
  ImuBiasWithStddev(const gtsam::imuBias::ConstantBias& bias, const Eigen::Vector3d& accelerometer_bias_stddev,
                    const Eigen::Vector3d& gyro_bias_stddev)
      : bias(bias), accelerometer_bias_stddev(accelerometer_bias_stddev), gyro_bias_stddev(gyro_bias_stddev) {}
  gtsam::imuBias::ConstantBias bias;
  Eigen::Vector3d accelerometer_bias_stddev;
  Eigen::Vector3d gyro_bias_stddev;
};

// Buffers IMU measurements (and flight speed mode if available to help filter speed specific vibration noise) and
// estimates IMU biases. Assumes standstill behavior so the estimated bias is simply the average of the buffered
// measurements. Saves biases to a file and optionally loads from a file as well.
// If using in a gravity environment, assumes the IMU orientation wrt gravity doesn't
// change, so gravity is considered a constant IMU bias that contributes to the estimated IMU bias.
class ImuBiasInitializer {
 public:
  // Construct with params.
  explicit ImuBiasInitializer(const ImuBiasInitializerParams& params);

  // Add fan speed mode measurement for IMU bias filter.
  void AddFanSpeedModeMeasurement(const localization_measurements::FanSpeedMode fan_speed_mode);

  // Add IMU measurement. Estimate biases if enough measurements have been received and
  // save this to a file.
  void AddImuMeasurement(const localization_measurements::ImuMeasurement& imu_measurement);

  // Returns bias if it is available.
  boost::optional<ImuBiasWithStddev> Bias() const;

  // Manually sets the bias and saves it to file.
  void UpdateBias(const ImuBiasWithStddev& bias);

  // Clears measurement buffer, filter, and estimated bias.
  void Reset();

  // Loads the IMU bias from a file (filename set in params).
  bool LoadFromFile();

  // Save biases to a file (filename set in params).
  bool SaveToFile() const;

 private:
  boost::optional<ImuBiasWithStddev> imu_bias_;
  std::unique_ptr<imu_integration::DynamicImuFilter> imu_bias_filter_;
  std::vector<localization_measurements::ImuMeasurement> imu_bias_measurements_;
  ImuBiasInitializerParams params_;
};
}  // namespace ros_graph_vio

#endif  // ROS_GRAPH_VIO_IMU_BIAS_INITIALIZER_H_
