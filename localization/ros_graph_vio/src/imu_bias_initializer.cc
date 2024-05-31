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

#include <localization_common/logger.h>
#include <ros_graph_vio/imu_bias_initializer.h>

#include <iostream>
#include <string>

namespace ros_graph_vio {
namespace lm = localization_measurements;
ImuBiasInitializer::ImuBiasInitializer(const ImuBiasInitializerParams& params) : params_(params) { Reset(); }

void ImuBiasInitializer::AddFanSpeedModeMeasurement(const lm::FanSpeedMode fan_speed_mode) {
  imu_bias_filter_->SetFanSpeedMode(fan_speed_mode);
}

void ImuBiasInitializer::AddImuMeasurement(const localization_measurements::ImuMeasurement& imu_measurement) {
  // Nothing to do if bias already estimated.
  if (imu_bias_) return;

  // Add filtered measurement.
  const auto filtered_imu_measurement = imu_bias_filter_->AddMeasurement(imu_measurement);
  if (filtered_imu_measurement) {
    imu_bias_measurements_.emplace_back(*filtered_imu_measurement);
  }
  // Nothing to do if not enough measurements have been received.
  if (static_cast<int>(imu_bias_measurements_.size()) < params_.num_bias_estimation_measurements) return;

  // Estimate bias if enough measurements have been received.
  // Assumes standstill, so simply average accelerometer and gyro measurements.
  Eigen::Vector3d sum_of_acceleration_measurements = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_of_angular_velocity_measurements = Eigen::Vector3d::Zero();
  for (const auto& imu_measurement : imu_bias_measurements_) {
    sum_of_acceleration_measurements += imu_measurement.acceleration;
    sum_of_angular_velocity_measurements += imu_measurement.angular_velocity;
  }

  const Eigen::Vector3d accelerometer_bias = sum_of_acceleration_measurements / imu_bias_measurements_.size();
  const Eigen::Vector3d gyro_bias = sum_of_angular_velocity_measurements / imu_bias_measurements_.size();
  LogDebug("Estimated accelerometer bias: " << std::endl << accelerometer_bias.matrix());
  LogDebug("Estimated gyro bias: " << std::endl << gyro_bias.matrix());

  // Compute standard deviations
  Eigen::Vector3d accelerometer_variance = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_variance = Eigen::Vector3d::Zero();
  for (const auto& imu_measurement : imu_bias_measurements_) {
    accelerometer_variance += (imu_measurement.acceleration - accelerometer_bias).cwiseAbs2();
    gyro_variance += (imu_measurement.angular_velocity - gyro_bias).cwiseAbs2();
  }
  accelerometer_variance /= static_cast<double>(imu_bias_measurements_.size());
  gyro_variance /= static_cast<double>(imu_bias_measurements_.size());
  imu_bias_ =
    ImuBiasWithStddev(gtsam::imuBias::ConstantBias(accelerometer_bias, gyro_bias),
                      Eigen::Vector3d(accelerometer_variance.cwiseSqrt()), Eigen::Vector3d(gyro_variance.cwiseSqrt()));
  SaveToFile();
}

boost::optional<ImuBiasWithStddev> ImuBiasInitializer::Bias() const { return imu_bias_; }

void ImuBiasInitializer::UpdateBias(const ImuBiasWithStddev& bias) {
  imu_bias_ = bias;
  SaveToFile();
}

void ImuBiasInitializer::Reset() {
  imu_bias_ = boost::none;
  imu_bias_filter_.reset(new imu_integration::DynamicImuFilter(params_.filter));
  imu_bias_measurements_.clear();
}

bool ImuBiasInitializer::LoadFromFile() {
  std::ifstream imu_bias_file(params_.imu_bias_filename);
  if (!imu_bias_file.is_open()) {
    LogDebug("LoadFromFile: Failed to read imu bias file.");
    return false;
  }

  Eigen::Vector3d accelerometer_bias, gyro_bias, accelerometer_bias_stddev, gyro_bias_stddev;
  for (int line_num = 0; line_num < 4; ++line_num) {
    std::string line;
    if (!std::getline(imu_bias_file, line)) {
      LogError("LoadFromFile: Failed to get line from imu bias file.");
      return false;
    }
    std::istringstream line_stream(line);
    std::string val;
    for (int val_index = 0; val_index < 3; ++val_index) {
      if (!std::getline(line_stream, val, ',')) {
        LogError("LoadFromFile: Failed to get value from imu bias file.");
        return false;
      }
      if (line_num == 0) {
        accelerometer_bias[val_index] = std::stold(val);
      } else if (line_num == 1) {
        gyro_bias[val_index] = std::stold(val);
      } else if (line_num == 2) {
        accelerometer_bias_stddev[val_index] = std::stold(val);
      } else {
        gyro_bias_stddev[val_index] = std::stold(val);
      }
    }
  }
  imu_bias_ = ImuBiasWithStddev(gtsam::imuBias::ConstantBias(accelerometer_bias, gyro_bias), accelerometer_bias_stddev,
                                gyro_bias_stddev);
  return true;
}

bool ImuBiasInitializer::SaveToFile() const {
  if (!imu_bias_) {
    LogError("SaveToFile: No IMU bias available.");
    return false;
  }
  std::ofstream imu_bias_file(params_.imu_bias_filename);
  if (!imu_bias_file.is_open()) {
    LogError("SaveToFile: Failed to create imu bias output file.");
    return false;
  }
  const auto& accel_bias = imu_bias_->bias.accelerometer();
  imu_bias_file << accel_bias.x() << "," << accel_bias.y() << "," << accel_bias.z() << std::endl;
  const auto& gyro_bias = imu_bias_->bias.gyroscope();
  imu_bias_file << gyro_bias.x() << "," << gyro_bias.y() << "," << gyro_bias.z() << std::endl;
  const auto& accelerometer_bias_stddev = imu_bias_->accelerometer_bias_stddev;
  imu_bias_file << accelerometer_bias_stddev.x() << "," << accelerometer_bias_stddev.y() << ","
                << accelerometer_bias_stddev.z() << std::endl;
  const auto& gyro_bias_stddev = imu_bias_->gyro_bias_stddev;
  imu_bias_file << gyro_bias_stddev.x() << "," << gyro_bias_stddev.y() << "," << gyro_bias_stddev.z() << std::endl;
  imu_bias_file.close();
  return true;
}
}  // namespace ros_graph_vio
