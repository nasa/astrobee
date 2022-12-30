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

#include <graph_vio/graph_vio_initializer.h>
#include <graph_vio/parameter_reader.h>
#include <graph_vio/utilities.h>
#include <localization_common/utilities.h>

#include <iostream>
#include <string>

namespace graph_vio {
namespace lc = localization_common;
namespace lm = localization_measurements;
GraphLocalizerInitializer::GraphLocalizerInitializer()
    : has_biases_(false),
      has_start_pose_(false),
      has_params_(false),
      has_fan_speed_mode_(false),
      estimate_biases_(false),
      removed_gravity_from_bias_if_necessary_(false) {}
void GraphLocalizerInitializer::SetBiases(const gtsam::imuBias::ConstantBias& imu_bias,
                                          const bool loaded_from_previous_estimate, const bool save_to_file) {
  params_.graph_initializer.initial_imu_bias = imu_bias;
  has_biases_ = true;
  estimate_biases_ = false;
  // Assumes previous bias is already gravity compensated if neccessary
  if (!loaded_from_previous_estimate) RemoveGravityFromBiasIfPossibleAndNecessary();
  if (save_to_file) {
    std::ofstream imu_bias_file(params_.graph_initializer.imu_bias_filename);
    if (!imu_bias_file.is_open()) {
      LogError("SetBiases: Failed to create imu bias output file.");
      return;
    }
    const auto& accel_bias = params_.graph_initializer.initial_imu_bias.accelerometer();
    imu_bias_file << accel_bias.x() << "," << accel_bias.y() << "," << accel_bias.z() << std::endl;
    const auto& gyro_bias = params_.graph_initializer.initial_imu_bias.gyroscope();
    imu_bias_file << gyro_bias.x() << "," << gyro_bias.y() << "," << gyro_bias.z() << std::endl;
    imu_bias_file.close();
  }
}

void GraphLocalizerInitializer::SetStartPose(const lm::TimestampedPose& timestamped_pose) {
  params_.graph_initializer.start_time = timestamped_pose.time;
  params_.graph_initializer.global_T_body_start = timestamped_pose.pose;
  // Assumes zero initial velocity
  params_.graph_initializer.global_V_body_start = gtsam::Velocity3::Zero();
  has_start_pose_ = true;
  RemoveGravityFromBiasIfPossibleAndNecessary();
}

void GraphLocalizerInitializer::SetFanSpeedMode(const lm::FanSpeedMode fan_speed_mode) {
  params_.initial_fan_speed_mode = fan_speed_mode;
  has_fan_speed_mode_ = true;
}

void GraphLocalizerInitializer::RemoveGravityFromBiasIfPossibleAndNecessary() {
  if (RemovedGravityFromBiasIfNecessary() || !HasParams()) return;
  if (params_.graph_initializer.gravity.isZero()) {
    removed_gravity_from_bias_if_necessary_ = true;
    return;
  }
  if (!HasStartPose() || !HasBiases()) return;
  // Biases, start pose and params are available and gravity is non zero, gravity can and should now be removed
  // from the initial bias estimates.
  LogDebug("RemoveGravityFromBiasIfPossibleAndNecessary: Removing gravity from initial biases.");
  RemoveGravityFromBias(params_.graph_initializer.gravity, params_.graph_initializer.body_T_imu,
                        params_.graph_initializer.global_T_body_start, params_.graph_initializer.initial_imu_bias);

  LogDebug("RemoveGravityFromBiasIfPossibleAndNecessary: New gravity corrected accelerometer bias: "
           << params_.graph_initializer.initial_imu_bias.accelerometer().matrix());
  removed_gravity_from_bias_if_necessary_ = true;
  return;
}

void GraphLocalizerInitializer::ResetBiasesAndStartPose() {
  ResetBiases();
  ResetStartPose();
}

void GraphLocalizerInitializer::ResetBiasesFromFileAndResetStartPose() {
  ResetBiasesFromFile();
  ResetStartPose();
}

void GraphLocalizerInitializer::ResetStartPose() { has_start_pose_ = false; }

void GraphLocalizerInitializer::ResetBiases() {
  has_biases_ = false;
  imu_bias_filter_.reset(new imu_integration::DynamicImuFilter(params_.graph_initializer.filter));
  imu_bias_measurements_.clear();
  StartBiasEstimation();
}

void GraphLocalizerInitializer::ResetBiasesFromFile() {
  std::ifstream imu_bias_file(params_.graph_initializer.imu_bias_filename);
  if (!imu_bias_file.is_open()) {
    LogDebug("ResetBiasesFromFile: Failed to read imu bias file.");
    return;
  }

  gtsam::Vector3 accelerometer_bias;
  gtsam::Vector3 gyro_bias;
  for (int line_num = 0; line_num < 2; ++line_num) {
    std::string line;
    if (!std::getline(imu_bias_file, line)) {
      LogError("ResetBiasesFromFile: Failed to get line from imu bias file.");
      return;
    }
    std::istringstream line_stream(line);
    std::string val;
    for (int val_index = 0; val_index < 3; ++val_index) {
      if (!std::getline(line_stream, val, ',')) {
        LogError("ResetBiasesFromFile: Failed to get value from imu bias file.");
        return;
      }
      if (line_num == 0) {
        accelerometer_bias[val_index] = std::stold(val);
      } else {
        gyro_bias[val_index] = std::stold(val);
      }
    }
  }
  SetBiases(gtsam::imuBias::ConstantBias(accelerometer_bias, gyro_bias), true);
}

void GraphLocalizerInitializer::EstimateAndSetImuBiases(
  const localization_measurements::ImuMeasurement& imu_measurement, const lm::FanSpeedMode fan_speed_mode) {
  imu_bias_filter_->SetFanSpeedMode(fan_speed_mode);
  const auto filtered_imu_measurement = imu_bias_filter_->AddMeasurement(imu_measurement);
  if (filtered_imu_measurement) {
    imu_bias_measurements_.emplace_back(*filtered_imu_measurement);
  }
  if (static_cast<int>(imu_bias_measurements_.size()) < params_.graph_initializer.num_bias_estimation_measurements)
    return;

  Eigen::Vector3d sum_of_acceleration_measurements = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_of_angular_velocity_measurements = Eigen::Vector3d::Zero();
  for (const auto& imu_measurement : imu_bias_measurements_) {
    sum_of_acceleration_measurements += imu_measurement.acceleration;
    sum_of_angular_velocity_measurements += imu_measurement.angular_velocity;
  }

  LogDebug(
    "Number of imu measurements per bias estimate: " << params_.graph_initializer.num_bias_estimation_measurements);
  const Eigen::Vector3d accelerometer_bias = sum_of_acceleration_measurements / imu_bias_measurements_.size();
  const Eigen::Vector3d gyro_bias = sum_of_angular_velocity_measurements / imu_bias_measurements_.size();
  LogDebug("Accelerometer bias: " << std::endl << accelerometer_bias.matrix());
  LogDebug("Gyro bias: " << std::endl << gyro_bias.matrix());

  gtsam::imuBias::ConstantBias biases(accelerometer_bias, gyro_bias);
  SetBiases(biases, false, true);
  imu_bias_filter_.reset(new imu_integration::DynamicImuFilter(params_.graph_initializer.filter, fan_speed_mode));
  imu_bias_measurements_.clear();
}

void GraphLocalizerInitializer::RemoveGravityFromBias(const gtsam::Vector3& global_F_gravity,
                                                      const gtsam::Pose3& body_T_imu, const gtsam::Pose3& global_T_body,
                                                      gtsam::imuBias::ConstantBias& imu_bias) {
  const gtsam::Vector3 gravity_corrected_accelerometer_bias = lc::RemoveGravityFromAccelerometerMeasurement(
    global_F_gravity, body_T_imu, global_T_body, imu_bias.accelerometer());
  imu_bias = gtsam::imuBias::ConstantBias(gravity_corrected_accelerometer_bias, imu_bias.gyroscope());
}

void GraphLocalizerInitializer::LoadGraphLocalizerParams(config_reader::ConfigReader& config) {
  graph_vio::LoadGraphLocalizerParams(config, params_);
  has_params_ = true;
}

bool GraphLocalizerInitializer::ReadyToInitialize() const {
  return HasBiases() && HasStartPose() && HasParams() && HasFanSpeedMode() && RemovedGravityFromBiasIfNecessary();
}

void GraphLocalizerInitializer::StartBiasEstimation() { estimate_biases_ = true; }

bool GraphLocalizerInitializer::HasBiases() const { return has_biases_; }
bool GraphLocalizerInitializer::HasStartPose() const { return has_start_pose_; }
bool GraphLocalizerInitializer::HasParams() const { return has_params_; }
bool GraphLocalizerInitializer::HasFanSpeedMode() const { return has_fan_speed_mode_; }
bool GraphLocalizerInitializer::EstimateBiases() const { return estimate_biases_; }
bool GraphLocalizerInitializer::RemovedGravityFromBiasIfNecessary() const {
  return removed_gravity_from_bias_if_necessary_;
}

const GraphLocalizerParams& GraphLocalizerInitializer::params() const { return params_; }

}  // namespace graph_vio
