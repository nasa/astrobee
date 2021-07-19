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
#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_INITIALIZER_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_INITIALIZER_H_

#include <camera/camera_params.h>
#include <config_reader/config_reader.h>
#include <graph_localizer/graph_localizer_params.h>
#include <imu_integration/dynamic_imu_filter.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/imu_measurement.h>
#include <localization_measurements/timestamped_pose.h>
#include <msg_conversions/msg_conversions.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include <string>
#include <vector>

namespace graph_localizer {
class GraphLocalizerInitializer {
 public:
  GraphLocalizerInitializer();
  void SetBiases(const gtsam::imuBias::ConstantBias& imu_bias, const bool loaded_from_previous_estimate = false,
                 const bool save_to_file = false);
  void SetStartPose(const localization_measurements::TimestampedPose& timestamped_pose);
  void SetFanSpeedMode(const localization_measurements::FanSpeedMode fan_speed_mode);
  void RemoveGravityFromBiasIfPossibleAndNecessary();
  bool ReadyToInitialize() const;
  void ResetBiasesAndStartPose();
  void ResetBiasesFromFileAndResetStartPose();
  void ResetStartPose();
  void ResetBiases();
  void ResetBiasesFromFile();
  void StartBiasEstimation();
  bool HasBiases() const;
  bool HasStartPose() const;
  bool HasParams() const;
  bool HasFanSpeedMode() const;
  bool EstimateBiases() const;
  const GraphLocalizerParams& params() const;
  void LoadGraphLocalizerParams(config_reader::ConfigReader& config);
  bool RemovedGravityFromBiasIfNecessary() const;
  void EstimateAndSetImuBiases(const localization_measurements::ImuMeasurement& imu_measurement,
                               localization_measurements::FanSpeedMode fan_speed_mode);

 private:
  void RemoveGravityFromBias(const gtsam::Vector3& global_F_gravity, const gtsam::Pose3& body_T_imu,
                             const gtsam::Pose3& global_T_body, gtsam::imuBias::ConstantBias& imu_bias);

  bool has_biases_;
  bool has_start_pose_;
  bool has_params_;
  bool has_fan_speed_mode_;
  bool estimate_biases_;
  bool removed_gravity_from_bias_if_necessary_;
  graph_localizer::GraphLocalizerParams params_;
  std::unique_ptr<imu_integration::DynamicImuFilter> imu_bias_filter_;
  std::vector<localization_measurements::ImuMeasurement> imu_bias_measurements_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_INITIALIZER_H_
