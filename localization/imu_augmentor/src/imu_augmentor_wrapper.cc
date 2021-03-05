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

#include <config_reader/config_reader.h>
#include <imu_augmentor/imu_augmentor_wrapper.h>
#include <imu_integration/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/imu_measurement.h>
#include <localization_measurements/measurement_conversions.h>
#include <msg_conversions/msg_conversions.h>

namespace imu_augmentor {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

ImuAugmentorWrapper::ImuAugmentorWrapper(const std::string& graph_config_path_prefix) {
  config_reader::ConfigReader config;
  config.AddFile("transforms.config");
  config.AddFile("geometry.config");
  lc::LoadGraphLocalizerConfig(config, graph_config_path_prefix);

  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  ii::LoadImuIntegratorParams(config, params_);
  params_.standstill_enabled = mc::LoadBool(config, "imu_augmentor_standstill");
  imu_augmentor_.reset(new ImuAugmentor(params_));

  // Preintegration_helper_ is only being used to frame change and remove centrifugal acceleration, so body_T_imu is the
  // only parameter needed.
  boost::shared_ptr<gtsam::PreintegrationParams> preintegration_params(new gtsam::PreintegrationParams());
  preintegration_params->setBodyPSensor(params_.body_T_imu);
  preintegration_helper_.reset(new gtsam::TangentPreintegration(preintegration_params, gtsam::imuBias::ConstantBias()));
}

void ImuAugmentorWrapper::LocalizationStateCallback(const ff_msgs::GraphState& loc_msg) {
  loc_state_timer_.RecordAndVlogEveryN(10, 2);
  const auto loc_state_elapsed_time = loc_state_timer_.LastValue();
  if (loc_state_elapsed_time && *loc_state_elapsed_time >= 2) {
    LogError("LocalizationStateCallback: More than 2 seconds elapsed between loc state msgs.");
  }

  latest_combined_nav_state_ = lc::CombinedNavStateFromMsg(loc_msg);
  latest_covariances_ = lc::CombinedNavStateCovariancesFromMsg(loc_msg);
  latest_loc_msg_ = loc_msg;
  standstill_ = loc_msg.standstill;
}

bool ImuAugmentorWrapper::standstill() const {
  if (!params_.standstill_enabled) return false;
  // If uninitialized, return not at standstill
  // TODO(rsoussan): Is this the appropriate behavior?
  if (!standstill_) return false;
  return *standstill_;
}

void ImuAugmentorWrapper::ImuCallback(const sensor_msgs::Imu& imu_msg) {
  imu_augmentor_->BufferImuMeasurement(lm::ImuMeasurement(imu_msg));
}

void ImuAugmentorWrapper::FlightModeCallback(const ff_msgs::FlightMode& flight_mode) {
  imu_augmentor_->SetFanSpeedMode(lm::ConvertFanSpeedMode(flight_mode.speed));
}

boost::optional<std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>>
ImuAugmentorWrapper::LatestImuAugmentedCombinedNavStateAndCovariances() {
  if (!latest_combined_nav_state_ || !latest_covariances_ || !imu_augmentor_) {
    LogError(
      "LatestImuAugmentedCombinedNavStateAndCovariances: Not enough information available to create desired data.");
    return boost::none;
  }

  if (standstill()) {
    LogDebugEveryN(100, "LatestImuAugmentedCombinedNavStateAndCovariances: Standstill.");
    return std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>{*latest_combined_nav_state_,
                                                                            *latest_covariances_};
  }

  const auto latest_imu_augmented_combined_nav_state = imu_augmentor_->PimPredict(*latest_combined_nav_state_);
  if (!latest_imu_augmented_combined_nav_state) {
    LogError("LatestImuAugmentedCombinedNavSTateAndCovariances: Failed to pim predict combined nav state.");
    return boost::none;
  }
  // TODO(rsoussan): propogate uncertainties from imu augmentor
  return std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>{*latest_imu_augmented_combined_nav_state,
                                                                          *latest_covariances_};
}

boost::optional<ff_msgs::EkfState> ImuAugmentorWrapper::LatestImuAugmentedLocalizationMsg() {
  if (!latest_loc_msg_) {
    LogDebugEveryN(50, "LatestImuAugmentedLocalizationMsg: No latest loc msg available.");
    return boost::none;
  }

  const auto latest_imu_augmented_combined_nav_state_and_covariances =
    LatestImuAugmentedCombinedNavStateAndCovariances();
  if (!latest_imu_augmented_combined_nav_state_and_covariances) {
    LogError("LatestImuAugmentedLocalizationMsg: Failed to get latest imu augmented nav state and covariances.");
    return boost::none;
  }

  // Get feature counts and other info from latest_loc_msg
  ff_msgs::EkfState latest_imu_augmented_loc_msg;
  latest_imu_augmented_loc_msg.header = latest_loc_msg_->header;
  latest_imu_augmented_loc_msg.child_frame_id = latest_loc_msg_->child_frame_id;
  latest_imu_augmented_loc_msg.confidence = latest_loc_msg_->confidence;
  // Prevent overflow of uin8_t
  latest_imu_augmented_loc_msg.of_count =
    latest_loc_msg_->num_of_factors <= 255 ? latest_loc_msg_->num_of_factors : 255;
  latest_imu_augmented_loc_msg.ml_count =
    latest_loc_msg_->num_ml_projection_factors <= 255 ? latest_loc_msg_->num_ml_projection_factors : 255;
  latest_imu_augmented_loc_msg.estimating_bias = latest_loc_msg_->estimating_bias;

  // Update nav state and covariances with latest imu measurements
  lc::CombinedNavStateToMsg(latest_imu_augmented_combined_nav_state_and_covariances->first,
                            latest_imu_augmented_loc_msg);
  lc::CombinedNavStateCovariancesToMsg(latest_imu_augmented_combined_nav_state_and_covariances->second,
                                       latest_imu_augmented_loc_msg);

  // Add latest bias corrected acceleration and angular velocity to loc msg
  const auto latest_imu_measurement = imu_augmentor_->LatestMeasurement();
  if (!latest_imu_measurement) {
    LogError("LatestImuAugmentedLocalizationMsg: Failed to get latest measurement.");
    return boost::none;
  }
  const auto& latest_bias = latest_imu_augmented_combined_nav_state_and_covariances->first.bias();
  auto latest_bias_corrected_acceleration = latest_bias.correctAccelerometer(latest_imu_measurement->acceleration);
  const auto latest_bias_corrected_angular_velocity =
    latest_bias.correctGyroscope(latest_imu_measurement->angular_velocity);
  // Correct for gravity if needed
  if (!params_.gravity.isZero()) {
    const gtsam::Pose3& global_T_body_latest = latest_imu_augmented_combined_nav_state_and_covariances->first.pose();
    latest_bias_corrected_acceleration = lc::RemoveGravityFromAccelerometerMeasurement(
      params_.gravity, params_.body_T_imu, global_T_body_latest, latest_bias_corrected_acceleration);
  }
  // Frame change measurements to body frame, correct for centripetal accel
  const auto corrected_measurements = preintegration_helper_->correctMeasurementsBySensorPose(
    latest_bias_corrected_acceleration, latest_bias_corrected_angular_velocity);

  mc::VectorToMsg(corrected_measurements.first, latest_imu_augmented_loc_msg.accel);
  mc::VectorToMsg(corrected_measurements.second, latest_imu_augmented_loc_msg.omega);
  return latest_imu_augmented_loc_msg;
}
}  // namespace imu_augmentor
