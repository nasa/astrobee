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
#include <localization_common/utilities.h>
#include <localization_measurements/imu_measurement.h>
#include <msg_conversions/msg_conversions.h>

#include <glog/logging.h>

namespace imu_augmentor {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
ImuAugmentorWrapper::ImuAugmentorWrapper() {
  config_reader::ConfigReader config;
  config.AddFile("transforms.config");
  config.AddFile("geometry.config");

  if (!config.ReadFiles()) {
    LOG(FATAL) << "Failed to read config files.";
  }

  // TODO(rsoussan): Unify this with graph localizer param reader LoadImuIntegratorParams
  params_.body_T_imu = lc::LoadTransform(config, "imu_transform");
  params_.gravity = lc::LoadVector3(config, "world_gravity_vector");
  imu_augmentor_.reset(new ImuAugmentor(params_));

  // Preintegration_helper_ is only being used to frame change and remove centrifugal acceleration, so body_T_imu is the
  // only parameter needed.
  boost::shared_ptr<gtsam::PreintegrationParams> preintegration_params(new gtsam::PreintegrationParams());
  preintegration_params->setBodyPSensor(params_.body_T_imu);
  preintegration_helper_.reset(new gtsam::TangentPreintegration(preintegration_params, gtsam::imuBias::ConstantBias()));
}

void ImuAugmentorWrapper::LocalizationStateCallback(const ff_msgs::EkfState& loc_msg) {
  latest_combined_nav_state_.reset(new lc::CombinedNavState(lc::CombinedNavStateFromMsg(loc_msg)));
  latest_covariances_.reset(new lc::CombinedNavStateCovariances(lc::CombinedNavStateCovariancesFromMsg(loc_msg)));
  latest_loc_msg_.reset(new ff_msgs::EkfState(loc_msg));
}

void ImuAugmentorWrapper::ImuCallback(const sensor_msgs::Imu& imu_msg) {
  imu_augmentor_->BufferImuMeasurement(lm::ImuMeasurement(imu_msg));
}

boost::optional<std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>>
ImuAugmentorWrapper::LatestImuAugmentedCombinedNavStateAndCovariances() {
  if (!latest_combined_nav_state_ || !latest_covariances_ || !imu_augmentor_) {
    LOG(ERROR)
        << "LatestImuAugmentedCombinedNavStateAndCovariances: Not enough information available to create desired data.";
    return boost::none;
  }

  const auto latest_imu_augmented_combined_nav_state = imu_augmentor_->PimPredict(*latest_combined_nav_state_);
  if (!latest_imu_augmented_combined_nav_state) {
    LOG(ERROR) << "LatestImuAugmentedCombinedNavSTateAndCovariances: Failed to pim predict combined nav state.";
    return boost::none;
  }
  // TODO(rsoussan): propogate uncertainties from imu augmentor
  return std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>{*latest_imu_augmented_combined_nav_state,
                                                                          *latest_covariances_};
}

boost::optional<ff_msgs::EkfState> ImuAugmentorWrapper::LatestImuAugmentedLocalizationMsg() {
  if (!latest_loc_msg_) {
    LOG_EVERY_N(WARNING, 50) << "LatestImuAugmentedLocalizationMsg: No latest loc msg available.";
    return boost::none;
  }

  const auto latest_imu_augmented_combined_nav_state_and_covariances =
      LatestImuAugmentedCombinedNavStateAndCovariances();
  if (!latest_imu_augmented_combined_nav_state_and_covariances) {
    LOG(ERROR) << "LatestImuAugmentedLocalizationMsg: Failed to get latest imu augmented nav state and covariances.";
    return boost::none;
  }

  // Get feature counts and other info from latest_loc_msg
  auto latest_imu_augmented_loc_msg = *latest_loc_msg_;

  // Update nav state and covariances with latest imu measurements
  lc::CombinedNavStateToMsg(latest_imu_augmented_combined_nav_state_and_covariances->first,
                            latest_imu_augmented_loc_msg);
  lc::CombinedNavStateCovariancesToMsg(latest_imu_augmented_combined_nav_state_and_covariances->second,
                                       latest_imu_augmented_loc_msg);

  // Add latest bias corrected acceleration and angular velocity to loc msg
  const auto latest_imu_measurement = imu_augmentor_->LatestMeasurement();
  if (!latest_imu_measurement) {
    LOG(ERROR) << "LatestImuAugmentedLocalizationMsg: Failed to get latest measurement.";
    return boost::none;
  }
  const auto& latest_bias = latest_imu_augmented_combined_nav_state_and_covariances->first.bias();
  const auto latest_bias_corrected_acceleration =
      latest_bias.correctAccelerometer(latest_imu_measurement->acceleration);
  const auto latest_bias_corrected_angular_velocity =
      latest_bias.correctGyroscope(latest_imu_measurement->angular_velocity);
  // Frame change measurements to body frame, correct for centripetal accel, correct for gravity if needed
  auto corrected_measurements = preintegration_helper_->correctMeasurementsBySensorPose(
      latest_bias_corrected_acceleration, latest_bias_corrected_angular_velocity);
  if (!params_.gravity.isZero()) {
    const gtsam::Pose3& global_T_body_latest = latest_imu_augmented_combined_nav_state_and_covariances->first.pose();
    corrected_measurements.first = lc::RemoveGravityFromAccelerometerMeasurement(
        params_.gravity, params_.body_T_imu, global_T_body_latest, corrected_measurements.first);
  }

  lc::VectorToMsg(corrected_measurements.first, latest_imu_augmented_loc_msg.accel);
  lc::VectorToMsg(corrected_measurements.second, latest_imu_augmented_loc_msg.omega);
  return latest_imu_augmented_loc_msg;
}
}  // namespace imu_augmentor
