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

#include <imu_integration/imu_integrator.h>
#include <imu_integration/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

namespace imu_integration {
namespace lc = localization_common;
namespace lm = localization_measurements;
ImuIntegrator::ImuIntegrator(const ImuIntegratorParams& params) : params_(params) {
  imu_filter_.reset(new DynamicImuFilter(params_.filter));
  LogDebug("ImuIntegrator: Gravity vector: " << std::endl << params_.gravity.matrix());
  pim_params_.reset(new gtsam::PreintegratedCombinedMeasurements::Params(params_.gravity));
  // Set sensor covariances
  pim_params_->gyroscopeCovariance = params_.gyro_sigma * params_.gyro_sigma * gtsam::I_3x3;
  pim_params_->accelerometerCovariance = params_.accel_sigma * params_.accel_sigma * gtsam::I_3x3;
  pim_params_->integrationCovariance = params_.integration_variance * gtsam::I_3x3;
  // Set bias random walk covariances
  pim_params_->biasAccCovariance = params_.accel_bias_sigma * params_.accel_bias_sigma * gtsam::I_3x3;
  pim_params_->biasOmegaCovariance = params_.gyro_bias_sigma * params_.gyro_bias_sigma * gtsam::I_3x3;
  // Set bias covariance used for pim integration
  pim_params_->biasAccOmegaInt = params_.bias_acc_omega_int * gtsam::I_6x6;
  // Set imu calibration relative pose
  pim_params_->setBodyPSensor(params_.body_T_imu);
}

void ImuIntegrator::AddImuMeasurement(const lm::ImuMeasurement& imu_measurement) {
  const auto filtered_imu_measurement = imu_filter_->AddMeasurement(imu_measurement);
  if (filtered_imu_measurement) {
    Add(filtered_imu_measurement->timestamp, *filtered_imu_measurement);
  }
}

boost::optional<lc::Time> ImuIntegrator::IntegrateImuMeasurements(const lc::Time start_time, const lc::Time end_time,
                                                                  gtsam::PreintegratedCombinedMeasurements& pim) const {
  if (size() < 2) {
    LogError("IntegrateImuMeasurements: Less than 2 measurements available.");
    return boost::none;
  }
  if (end_time < *OldestTimestamp()) {
    LogError("IntegrateImuMeasurements: End time occurs before first measurement.");
    return boost::none;
  }
  if (end_time > *LatestTimestamp()) {
    LogError("IntegrateImuMeasurements: End time occurs after last measurement.");
    return boost::none;
  }
  if (start_time > end_time) {
    LogError("IntegrateImuMeasurements: Start time occurs after end time.");
    return boost::none;
  }

  // Start with least upper bound measurement
  // Don't add measurements with same timestamp as start_time
  // since these would have a dt of 0 (wrt the pim start time) and cause errors for the pim
  auto measurement_it = set().upper_bound(start_time);
  lc::Time last_added_imu_measurement_time = start_time;
  int num_measurements_added = 0;
  for (; measurement_it != set().cend() && measurement_it->first <= end_time; ++measurement_it) {
    AddMeasurement(measurement_it->second, last_added_imu_measurement_time, pim);
    ++num_measurements_added;
  }

  // Add final interpolated measurement if necessary
  if (last_added_imu_measurement_time != end_time) {
    const auto interpolated_measurement =
      Interpolate(std::prev(measurement_it)->second, measurement_it->second, end_time);
    if (!interpolated_measurement) {
      LogError("IntegrateImuMeasurements: Failed to interpolate final measurement.");
      return boost::none;
    }
    AddMeasurement(*interpolated_measurement, last_added_imu_measurement_time, pim);
    ++num_measurements_added;
  }

  if (last_added_imu_measurement_time != end_time) {
    LogError("IntegrateImuMeasurements: Last added time not equal to end time.");
    return boost::none;
  }

  LogDebug("IntegrateImuMeasurements: Num imu measurements integrated: " << num_measurements_added);
  LogDebug(
    "IntegrateImuMeasurements: Total Num Imu Measurements after "
    "integrating: "
    << size());
  return last_added_imu_measurement_time;
}

boost::optional<gtsam::PreintegratedCombinedMeasurements> ImuIntegrator::IntegratedPim(
  const gtsam::imuBias::ConstantBias& bias, const lc::Time start_time, const lc::Time end_time) const {
  auto pim = Pim(bias, pim_params_);
  const auto last_integrated_measurement_time = IntegrateImuMeasurements(start_time, end_time, pim);
  if (!last_integrated_measurement_time) {
    LogError("IntegratedPim: Failed to integrate imu measurments.");
    return boost::none;
  }
  return pim;
}

boost::optional<lc::CombinedNavState> ImuIntegrator::Extrapolate(const lc::CombinedNavState& combined_nav_state,
                                                                 const lc::Time end_time) const {
  const auto pim = IntegratedPim(combined_nav_state.bias(), combined_nav_state.timestamp(), end_time);
  if (!pim) {
    LogError("Extrapolate: Failed to get pim.");
    return boost::none;
  }

  return PimPredict(combined_nav_state, *pim);
}

boost::optional<localization_common::CombinedNavState> ImuIntegrator::ExtrapolateLatest(
  const localization_common::CombinedNavState& combined_nav_state) const {
  const auto latest = Latest();
  if (!latest) {
    LogError("ExtrapolateLatest: Failed to get latest measurement.");
    return boost::none;
  }

  const auto extrapolated_combined_nav_state = Extrapolate(combined_nav_state, latest->timestamp);
  if (!extrapolated_combined_nav_state) {
    LogError("ExtrapolateLatest: Failed to extrapolate combined nav state.");
    return boost::none;
  }

  return *extrapolated_combined_nav_state;
}

void ImuIntegrator::SetFanSpeedMode(const lm::FanSpeedMode fan_speed_mode) {
  imu_filter_->SetFanSpeedMode(fan_speed_mode);
}

lm::FanSpeedMode ImuIntegrator::fan_speed_mode() const { return imu_filter_->fan_speed_mode(); }
}  // namespace imu_integration
