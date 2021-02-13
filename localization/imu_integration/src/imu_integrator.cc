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

void ImuIntegrator::BufferImuMeasurement(const lm::ImuMeasurement& imu_measurement) {
  const auto filtered_imu_measurement = imu_filter_.AddMeasurement(imu_measurement);
  if (filtered_imu_measurement) {
    // TODO(rsoussan): Prevent measurements_ from growing too large, add optional window size
    measurements_.emplace(filtered_imu_measurement->timestamp, *filtered_imu_measurement);
  }
}

boost::optional<lc::Time> ImuIntegrator::IntegrateImuMeasurements(const lc::Time start_time, const lc::Time end_time,
                                                                  gtsam::PreintegratedCombinedMeasurements& pim) const {
  if (measurements_.size() < 2) {
    LogError("IntegrateImuMeasurements: Less than 2 measurements available.");
    return boost::none;
  }
  if (end_time < measurements_.cbegin()->first) {
    LogError("IntegrateImuMeasurements: End time occurs before first measurement.");
    return boost::none;
  }
  if (end_time > measurements_.crbegin()->first) {
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
  auto measurement_it = measurements_.upper_bound(start_time);
  lc::Time last_added_imu_measurement_time = start_time;
  int num_measurements_added = 0;
  for (; measurement_it != measurements_.cend() && measurement_it->first <= end_time; ++measurement_it) {
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
    << measurements_.size());
  return last_added_imu_measurement_time;
}

void ImuIntegrator::RemoveOldMeasurements(const lc::Time new_start_time) {
  for (auto measurement_it = measurements_.cbegin();
       measurement_it != measurements_.cend() && measurement_it->first < new_start_time;) {
    measurement_it = measurements_.erase(measurement_it);
  }
}

boost::optional<gtsam::PreintegratedCombinedMeasurements> ImuIntegrator::IntegratedPim(
  const gtsam::imuBias::ConstantBias& bias, const lc::Time start_time, const lc::Time end_time,
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params) const {
  auto pim = Pim(bias, params);
  const auto last_integrated_measurement_time = IntegrateImuMeasurements(start_time, end_time, pim);
  if (!last_integrated_measurement_time) {
    LogError("IntegratedPim: Failed to integrate imu measurments.");
    return boost::none;
  }
  return pim;
}

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> ImuIntegrator::pim_params() const {
  // Make a copy so internal params aren't exposed.  Gtsam expects a point to a
  // non const type, so can't pass a pointer to const.
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> copied_params(
    new gtsam::PreintegratedCombinedMeasurements::Params(*pim_params_));
  return copied_params;
}

boost::optional<lc::Time> ImuIntegrator::OldestTime() const {
  if (Empty()) {
    LogError("OldestTime: No measurements available.");
    return boost::none;
  }
  return measurements_.cbegin()->first;
}

boost::optional<lc::Time> ImuIntegrator::LatestTime() const {
  if (Empty()) {
    LogError("LatestTime: No measurements available.");
    return boost::none;
  }
  return measurements_.crbegin()->first;
}

boost::optional<lm::ImuMeasurement> ImuIntegrator::LatestMeasurement() const {
  if (Empty()) {
    LogError("LatestTime: No measurements available.");
    return boost::none;
  }
  return measurements_.crbegin()->second;
}

bool ImuIntegrator::Empty() const { return measurements_.empty(); }

int ImuIntegrator::Size() const { return measurements_.size(); }

bool ImuIntegrator::WithinBounds(const localization_common::Time timestamp) {
  const auto oldest_time = OldestTime();
  const auto latest_time = LatestTime();
  if (!oldest_time || !latest_time) {
    LogError("WithinBounds: Failed to get time bounds.");
    return false;
  }
  return (timestamp >= *oldest_time && timestamp <= *latest_time);
}

const std::map<localization_common::Time, localization_measurements::ImuMeasurement>& ImuIntegrator::measurements()
  const {
  return measurements_;
}

}  // namespace imu_integration
