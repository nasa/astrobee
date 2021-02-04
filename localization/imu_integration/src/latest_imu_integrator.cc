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

#include <imu_integration/latest_imu_integrator.h>
#include <localization_common/logger.h>

namespace imu_integration {
namespace lc = localization_common;
namespace lm = localization_measurements;
LatestImuIntegrator::LatestImuIntegrator(const LatestImuIntegratorParams& params)
    : ImuIntegrator(params), params_(params), last_added_imu_measurement_time_(0) {
  pim_.reset(new gtsam::PreintegratedCombinedMeasurements(pim_params()));
  ResetPimIntegrationAndSetBias(params_.initial_imu_bias);
}

const gtsam::PreintegratedCombinedMeasurements& LatestImuIntegrator::pim() const { return *pim_; }

void LatestImuIntegrator::ResetPimIntegrationAndSetBias(const gtsam::imuBias::ConstantBias& bias) {
  pim_->resetIntegrationAndSetBias(bias);
}

bool LatestImuIntegrator::IntegrateLatestImuMeasurements(const lc::Time end_time) {
  if (Size() < 2) {
    LogError(
      "IntegrateLatestImuMeasurements: Less than 2 measurements "
      "available.");
    return false;
  }

  if (last_added_imu_measurement_time_ == 0) {
    LogDebug("IntegrateLatestImuMeasurements: Adding first imu measurement.");
    last_added_imu_measurement_time_ = params_.start_time;
  }

  const auto last_added_imu_measurement_time =
    IntegrateImuMeasurements(last_added_imu_measurement_time_, end_time, *pim_);
  if (!last_added_imu_measurement_time) {
    LogError("IntegrateLatestImuMeasurements: Failed to integrate measurements.");
    return false;
  }

  last_added_imu_measurement_time_ = *last_added_imu_measurement_time;
  return true;
}
}  // namespace imu_integration
