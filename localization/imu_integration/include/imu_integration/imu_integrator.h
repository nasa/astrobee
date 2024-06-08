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

#ifndef IMU_INTEGRATION_IMU_INTEGRATOR_H_
#define IMU_INTEGRATION_IMU_INTEGRATOR_H_

#include <imu_integration/dynamic_imu_filter.h>
#include <imu_integration/imu_integrator_params.h>

#include <localization_common/combined_nav_state.h>
#include <localization_common/time.h>
#include <localization_common/timestamped_set.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/imu_measurement.h>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include <map>

namespace imu_integration {
// Integrates imu measurements and propagates uncertainties.
// Maintains a window of measurements so that any interval of measurements in
// that window can be integrated into a pim.
class ImuIntegrator : public localization_common::TimestampedSet<localization_measurements::ImuMeasurement> {
 public:
  explicit ImuIntegrator(const ImuIntegratorParams& params = ImuIntegratorParams());

  // Buffers an imu measurement.
  void AddImuMeasurement(const localization_measurements::ImuMeasurement& imu_measurement);

  // Creates an integrated GTSAM Pim using the provided bias and imu measurements
  // between the provided start and end times.
  boost::optional<gtsam::PreintegratedCombinedMeasurements> IntegratedPim(
    const gtsam::imuBias::ConstantBias& bias, const localization_common::Time start_time,
    const localization_common::Time end_time) const;

  // Extrapolates the provided combined nav state using imu measurements up to the provided
  // end time.
  boost::optional<localization_common::CombinedNavState> Extrapolate(
    const localization_common::CombinedNavState& combined_nav_state, const localization_common::Time end_time) const;

  // Extrapolates using all IMU measurements more recent than the combined nav state
  boost::optional<localization_common::CombinedNavState> ExtrapolateLatest(
    const localization_common::CombinedNavState& combined_nav_state) const;

  void SetFanSpeedMode(const localization_measurements::FanSpeedMode fan_speed_mode);

  localization_measurements::FanSpeedMode fan_speed_mode() const;

 private:
  // Returns last integrated measurement timestamp.
  boost::optional<localization_common::Time> IntegrateImuMeasurements(
    const localization_common::Time start_time, const localization_common::Time end_time,
    gtsam::PreintegratedCombinedMeasurements& pim) const;

  ImuIntegratorParams params_;
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> pim_params_;
  std::unique_ptr<DynamicImuFilter> imu_filter_;
};
}  // namespace imu_integration

#endif  // IMU_INTEGRATION_IMU_INTEGRATOR_H_
