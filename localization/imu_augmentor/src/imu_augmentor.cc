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

#include <imu_augmentor/imu_augmentor.h>
#include <imu_integration/utilities.h>

#include <glog/logging.h>

namespace imu_augmentor {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
ImuAugmentor::ImuAugmentor(const ImuIntegratorParams& params) : imu_integrator_(params) {}

void ImuAugmentor::BufferImuMeasurement(const lm::ImuMeasurement& imu_measurement) {
  imu_integrator_.BufferImuMeasurement(imu_measurement);
}

boost::optional<lc::CombinedNavState> ImuAugmentor::PimPredict(const lc::CombinedNavState& combined_nav_state) {
  const auto latest_time = imu_integrator_.LatestTime();
  if (!latest_time) {
    LOG(ERROR) << "PimPredict: Failed to get latest imu time.";
    return boost::none;
  }
  const auto pim = imu_integrator_.IntegratedPim(combined_nav_state.bias(), combined_nav_state.timestamp(),
                                                 *latest_time, imu_integrator_.pim_params());
  if (!pim) {
    LOG(ERROR) << "PimPredict: Failed to create integrated pim.";
    return boost::none;
  }

  const auto latest_combined_nav_state = ii::PimPredict(combined_nav_state, *pim);
  RemoveOldMeasurements(combined_nav_state.timestamp());
  return latest_combined_nav_state;
}

void ImuAugmentor::RemoveOldMeasurements(const lc::Time new_start_time) {
  imu_integrator_.RemoveOldMeasurements(new_start_time);
}

boost::optional<lm::ImuMeasurement> ImuAugmentor::LatestMeasurement() const {
  return imu_integrator_.LatestMeasurement();
}
}  // namespace imu_augmentor
