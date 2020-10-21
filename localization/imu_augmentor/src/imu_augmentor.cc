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
ImuAugmentor::ImuAugmentor(const ii::ImuIntegratorParams& params) : ii::ImuIntegrator(params) {}

boost::optional<lc::CombinedNavState> ImuAugmentor::PimPredict(const lc::CombinedNavState& combined_nav_state) {
  if (Empty()) return combined_nav_state;
  // Start with least upper bound measurement
  // Don't add measurements with same timestamp as start_time
  // since these would have a dt of 0 (wrt the pim start time) and cause errors for the pim
  auto measurement_it = measurements().upper_bound(combined_nav_state.timestamp());
  if (measurement_it == measurements().cend()) return combined_nav_state;
  auto last_predicted_combined_nav_state = combined_nav_state;
  auto pim = ii::Pim(last_predicted_combined_nav_state.bias(), pim_params());
  int num_measurements_added = 0;
  // Create new pim each time since pim uses beginning orientation and velocity for
  // gravity integration and initial velocity integration.
  for (; measurement_it != measurements().cend(); ++measurement_it) {
    pim.resetIntegrationAndSetBias(last_predicted_combined_nav_state.bias());
    auto time = last_predicted_combined_nav_state.timestamp();
    ii::AddMeasurement(measurement_it->second, time, pim);
    last_predicted_combined_nav_state = ii::PimPredict(last_predicted_combined_nav_state, pim);
    ++num_measurements_added;
  }

  RemoveOldMeasurements(combined_nav_state.timestamp());
  VLOG(2) << "PimPredict: Added " << num_measurements_added << " measurements.";
  return last_predicted_combined_nav_state;
}
}  // namespace imu_augmentor
