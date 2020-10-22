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

#include <graph_bag/imu_bias_tester.h>
#include <imu_integration/utilities.h>

#include <glog/logging.h>

namespace graph_bag {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
ImuBiasTester::ImuBiasTester(const ii::ImuIntegratorParams& params) : ii::ImuIntegrator(params), initialized_(false) {}

std::vector<lc::CombinedNavState> ImuBiasTester::PimPredict(const lc::CombinedNavState& combined_nav_state) {
  combined_nav_states_.emplace(combined_nav_state.timestamp(), combined_nav_state);
  if (Empty()) return {};
  RemoveOldCombinedNavStatesIfNeeded();
  std::vector<lc::CombinedNavState> predicted_states;
  // Wait until imu measurements are newer than upper bound nav state to avoid prematurely integrating
  // measurements with an old nav state bias
  while (combined_nav_states_.size() >= 2) {
    auto lower_bound_state_it = combined_nav_states_.begin();
    const auto upper_bound_timestamp = std::next(lower_bound_state_it)->first;
    if (LatestTime() < upper_bound_timestamp) break;
    if (!Initialized()) Initialize(lower_bound_state_it->second);
    IntegrateAndRemoveMeasurements(lower_bound_state_it->second, upper_bound_timestamp, predicted_states);
    combined_nav_states_.erase(lower_bound_state_it);
  }
  return predicted_states;
}

void ImuBiasTester::IntegrateAndRemoveMeasurements(const lc::CombinedNavState& lower_bound_state,
                                                   const lc::Time& upper_bound_timestamp,
                                                   std::vector<lc::CombinedNavState>& predicted_states) {
  RemoveOldMeasurements(lower_bound_state.timestamp());
  auto pim = ii::Pim(lower_bound_state.bias(), pim_params());
  // Reset pim after each integration since pim uses beginning orientation and velocity for
  // gravity integration and initial velocity integration.
  for (auto measurement_it = measurements().cbegin();
       measurement_it != measurements().cend() && measurement_it->first < upper_bound_timestamp; ++measurement_it) {
    pim.resetIntegrationAndSetBias(lower_bound_state.bias());
    auto time = last_predicted_combined_nav_state_->timestamp();
    ii::AddMeasurement(measurement_it->second, time, pim);
    last_predicted_combined_nav_state_ = ii::PimPredict(*last_predicted_combined_nav_state_, pim);
    predicted_states.emplace_back(*last_predicted_combined_nav_state_);
  }
  RemoveOldMeasurements(upper_bound_timestamp);
}

bool ImuBiasTester::Initialized() { return initialized_; }
void ImuBiasTester::Initialize(const lc::CombinedNavState& combined_nav_state) {
  last_predicted_combined_nav_state_ = combined_nav_state;
  initialized_ = true;
}

void ImuBiasTester::RemoveOldCombinedNavStatesIfNeeded() {
  if (Empty()) return;
  const auto oldest_imu_time = *OldestTime();
  auto oldest_state_it = combined_nav_states_.begin();
  // Remove nav states older than oldest imu measurement as long as they aren't the lower bound
  while (combined_nav_states_.size() >= 2 && oldest_state_it->first < oldest_imu_time &&
         std::next(oldest_state_it)->first < oldest_imu_time) {
    oldest_state_it = combined_nav_states_.erase(oldest_state_it);
  }
}
}  // namespace graph_bag
