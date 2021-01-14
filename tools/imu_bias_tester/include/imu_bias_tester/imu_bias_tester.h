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

#ifndef IMU_BIAS_TESTER_IMU_BIAS_TESTER_H_
#define IMU_BIAS_TESTER_IMU_BIAS_TESTER_H_

#include <imu_integration/imu_integrator.h>
#include <imu_integration/imu_integrator_params.h>
#include <localization_common/combined_nav_state.h>

#include <map>
#include <vector>

namespace imu_bias_tester {
class ImuBiasTester : public imu_integration::ImuIntegrator {
 public:
  explicit ImuBiasTester(const imu_integration::ImuIntegratorParams& params);

  std::vector<localization_common::CombinedNavState> PimPredict(
    const localization_common::CombinedNavState& combined_nav_state);

 private:
  // Integrates imu measurements between lower and upper bound nav states using lower bound's biases
  void IntegrateAndRemoveMeasurements(const localization_common::CombinedNavState& lower_bound_state,
                                      const localization_common::Time& upper_bound_timestamp,
                                      std::vector<localization_common::CombinedNavState>& predicted_states);
  void RemoveOldCombinedNavStatesIfNeeded();
  bool Initialized();
  void Initialize(const localization_common::CombinedNavState& combined_nav_state);

  std::map<localization_common::Time, localization_common::CombinedNavState> combined_nav_states_;
  bool initialized_;
  boost::optional<localization_common::CombinedNavState> last_predicted_combined_nav_state_;
};
}  // namespace imu_bias_tester

#endif  // IMU_BIAS_TESTER_IMU_BIAS_TESTER_H_
