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

#include <imu_integration/imu_utilities.h>
#include <localization_measurements/measurement_conversions.h>

#include <glog/logging.h>

namespace imu_augmentor {
namespace ii = imu_integration;
namespace lm = localization_measurements;
// Initialize with zero bias and start time since these will be set when a combined_nav_state is passed to the augmentor
ImuAugmentor::ImuAugmentor(const Eigen::Isometry3d& body_T_imu, const Eigen::Vector3d& gravity)
    : imu_integrator_(body_T_imu, Eigen
                      : Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, gravity) {}

lm::CombinedNavState ImuAugmentor::PimPredict(const lm::CombinedNavState& combined_nav_state) {
  imu_integrator_.ResetLatestPimIntegrationAndSetBias(combined_nav_state.bias());
  imu_integrator_.IntegrateImuMeasurements(combined_nav_state.timestamp(), imu_integrator_.LatestTimestamp(),
                                           imu_integrator_.latest_pim());
  // integrate measurements from start time to end time

  return ii::PimPredict(combined_nav_state, imu_integrator_.latest_pim());
}
}  // namespace imu_augmentor
