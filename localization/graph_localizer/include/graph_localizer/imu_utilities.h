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

#ifndef GRAPH_LOCALIZER_IMU_UTILITIES_H_
#define GRAPH_LOCALIZER_IMU_UTILITIES_H_

#include <graph_localizer/graph_loc_initialization.h>
#include <localization_measurements/combined_nav_state.h>
#include <localization_measurements/imu_measurement.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include <Eigen/Core>

#include <vector>

namespace graph_localizer {
namespace sym = gtsam::symbol_shorthand;
void EstimateAndSetImuBiases(const localization_measurements::ImuMeasurement &imu_measurement,
                             const int num_imu_measurements_per_bias_estimate,
                             std::vector<localization_measurements::ImuMeasurement> &imu_bias_measurements,
                             GraphLocInitialization &graph_loc_initialization);

localization_measurements::ImuMeasurement Interpolate(
    const localization_measurements::ImuMeasurement &imu_measurement_a,
    const localization_measurements::ImuMeasurement &imu_measurement_b,
    const localization_measurements::Time timestamp);

gtsam::PreintegratedCombinedMeasurements Pim(
    const gtsam::imuBias::ConstantBias &bias,
    const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &params);

void AddMeasurement(const localization_measurements::ImuMeasurement &imu_measurement,
                    localization_measurements::Time &last_added_imu_measurement_time,
                    gtsam::PreintegratedCombinedMeasurements &pim);

localization_measurements::CombinedNavState PimPredict(
    const localization_measurements::CombinedNavState &combined_nav_state,
    const gtsam::PreintegratedCombinedMeasurements &pim);

gtsam::CombinedImuFactor::shared_ptr MakeCombinedImuFactor(const int key_index_0, const int key_index_1,
                                                           const gtsam::PreintegratedCombinedMeasurements &pim);
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_IMU_UTILITIES_H_
