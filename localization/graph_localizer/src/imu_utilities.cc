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

#include <graph_localizer/imu_utilities.h>

#include <glog/logging.h>

namespace graph_localizer {
namespace lm = localization_measurements;
void EstimateAndSetImuBiases(const lm::ImuMeasurement &imu_measurement,
                             const int num_imu_measurements_per_bias_estimate,
                             std::vector<lm::ImuMeasurement> &imu_bias_measurements,
                             GraphLocInitialization &graph_loc_initialization) {
  imu_bias_measurements.emplace_back(imu_measurement);
  if (imu_bias_measurements.size() >= num_imu_measurements_per_bias_estimate) {
    Eigen::Vector3d sum_of_acceleration_measurements = Eigen::Vector3d::Zero();
    Eigen::Vector3d sum_of_angular_velocity_measurements = Eigen::Vector3d::Zero();
    for (const auto &imu_measurement : imu_bias_measurements) {
      sum_of_acceleration_measurements += imu_measurement.acceleration;
      sum_of_angular_velocity_measurements += imu_measurement.angular_velocity;
    }

    DLOG(INFO) << "Number of imu measurements per bias estimate: " << num_imu_measurements_per_bias_estimate;
    const Eigen::Vector3d accelerometer_bias = sum_of_acceleration_measurements / imu_bias_measurements.size();
    const Eigen::Vector3d gyro_bias = sum_of_angular_velocity_measurements / imu_bias_measurements.size();
    LOG(INFO) << "Accelerometer bias: " << std::endl << accelerometer_bias.matrix();
    LOG(INFO) << "Gyro bias: " << std::endl << gyro_bias.matrix();

    graph_loc_initialization.SetBiases(accelerometer_bias, gyro_bias);
    imu_bias_measurements.clear();
  }
}

lm::ImuMeasurement Interpolate(const lm::ImuMeasurement &imu_measurement_a, const lm::ImuMeasurement &imu_measurement_b,
                               const lm::Time timestamp) {
  if (timestamp < imu_measurement_a.timestamp || timestamp > imu_measurement_b.timestamp) {
    LOG(FATAL) << "Interpolate: Interpolation timestamp out of range of imu "
                  "measurements.";
  }

  const double alpha =
      (timestamp - imu_measurement_a.timestamp) / (imu_measurement_b.timestamp - imu_measurement_a.timestamp);
  const Eigen::Vector3d interpolated_acceleration =
      (1.0 - alpha) * imu_measurement_a.acceleration + alpha * imu_measurement_b.acceleration;
  const Eigen::Vector3d interpolated_angular_velocity =
      (1.0 - alpha) * imu_measurement_a.angular_velocity + alpha * imu_measurement_b.angular_velocity;

  return lm::ImuMeasurement(interpolated_acceleration, interpolated_angular_velocity, timestamp);
}

gtsam::PreintegratedCombinedMeasurements Pim(
    const gtsam::imuBias::ConstantBias &bias,
    const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &params) {
  gtsam::PreintegratedCombinedMeasurements pim(params);
  pim.resetIntegrationAndSetBias(bias);
  return pim;
}

void AddMeasurement(const lm::ImuMeasurement &imu_measurement, lm::Time &last_added_imu_measurement_time,
                    gtsam::PreintegratedCombinedMeasurements &pim) {
  // TODO(rsoussan): check if dt too large?

  const double dt = imu_measurement.timestamp - last_added_imu_measurement_time;
  pim.integrateMeasurement(imu_measurement.acceleration, imu_measurement.angular_velocity, dt);
  last_added_imu_measurement_time = imu_measurement.timestamp;
  // DLOG(INFO) << "deltaPij: " << pim_->deltaPij().matrix();
  // DLOG(INFO) << "deltaVij: " << pim_->deltaVij().matrix();
  // DLOG(INFO) << "deltaRij: " << pim_->deltaRij().matrix();
}

lm::CombinedNavState PimPredict(const lm::CombinedNavState &combined_nav_state,
                                const gtsam::PreintegratedCombinedMeasurements &pim) {
  const gtsam::NavState predicted_nav_state = pim.predict(combined_nav_state.nav_state(), pim.biasHat());
  const lm::Time timestamp = combined_nav_state.timestamp() + pim.deltaTij();
  return lm::CombinedNavState(predicted_nav_state, pim.biasHat(), timestamp);
}

gtsam::CombinedImuFactor::shared_ptr MakeCombinedImuFactor(const int key_index_0, const int key_index_1,
                                                           const gtsam::PreintegratedCombinedMeasurements &pim) {
  return gtsam::CombinedImuFactor::shared_ptr(
      new gtsam::CombinedImuFactor(sym::P(key_index_0), sym::V(key_index_0), sym::P(key_index_1), sym::V(key_index_1),
                                   sym::B(key_index_0), sym::B(key_index_1), pim));
}
}  // namespace graph_localizer
