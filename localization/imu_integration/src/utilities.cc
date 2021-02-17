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

#include <imu_integration/utilities.h>
#include <imu_integration/butterO1.h>
#include <imu_integration/butterO3.h>
#include <imu_integration/butterO5.h>
#include <imu_integration/butterworth_lowpass_filter_5th_order_05.h>
#include <imu_integration/butterworth_lowpass_filter_5th_order_1.h>
#include <imu_integration/identity_filter.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

namespace imu_integration {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

boost::optional<lm::ImuMeasurement> Interpolate(const lm::ImuMeasurement& imu_measurement_a,
                                                const lm::ImuMeasurement& imu_measurement_b, const lc::Time timestamp) {
  if (timestamp < imu_measurement_a.timestamp || timestamp > imu_measurement_b.timestamp) {
    LogError(
      "Interpolate: Interpolation timestamp out of range of imu "
      "measurements.");
    return boost::none;
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
  const gtsam::imuBias::ConstantBias& bias,
  const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>& params) {
  gtsam::PreintegratedCombinedMeasurements pim(params);
  pim.resetIntegrationAndSetBias(bias);
  return pim;
}

void AddMeasurement(const lm::ImuMeasurement& imu_measurement, lc::Time& last_added_imu_measurement_time,
                    gtsam::PreintegratedCombinedMeasurements& pim) {
  const double dt = imu_measurement.timestamp - last_added_imu_measurement_time;
  // TODO(rsoussan): check if dt too large? Pass threshold param?
  if (dt == 0) {
    LogError("AddMeasurement: Timestamp difference 0, failed to add measurement.");
    return;
  }
  pim.integrateMeasurement(imu_measurement.acceleration, imu_measurement.angular_velocity, dt);
  last_added_imu_measurement_time = imu_measurement.timestamp;
}

lc::CombinedNavState PimPredict(const lc::CombinedNavState& combined_nav_state,
                                const gtsam::PreintegratedCombinedMeasurements& pim) {
  const gtsam::NavState predicted_nav_state = pim.predict(combined_nav_state.nav_state(), pim.biasHat());
  const lc::Time timestamp = combined_nav_state.timestamp() + pim.deltaTij();
  return lc::CombinedNavState(predicted_nav_state, pim.biasHat(), timestamp);
}

gtsam::CombinedImuFactor::shared_ptr MakeCombinedImuFactor(const int key_index_0, const int key_index_1,
                                                           const gtsam::PreintegratedCombinedMeasurements& pim) {
  return gtsam::CombinedImuFactor::shared_ptr(
    new gtsam::CombinedImuFactor(sym::P(key_index_0), sym::V(key_index_0), sym::P(key_index_1), sym::V(key_index_1),
                                 sym::B(key_index_0), sym::B(key_index_1), pim));
}

void LoadImuIntegratorParams(config_reader::ConfigReader& config, ImuIntegratorParams& params) {
  params.gravity = lc::LoadVector3(config, "world_gravity_vector");
  const bool ignore_gravity = mc::LoadBool(config, "ignore_gravity");
  if (ignore_gravity) params.gravity = gtsam::Vector3::Zero();
  params.body_T_imu = lc::LoadTransform(config, "imu_transform");
  LoadImuFilterParams(config, params.filter);
  params.gyro_sigma = mc::LoadDouble(config, "gyro_sigma");
  params.accel_sigma = mc::LoadDouble(config, "accel_sigma");
  params.accel_bias_sigma = mc::LoadDouble(config, "accel_bias_sigma");
  params.gyro_bias_sigma = mc::LoadDouble(config, "gyro_bias_sigma");
  params.integration_variance = mc::LoadDouble(config, "integration_variance");
  params.bias_acc_omega_int = mc::LoadDouble(config, "bias_acc_omega_int");
}

void LoadImuFilterParams(config_reader::ConfigReader& config, ImuFilterParams& params) {
  params.quiet_accel = mc::LoadString(config, "imu_filter_quiet_accel");
  params.quiet_ang_vel = mc::LoadString(config, "imu_filter_quiet_ang_vel");
  params.nominal_accel = mc::LoadString(config, "imu_filter_nominal_accel");
  params.nominal_ang_vel = mc::LoadString(config, "imu_filter_nominal_ang_vel");
  params.aggressive_accel = mc::LoadString(config, "imu_filter_aggressive_accel");
  params.aggressive_ang_vel = mc::LoadString(config, "imu_filter_aggressive_ang_vel");
}

std::unique_ptr<Filter> LoadFilter(const std::string& filter_type) {
  // 1st Order
  if (filter_type == "ButterO1S62_5Lp3N29_16") {
    return std::unique_ptr<Filter>(new ButterO1S62_5Lp3N29_16());
  } else if (filter_type == "ButterO1S62_5Lp3N20_83") {
    return std::unique_ptr<Filter>(new ButterO1S62_5Lp3N20_83());
  } else if (filter_type == "ButterO1S62_5Lp3N15_83") {
    return std::unique_ptr<Filter>(new ButterO1S62_5Lp3N15_83());
  } else if (filter_type == "ButterO3S62_5Lp3N29_16") {  // 3rd Order
    return std::unique_ptr<Filter>(new ButterO3S62_5Lp3N29_16());
  } else if (filter_type == "ButterO5S62_5Lp3N29_16") {  // 5th Order
    return std::unique_ptr<Filter>(new ButterO5S62_5Lp3N29_16());
  } else if (filter_type == "butter5_1") {
    return std::unique_ptr<Filter>(new ButterworthLowpassFilter5thOrder1());
  } else if (filter_type == "butter5_05") {
    return std::unique_ptr<Filter>(new ButterworthLowpassFilter5thOrder05());
  } else if (filter_type == "none") {
    return std::unique_ptr<Filter>(new IdentityFilter());
  } else {
    LogFatal("LoadFilter: Invalid filter selection.");
    return std::unique_ptr<Filter>(new IdentityFilter());
  }
}
}  // namespace imu_integration
