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

#include <imu_integration/parameter_reader.h>
#include <imu_integration/butterO1.h>
#include <imu_integration/butterO3.h>
#include <imu_integration/butterO5.h>
#include <imu_integration/butterO7.h>
#include <imu_integration/butterO10.h>
#include <imu_integration/identity_filter.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

namespace imu_integration {
namespace lc = localization_common;
namespace mc = msg_conversions;
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
  } else if (filter_type == "ButterO1S125Lp3N33_33") {  // 125Hz 1st Order
    return std::unique_ptr<Filter>(new ButterO1S125Lp3N33_33());
  } else if (filter_type == "ButterO1S125Lp3N41_66") {
    return std::unique_ptr<Filter>(new ButterO1S125Lp3N41_66());
  } else if (filter_type == "ButterO1S125Lp3N46_66") {
    return std::unique_ptr<Filter>(new ButterO1S125Lp3N46_66());
  } else if (filter_type == "ButterO3S62_5Lp3N29_16") {  // 3rd Order
    return std::unique_ptr<Filter>(new ButterO3S62_5Lp3N29_16());
  } else if (filter_type == "ButterO3S62_5Lp3N20_83") {
    return std::unique_ptr<Filter>(new ButterO3S62_5Lp3N20_83());
  } else if (filter_type == "ButterO3S62_5Lp3N15_83") {
    return std::unique_ptr<Filter>(new ButterO3S62_5Lp3N15_83());
  } else if (filter_type == "ButterO3S125Lp3N33_33") {  // 125Hz 3rd Order
    return std::unique_ptr<Filter>(new ButterO3S125Lp3N33_33());
  } else if (filter_type == "ButterO3S125Lp3N41_66") {
    return std::unique_ptr<Filter>(new ButterO3S125Lp3N41_66());
  } else if (filter_type == "ButterO3S125Lp3N46_66") {
    return std::unique_ptr<Filter>(new ButterO3S125Lp3N46_66());
  } else if (filter_type == "ButterO5S62_5Lp3N29_16") {  // 5th Order
    return std::unique_ptr<Filter>(new ButterO5S62_5Lp3N29_16());
  } else if (filter_type == "ButterO5S62_5Lp3N20_83") {
    return std::unique_ptr<Filter>(new ButterO5S62_5Lp3N20_83());
  } else if (filter_type == "ButterO5S62_5Lp3N15_83") {
    return std::unique_ptr<Filter>(new ButterO5S62_5Lp3N15_83());
  } else if (filter_type == "ButterO5S125Lp3N41_66") {  // 125Hz
    return std::unique_ptr<Filter>(new ButterO5S125Lp3N41_66());
  } else if (filter_type == "ButterO5S125Lp3N46_66") {
    return std::unique_ptr<Filter>(new ButterO5S125Lp3N46_66());
  } else if (filter_type == "ButterO5S125Lp3N33_33") {
    return std::unique_ptr<Filter>(new ButterO5S125Lp3N33_33());
  } else if (filter_type == "ButterO7S62_5Lp3N20_83") {  // 7th Order
    return std::unique_ptr<Filter>(new ButterO7S62_5Lp3N20_83());
  } else if (filter_type == "ButterO10S62_5Lp3N20_83") {  // 10th Order
    return std::unique_ptr<Filter>(new ButterO10S62_5Lp3N20_83());
  } else if (filter_type == "ButterO5S62_5Lp1N29_16") {  // Lower pass band
    return std::unique_ptr<Filter>(new ButterO5S62_5Lp1N29_16());
  } else if (filter_type == "ButterO5S62_5Lp0_5N29_16") {
    return std::unique_ptr<Filter>(new ButterO5S62_5Lp0_5N29_16());
  } else if (filter_type == "none") {
    return std::unique_ptr<Filter>(new IdentityFilter());
  } else {
    LogFatal("LoadFilter: Invalid filter selection.");
    return std::unique_ptr<Filter>(new IdentityFilter());
  }
}
}  // namespace imu_integration
