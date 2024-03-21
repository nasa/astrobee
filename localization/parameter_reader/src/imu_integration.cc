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

#include <parameter_reader/imu_integration.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <string>

namespace parameter_reader {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace mc = msg_conversions;

void LoadImuFilterParams(config_reader::ConfigReader& config, ii::ImuFilterParams& params, const std::string& prefix) {
  LOAD_PARAM(params.quiet_accel, config, prefix + "if_");
  LOAD_PARAM(params.quiet_ang_vel, config, prefix + "if_");
  LOAD_PARAM(params.nominal_accel, config, prefix + "if_");
  LOAD_PARAM(params.nominal_ang_vel, config, prefix + "if_");
  LOAD_PARAM(params.aggressive_accel, config, prefix + "if_");
  LOAD_PARAM(params.aggressive_ang_vel, config, prefix + "if_");
}

void LoadImuIntegratorParams(config_reader::ConfigReader& config, ii::ImuIntegratorParams& params,
                             const std::string& prefix) {
  params.gravity = lc::LoadVector3(config, "world_gravity_vector", prefix);
  // Nullify gravity factor if ignore_gravity set to true.
  const bool ignore_gravity = mc::LoadBool(config, "ignore_gravity", prefix + "ii_");
  if (ignore_gravity) params.gravity = gtsam::Vector3::Zero();
  params.body_T_imu = lc::LoadTransform(config, "imu_transform");
  LoadImuFilterParams(config, params.filter);
  LOAD_PARAM(params.gyro_sigma, config, prefix + "ii_");
  LOAD_PARAM(params.accel_sigma, config, prefix + "ii_");
  LOAD_PARAM(params.accel_bias_sigma, config, prefix + "ii_");
  LOAD_PARAM(params.gyro_bias_sigma, config, prefix + "ii_");
  LOAD_PARAM(params.integration_variance, config, prefix + "ii_");
  LOAD_PARAM(params.bias_acc_omega_int, config, prefix + "ii_");
}
}  // namespace parameter_reader
