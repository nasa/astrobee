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
#include <parameter_reader/ros_pose_extrapolator.h>
#include <msg_conversions/msg_conversions.h>

namespace parameter_reader {
namespace mc = msg_conversions;
namespace rp = ros_pose_extrapolator;

void LoadRosPoseExtrapolatorParams(config_reader::ConfigReader& config, rp::RosPoseExtrapolatorParams& params,
                              const std::string& prefix) {
  LoadImuIntegratorParams(config, params.imu_integrator, prefix);
  LOAD_PARAM(params.standstill_enabled, config, prefix);
}
}  // namespace parameter_reader
