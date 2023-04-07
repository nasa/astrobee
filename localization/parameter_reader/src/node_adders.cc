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
#include <parameter_reader/node_adders.h>
#include <msg_conversions/msg_conversions.h>

namespace parameter_reader {
namespace lc = localization_common;
namespace mc = msg_conversions;
namespace na = node_adders;

void LoadTimestampedNodeAdderModelParams(config_reader::ConfigReader& config,
                                         na::TimestampedNodeAdderModelParams& params, const std::string& prefix) {
  LOAD_PARAM(params.huber_k, config, prefix);
}

void LoadCombinedNavStateNodeAdderModelParams(config_reader::ConfigReader& config,
                                              na::CombinedNavStateNodeAdderModelParams& params,
                                              const std::string& prefix) {
  LoadImuIntegratorParams(config, params.imu_integrator, prefix);
  LoadTimestampedNodeAdderModelParams(config, params, prefix);
}

void LoadPoseNodeAdderParams(config_reader::ConfigReader& config, na::PoseNodeAdderParams& params,
                              const std::string& prefix) {
// TODO(rsoussan): add prefix for these?
LOAD_PARAM(params.starting_prior_translation_stddev, config, prefix);
LOAD_PARAM(params.starting_prior_quaternion_stddev, config, prefix);
LoadBaseTimestampedNodeAdderParams<gtsam::Pose3>(config, params, prefix);
       params.start_node= lc::LoadTransform(config, "start_node", prefix);
       // Set noise and starting pose based on stddevs and loaded values
       params.Initialize();
}
}  // namespace parameter_reader
