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

#include "test_utilities.h"  // NOLINT
#include <imu_integration/test_utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

namespace node_adders {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;

PoseNodeAdderParams DefaultPoseNodeAdderParams() {
  PoseNodeAdderParams params;
  params.starting_prior_translation_stddev = 0.1;
  params.starting_prior_quaternion_stddev = 0.1;
  // Base
  params.start_node = gtsam::Pose3::identity();
  params.huber_k = 1.345;
  params.add_priors = true;
  params.starting_time = 0;
  // Only kept if there are at least min_num_states and not more than max_num_states
  params.ideal_duration = 5;
  params.min_num_states = 5;
  params.max_num_states = 20;
  params.SetStartNoiseModels();
  return params;
}

CombinedNavStateNodeAdderModelParams DefaultCombinedNavStateNodeAdderModelParams() {
  CombinedNavStateNodeAdderModelParams params;
  params.imu_integrator = ii::DefaultImuIntegratorParams();
  params.huber_k = 1.345;
  return params;
}
}  // namespace node_adders
