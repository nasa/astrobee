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

#include <parameter_reader/optimizers.h>
#include <msg_conversions/msg_conversions.h>

namespace parameter_reader {
namespace mc = msg_conversions;
namespace op = optimizers;

void LoadOptimizerParams(config_reader::ConfigReader& config, op::OptimizerParams& params, const std::string& prefix) {
  LOAD_PARAM(params.marginals_factorization, config, prefix);
}

void LoadISAM2OptimizerParams(config_reader::ConfigReader& config, op::ISAM2OptimizerParams& params,
                              const std::string& prefix) {
  LoadOptimizerParams(config, params, prefix);
}

void LoadNonlinearOptimizerParams(config_reader::ConfigReader& config, op::NonlinearOptimizerParams& params,
                                  const std::string& prefix) {
  LoadOptimizerParams(config, params, prefix);
  LOAD_PARAM(params.max_iterations, config, prefix);
  LOAD_PARAM(params.verbose, config, prefix);
  LOAD_PARAM(params.use_ceres_params, config, prefix);
}
}  // namespace parameter_reader
