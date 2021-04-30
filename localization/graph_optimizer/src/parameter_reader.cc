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

#include <graph_optimizer/parameter_reader.h>
#include <msg_conversions/msg_conversions.h>

namespace graph_optimizer {
namespace mc = msg_conversions;

void LoadGraphValuesParams(config_reader::ConfigReader& config, GraphValuesParams& params) {
  params.ideal_duration = mc::LoadDouble(config, "ideal_duration");
  params.min_num_states = mc::LoadInt(config, "min_num_states");
  params.max_num_states = mc::LoadInt(config, "max_num_states");
  params.min_num_factors_per_feature = mc::LoadInt(config, "min_num_factors_per_feature");
}

void LoadGraphOptimizerParams(config_reader::ConfigReader& config, GraphOptimizerParams& params) {
  LoadGraphValuesParams(config, params.graph_values);
  params.verbose = mc::LoadBool(config, "verbose");
  params.fatal_failures = mc::LoadBool(config, "fatal_failures");
  params.print_factor_info = mc::LoadBool(config, "print_factor_info");
  params.use_ceres_params = mc::LoadBool(config, "use_ceres_params");
  params.max_iterations = mc::LoadInt(config, "max_iterations");
  params.marginals_factorization = mc::LoadString(config, "marginals_factorization");
  params.add_marginal_factors = mc::LoadBool(config, "add_marginal_factors");
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.log_rate = mc::LoadInt(config, "log_rate");
}
}  // namespace graph_optimizer
