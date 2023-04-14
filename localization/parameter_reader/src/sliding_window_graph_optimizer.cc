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

#include <parameter_reader/graph_optimizer.h>
#include <parameter_reader/sliding_window_graph_optimizer.h>
#include <msg_conversions/msg_conversions.h>

namespace parameter_reader {
namespace go = graph_optimizer;
namespace mc = msg_conversions;
namespace sw = sliding_window_graph_optimizer;

void LoadSlidingWindowGraphOptimizerParams(config_reader::ConfigReader& config,
                                           sw::SlidingWindowGraphOptimizerParams& params, const std::string& prefix) {
  LoadGraphOptimizerParams(config, params, prefix);
  LOAD_PARAM(params.add_marginal_factors, config, prefix);
  LOAD_PARAM(params.slide_window_before_optimization, config, prefix);
}
}  // namespace parameter_reader
