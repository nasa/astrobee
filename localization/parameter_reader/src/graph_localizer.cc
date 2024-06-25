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

#include <parameter_reader/factor_adders.h>
#include <parameter_reader/graph_localizer.h>
#include <parameter_reader/node_adders.h>
#include <parameter_reader/optimizers.h>
#include <parameter_reader/sliding_window_graph_optimizer.h>
#include <msg_conversions/msg_conversions.h>

namespace parameter_reader {
namespace gl = graph_localizer;
namespace mc = msg_conversions;

void LoadGraphLocalizerParams(config_reader::ConfigReader& config, gl::GraphLocalizerParams& params,
                              const std::string& prefix) {
  LoadLocFactorAdderParams(config, params.ar_tag_loc_factor_adder, prefix + "gl_fa_loc_ar_", "dock");
  LoadLocFactorAdderParams(config, params.sparse_map_loc_factor_adder, prefix + "gl_fa_loc_sm_", "nav");
  LoadPoseNodeAdderParams(config, params.pose_node_adder, prefix + "gl_na_pose_");
  LoadTimestampedNodeAdderModelParams(config, params.pose_node_adder_model, prefix + "gl_na_pose_model_");
  LoadNonlinearOptimizerParams(config, params.nonlinear_optimizer, prefix + "gl_op_nl_");
  LoadSlidingWindowGraphOptimizerParams(config, params.sliding_window_graph_optimizer, prefix + "gl_go_sw_");
  LOAD_PARAM(params.max_vio_measurement_gap, config, prefix + "gl_");
}
}  // namespace parameter_reader
