/* Copyright (c) 2017, United States Government, as represented
 * by the Administrator of the National Aeronautics and Space
 * Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License,
 * Version 2.0 (the "License"); you may not use this file except
 * in compliance with the License. You may obtain a copy of the
 * License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the
 * License.
 */

#ifndef TUTORIAL_EXAMPLES_SIMPLE_LOCALIZER_PARAMS_H_
#define TUTORIAL_EXAMPLES_SIMPLE_LOCALIZER_PARAMS_H_

#include <factor_adders/pose_factor_adder.h>
#include <node_adders/pose_node_adder.h>
#include <node_adders/pose_node_adder_model.h>
#include <node_adders/pose_node_adder_params.h>
#include <optimizers/nonlinear_optimizer_params.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer_params.h>

namespace tutorial_examples {
struct SimpleLocalizerParams {
  // Initialize params with some default values.
  // For more details on the meaning of each param, see the
  // respective header file.
  SimpleLocalizerParams() {
    // Factor Adder
    pose_factor_adder.enabled = true;
    pose_factor_adder.huber_k = 1.345;
    // Node Adder
    pose_node_adder.huber_k = 1.345;
    pose_node_adder.add_priors = true;
    pose_node_adder.starting_time = 0.0;
    pose_node_adder.ideal_duration = 10.0;
    pose_node_adder.min_num_states = 2;
    pose_node_adder.max_num_states = 20;
    // Node Adder Model
    pose_node_adder_model.huber_k = 1.345;
    // Nonlinear Optimizer
    nonlinear_optimizer.marginals_factorization = "qr";
    nonlinear_optimizer.max_iterations = 10;
    nonlinear_optimizer.verbose = false;
    nonlinear_optimizer.use_ceres_params = false;
    // Sliding Window Graph Optimizer
    sliding_window_graph_optimizer.huber_k = 1.345;
    sliding_window_graph_optimizer.log_stats_on_destruction =
      false;
    sliding_window_graph_optimizer.print_after_optimization =
      false;
    sliding_window_graph_optimizer.add_marginal_factors = false;
    sliding_window_graph_optimizer
      .slide_window_before_optimization = true;
  }

  factor_adders::PoseFactorAdderParams pose_factor_adder;
  node_adders::PoseNodeAdderParams pose_node_adder;
  node_adders::PoseNodeAdderModel::Params pose_node_adder_model;
  optimizers::NonlinearOptimizerParams nonlinear_optimizer;
  sliding_window_graph_optimizer::
    SlidingWindowGraphOptimizerParams
      sliding_window_graph_optimizer;
};
}  // namespace tutorial_examples

#endif  // TUTORIAL_EXAMPLES_SIMPLE_LOCALIZER_PARAMS_H_
