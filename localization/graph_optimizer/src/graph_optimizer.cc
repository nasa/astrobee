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

#include <graph_optimizer/graph_optimizer.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

namespace graph_optimizer {
namespace fa = factor_adders;
namespace lc = localization_common;
namespace na = node_adders;

GraphOptimizer::GraphOptimizer(const GraphOptimizerParams& params)
    : params_(params), stats_logger_(params_.log_stats_on_destruction) {
  SetOptimizationParams();
  AddAveragersAndTimers();
}

void GraphOptimizer::SetOptimizationParams() {
  // Initialize lm params
  if (params_.verbose) {
    levenberg_marquardt_params_.verbosityLM = gtsam::LevenbergMarquardtParams::VerbosityLM::TRYDELTA;
    levenberg_marquardt_params_.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::LINEAR;
  }
  if (params_.use_ceres_params) {
    gtsam::LevenbergMarquardtParams::SetCeresDefaults(&levenberg_marquardt_params_);
  }

  levenberg_marquardt_params_.maxIterations = params_.max_iterations;
}

void GraphOptimizer::AddAveragersAndTimers() {
  stats_averager_.AddTimer(optimization_timer_);
  stats_averager_.AddAverager(iterations_averager_);
  stats_averager_.AddAverager(total_error_averager_);
}

void GraphOptimizer::AddNodeAdder(std::shared_ptr<na::NodeAdder> node_adder) {
  node_adders_.emplace_back(std::move(node_adder));
}

void GraphOptimizer::AddFactorAdder(std::shared_ptr<fa::FactorAdder> factor_adder) {
  factor_adders_.emplace_back(std::move(factor_adder));
}

bool GraphOptimizer::ValidGraph() const { return true; }

void GraphOptimizer::Print() const {
  factors_.print();
  stats_logger.Log();
}

const GraphOptimizerParams& GraphOptimizer::params() const { return params_; }

const gtsam::NonlinearFactorGraph& GraphOptimizer::factors() const { return factors_; }

gtsam::NonlinearFactorGraph& GraphOptimizer::factors() { return factors_; }

const int GraphOptimizer::num_factors() const { return factors_.size(); }

void GraphOptimizer::SaveGraphDotFile(const std::string& output_path) const {
  std::ofstream of(output_path.c_str());
  // TODO(rsoussan): get this from nodes
  factors.saveGraph(of, *values_);
}

bool GraphOptimizer::Optimize() {
  LogDebug("Optimize: Optimizing.");

  if (!ValidGraph()) {
    LogError("Optimize: Invalid graph, not optimizing.");
    return false;
  }

  // Optimize
  gtsam::LevenbergMarquardtOptimizer optimizer(factors, *values_, levenberg_marquardt_params_);
  bool successful_optimization = true;
  optimization_timer_.Start();
  // TODO(rsoussan): Indicate if failure occurs in state msg, perhaps using confidence value in msg
  try {
    *values_ = optimizer.optimize();
  } catch (gtsam::IndeterminantLinearSystemException) {
    LogError("Optimize: Graph optimization failed, indeterminant linear system.");
    successful_optimization = false;
  } catch (gtsam::InvalidNoiseModel) {
    LogError("Optimize: Graph optimization failed, invalid noise model.");
    successful_optimization = false;
  } catch (gtsam::InvalidMatrixBlock) {
    LogError("Optimize: Graph optimization failed, invalid matrix block.");
    successful_optimization = false;
  } catch (gtsam::InvalidDenseElimination) {
    LogError("Optimize: Graph optimization failed, invalid dense elimination.");
    successful_optimization = false;
  } catch (...) {
    LogError("Optimize: Graph optimization failed.");
    successful_optimization = false;
  }
  optimization_timer_.Stop();

  iterations_averager_.Update(optimizer.iterations());
  total_error_averager_.Update(TotalGraphError());

  if (params_.print_after_optimization) Print();
  return successful_optimization;
}

int GraphOptimizer::AddFactors(const localization_common::Time start_time, const localization_common end_time) {
  int num_added_factors = 0;
  for (const auto& factor_adder : factor_adders_) {
    num_added_factors += factor_adder->AddFactors(start_time, end_time, factors_);
  }
  return num_added_factors;
}


double TotalGraphError() const {
  double total_error = 0;
  for (const auto& factor : factors_) {
    // TODO(rsoussan): get values from somewhere else!!
    const double error = factor->error(values_);
    total_error += error;
  }
  return total_error;
}
}  // namespace graph_optimizer
