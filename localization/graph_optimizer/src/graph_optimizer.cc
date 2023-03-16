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
#include <graph_optimizer/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <unistd.h>

#include <chrono>
#include <unordered_set>

namespace {
// TODO(rsoussan): Is this necessary? Just use DFATAL and compile with debug?
// Avoid having to compile with DEBUG to toggle between fatal and non-fatal failures
void log(const bool fatal_failure, const std::string& description) {
  if (fatal_failure) {
    LogFatal(description);
  } else {
    LogError(description);
  }
}
}  // namespace

namespace graph_optimizer {
namespace fa = factor_adders;
namespace lc = localization_common;
namespace na = node_adders;

GraphOptimizer::GraphOptimizer(const GraphOptimizerParams& params) : params_(params) {
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

void GraphOptimizer::AddNodeAdder(std::shared_ptr<na::NodeAdder> node_adder) {
  node_adders_.emplace_back(std::move(node_adder));
}

void GraphOptimizer::AddFactorAdder(std::shared_ptr<fa::FactorAdder> factor_adder) {
  factor_adders_.emplace_back(std::move(factor_adder));
}

bool GraphOptimizer::ValidGraph() const { return true; }

void GraphOptimizer::PrintFactorDebugInfo() const {}

const GraphOptimizerParams& GraphOptimizer::params() const { return params_; }

const gtsam::NonlinearFactorGraph& GraphOptimizer::factors() const { return factors; }

gtsam::NonlinearFactorGraph& GraphOptimizer::factors() { return factors; }

const int GraphOptimizer::num_factors() const { return factors.size(); }

void GraphOptimizer::SaveGraphDotFile(const std::string& output_path) const {
  std::ofstream of(output_path.c_str());
  // TODO(rsoussan): get this from nodes
  factors.saveGraph(of, *values_);
}

const GraphStats& const GraphOptimizer::graph_stats() const { return graph_stats_; }

GraphStats& GraphOptimizer::graph_stats() { return graph_stats_; }

bool GraphOptimizer::Optimize() {
  LogDebug("Optimize: Optimizing.");
  graph_stats_->update_timer_.Start();

  if (!ValidGraph()) {
    LogError("Optimize: Invalid graph, not optimizing.");
    return false;
  }

  // Optimize
  gtsam::LevenbergMarquardtOptimizer optimizer(factors, *values_, levenberg_marquardt_params_);
  bool successful_optimization = true;
  graph_stats_->optimization_timer_.Start();
  // TODO(rsoussan): Indicate if failure occurs in state msg, perhaps using confidence value in msg
  try {
    *values_ = optimizer.optimize();
  } catch (gtsam::IndeterminantLinearSystemException) {
    log(params_.fatal_failures, "Optimize: Graph optimization failed, indeterminant linear system.");
    successful_optimization = false;
  } catch (gtsam::InvalidNoiseModel) {
    log(params_.fatal_failures, "Optimize: Graph optimization failed, invalid noise model.");
    successful_optimization = false;
  } catch (gtsam::InvalidMatrixBlock) {
    log(params_.fatal_failures, "Optimize: Graph optimization failed, invalid matrix block.");
    successful_optimization = false;
  } catch (gtsam::InvalidDenseElimination) {
    log(params_.fatal_failures, "Optimize: Graph optimization failed, invalid dense elimination.");
    successful_optimization = false;
  } catch (...) {
    log(params_.fatal_failures, "Optimize: Graph optimization failed.");
    successful_optimization = false;
  }
  graph_stats_->optimization_timer_.Stop();

  graph_stats_->log_stats_timer_.Start();
  graph_stats_->iterations_averager_.Update(optimizer.iterations());
  graph_stats_->UpdateStats(factors);
  graph_stats_->log_stats_timer_.Stop();
  graph_stats_->log_error_timer_.Start();
  graph_stats_->UpdateErrors(factors);
  graph_stats_->log_error_timer_.Stop();

  if (params_.print_factor_info) PrintFactorDebugInfo();
  graph_stats_->update_timer_.Stop();
  return successful_optimization;
}

int GraphOptimizer::AddFactors(const localization_common::Time start_time, const localization_common end_time) {
  int num_added_factors = 0;
  for (const auto& factor_adder : factor_adders_) {
    num_added_factors += factor_adder->AddFactors(start_time, end_time, factors_);
  }
  return num_added_factors;
}
}  // namespace graph_optimizer
