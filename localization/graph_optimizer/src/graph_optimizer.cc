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
namespace lc = localization_common;

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

GraphOptimizer::~GraphOptimizer() {
  if (params_.log_on_destruction) graph_stats_->Log();
}

void GraphOptimizer::AddNodeUpdater(std::shared_ptr<NodeUpdater> node_updater) {
  node_updaters_.emplace_back(std::move(node_updater));
}

bool GraphOptimizer::ValidGraph() const { return true; }

void GraphOptimizer::PrintFactorDebugInfo() const {}

const GraphOptimizerParams& GraphOptimizer::params() const { return params_; }

const gtsam::NonlinearFactorGraph& GraphOptimizer::factors() const { return graph_; }

gtsam::NonlinearFactorGraph& GraphOptimizer::factors() { return graph_; }

const int GraphOptimizer::num_factors() const { return graph_.size(); }

void GraphOptimizer::SaveGraphDotFile(const std::string& output_path) const {
  std::ofstream of(output_path.c_str());
  // TODO(rsoussan): get this from nodes
  graph_.saveGraph(of, *values_);
}

const GraphStats& const GraphOptimizer::graph_stats() const { return graph_stats_; }

GraphStats& GraphOptimizer::graph_stats() { return graph_stats_; }

void GraphOptimizer::LogOnDestruction(const bool log_on_destruction) {
  params_.log_on_destruction = log_on_destruction;
}

bool GraphOptimizer::Update() {
  LogDebug("Update: Updating.");
  graph_stats_->update_timer_.Start();

  if (!ValidGraph()) {
    LogError("Update: Invalid graph, not optimizing.");
    return false;
  }

  // Optimize
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, *values_, levenberg_marquardt_params_);
  bool successful_optimization = true;
  graph_stats_->optimization_timer_.Start();
  // TODO(rsoussan): Indicate if failure occurs in state msg, perhaps using confidence value in msg
  try {
    *values_ = optimizer.optimize();
  } catch (gtsam::IndeterminantLinearSystemException) {
    log(params_.fatal_failures, "Update: Graph optimization failed, indeterminant linear system.");
    successful_optimization = false;
  } catch (gtsam::InvalidNoiseModel) {
    log(params_.fatal_failures, "Update: Graph optimization failed, invalid noise model.");
    successful_optimization = false;
  } catch (gtsam::InvalidMatrixBlock) {
    log(params_.fatal_failures, "Update: Graph optimization failed, invalid matrix block.");
    successful_optimization = false;
  } catch (gtsam::InvalidDenseElimination) {
    log(params_.fatal_failures, "Update: Graph optimization failed, invalid dense elimination.");
    successful_optimization = false;
  } catch (...) {
    log(params_.fatal_failures, "Update: Graph optimization failed.");
    successful_optimization = false;
  }
  graph_stats_->optimization_timer_.Stop();

  graph_stats_->log_stats_timer_.Start();
  graph_stats_->iterations_averager_.Update(optimizer.iterations());
  graph_stats_->UpdateStats(graph_);
  graph_stats_->log_stats_timer_.Stop();
  graph_stats_->log_error_timer_.Start();
  graph_stats_->UpdateErrors(graph_);
  graph_stats_->log_error_timer_.Stop();

  if (params_.print_factor_info) PrintFactorDebugInfo();
  graph_stats_->update_timer_.Stop();
  return successful_optimization;
}
}  // namespace graph_optimizer
