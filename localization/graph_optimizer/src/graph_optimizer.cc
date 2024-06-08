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
#include <optimizers/isam2_optimizer.h>
#include <optimizers/nonlinear_optimizer.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

namespace graph_optimizer {
namespace fa = factor_adders;
namespace lc = localization_common;
namespace na = node_adders;
namespace no = nodes;
namespace op = optimizers;

GraphOptimizer::GraphOptimizer(const GraphOptimizerParams& params, std::unique_ptr<optimizers::Optimizer> optimizer)
    : params_(params),
      optimizer_(std::move(optimizer)),
      values_(new no::Values()),
      stats_logger_(params_.log_stats_on_destruction) {
  AddAveragersAndTimers();
}

void GraphOptimizer::AddNodeAdder(std::shared_ptr<na::NodeAdder> node_adder) {
  node_adder->AddInitialNodesAndPriors(factors_);
  node_adders_.emplace_back(std::move(node_adder));
}

void GraphOptimizer::AddFactorAdder(std::shared_ptr<fa::FactorAdder> factor_adder) {
  factor_adders_.emplace_back(std::move(factor_adder));
}

int GraphOptimizer::AddFactors(const lc::Time start_time, const lc::Time end_time) {
  int num_added_factors = 0;
  for (const auto& factor_adder : factor_adders_) {
    num_added_factors += factor_adder->AddFactors(start_time, end_time, factors_);
  }
  return num_added_factors;
}

int GraphOptimizer::AddFactors(const std::pair<lc::Time, lc::Time>& start_and_end_time) {
  return AddFactors(start_and_end_time.first, start_and_end_time.second);
}

bool GraphOptimizer::Optimize() {
  if (!ValidGraph()) {
    LogError("Optimize: Invalid graph, not optimizing.");
    return false;
  }

  // Optimize
  LogDebug("Optimize: Optimizing.");
  optimization_timer_.Start();
  const bool successful_optimization = optimizer_->Optimize(factors_, gtsam_values());
  optimization_timer_.Stop();

  optimization_iterations_averager_.Update(optimizer_->iterations());
  total_error_averager_.Update(TotalGraphError());

  if (params_.print_after_optimization) Print();
  return successful_optimization;
}

boost::optional<gtsam::Matrix> GraphOptimizer::Covariance(const gtsam::Key& key) const {
  return optimizer_->Covariance(key);
}

boost::optional<gtsam::Matrix> GraphOptimizer::Covariance(const gtsam::Key& key_a, const gtsam::Key& key_b) const {
  return optimizer_->Covariance(key_a, key_b);
}

const gtsam::NonlinearFactorGraph& GraphOptimizer::factors() const { return factors_; }

gtsam::NonlinearFactorGraph& GraphOptimizer::factors() { return factors_; }

int GraphOptimizer::num_factors() const { return factors_.size(); }

int GraphOptimizer::num_values() const { return values_->size(); }

const GraphOptimizerParams& GraphOptimizer::params() const { return params_; }

std::shared_ptr<nodes::Values> GraphOptimizer::values() { return values_; }

const gtsam::Values& GraphOptimizer::gtsam_values() const { return values_->gtsam_values(); }

gtsam::Values& GraphOptimizer::gtsam_values() { return values_->gtsam_values(); }

lc::StatsLogger& GraphOptimizer::stats_logger() { return stats_logger_; }

const lc::Timer& GraphOptimizer::optimization_timer() const { return optimization_timer_; }

const localization_common::Averager& GraphOptimizer::optimization_iterations_averager() const {
  return optimization_iterations_averager_;
}

double GraphOptimizer::TotalGraphError() const {
  double total_error = 0;
  for (const auto& factor : factors_) {
    const double error = factor->error(gtsam_values());
    total_error += error;
  }
  return total_error;
}

bool GraphOptimizer::ValidGraph() const { return true; }

void GraphOptimizer::Print() const {
  factors_.print();
  stats_logger_.Log();
}

void GraphOptimizer::SaveGraphDotFile(const std::string& output_path) const {
  std::ofstream of(output_path.c_str());
  factors_.saveGraph(of, gtsam_values());
}

boost::optional<const gtsam::Marginals&> GraphOptimizer::marginals() const { return optimizer_->marginals(); }

void GraphOptimizer::AddAveragersAndTimers() {
  stats_logger_.AddTimer(optimization_timer_);
  stats_logger_.AddAverager(optimization_iterations_averager_);
  stats_logger_.AddAverager(total_error_averager_);
}
}  // namespace graph_optimizer
