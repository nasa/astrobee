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
namespace op = optimizers;

GraphOptimizer::GraphOptimizer(const GraphOptimizerParams& params)
    : params_(params), stats_logger_(params_.log_stats_on_destruction) {
  SetOptimizer();
  AddAveragersAndTimers();
}

void GraphOptimizer::AddNodeAdder(std::shared_ptr<na::NodeAdder> node_adder) {
  node_adders_.emplace_back(std::move(node_adder));
}

void GraphOptimizer::AddFactorAdder(std::shared_ptr<fa::FactorAdder> factor_adder) {
  factor_adders_.emplace_back(std::move(factor_adder));
}

int GraphOptimizer::AddFactors(const localization_common::Time start_time, const localization_common::Time end_time) {
  int num_added_factors = 0;
  for (const auto& factor_adder : factor_adders_) {
    num_added_factors += factor_adder->AddFactors(start_time, end_time, factors_);
  }
  return num_added_factors;
}

bool GraphOptimizer::Optimize() {
  if (!ValidGraph()) {
    LogError("Optimize: Invalid graph, not optimizing.");
    return false;
  }

  // Optimize
  LogDebug("Optimize: Optimizing.");
  optimization_timer_.Start();
  const bool successful_optimization = optimizer_->Optimize(factors_, nodes_->values());
  optimization_timer_.Stop();

  // iterations_averager_.Update(optimizer.iterations());
  total_error_averager_.Update(TotalGraphError());

  if (params_.print_after_optimization) Print();
  return successful_optimization;
}

boost::optional<gtsam::Matrix> GraphOptimizer::Covariance(const gtsam::Key& key) const {
  return optimizer_->Covariance(key);
}

const gtsam::NonlinearFactorGraph& GraphOptimizer::factors() const { return factors_; }

gtsam::NonlinearFactorGraph& GraphOptimizer::factors() { return factors_; }

const int GraphOptimizer::num_factors() const { return factors_.size(); }

const GraphOptimizerParams& GraphOptimizer::params() const { return params_; }

std::shared_ptr<nodes::Nodes> GraphOptimizer::nodes() { return nodes_; }

const gtsam::Values& GraphOptimizer::values() const { return nodes_->values(); }

localization_common::StatsLogger& GraphOptimizer::stats_logger() { return stats_logger_; }

double GraphOptimizer::TotalGraphError() const {
  double total_error = 0;
  for (const auto& factor : factors_) {
    const double error = factor->error(nodes_->values());
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
  factors_.saveGraph(of, nodes_->values());
}

void GraphOptimizer::SetOptimizer() {
  if (params_.optimizer == "nonlinear") {
    optimizer_.reset(new op::NonlinearOptimizer(params_.nonlinear_optimizer));
  } else if (params_.optimizer == "isam2") {
    optimizer_.reset(new op::ISAM2Optimizer(params_.isam2_optimizer));
  } else {  // Default to nonlinear optimizer
    optimizer_.reset(new op::NonlinearOptimizer(params_.nonlinear_optimizer));
  }
}

void GraphOptimizer::AddAveragersAndTimers() {
  stats_logger_.AddTimer(optimization_timer_);
  stats_logger_.AddAverager(iterations_averager_);
  stats_logger_.AddAverager(total_error_averager_);
}
}  // namespace graph_optimizer
