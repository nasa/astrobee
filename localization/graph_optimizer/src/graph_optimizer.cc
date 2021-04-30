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

GraphOptimizer::GraphOptimizer(const GraphOptimizerParams& params)
    : graph_values_(new GraphValues(params.graph_values)), log_on_destruction_(true), params_(params) {
  // Initialize lm params
  if (params_.verbose) {
    levenberg_marquardt_params_.verbosityLM = gtsam::LevenbergMarquardtParams::VerbosityLM::TRYDELTA;
    levenberg_marquardt_params_.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::LINEAR;
  }
  if (params_.use_ceres_params) {
    gtsam::LevenbergMarquardtParams::SetCeresDefaults(&levenberg_marquardt_params_);
  }

  levenberg_marquardt_params_.maxIterations = params_.max_iterations;

  if (params_.marginals_factorization == "qr") {
    marginals_factorization_ = gtsam::Marginals::Factorization::QR;
  } else if (params_.marginals_factorization == "cholesky") {
    marginals_factorization_ = gtsam::Marginals::Factorization::CHOLESKY;
  } else {
    LogError("GraphOptimizer: No marginals factorization entered, defaulting to qr.");
    marginals_factorization_ = gtsam::Marginals::Factorization::QR;
  }
}

GraphOptimizer::~GraphOptimizer() {
  if (log_on_destruction_) graph_stats_.Log();
}

void GraphOptimizer::AddGraphActionCompleter(std::shared_ptr<GraphActionCompleter> graph_action_completer) {
  graph_action_completers_.emplace_back(std::move(graph_action_completer));
}

void GraphOptimizer::AddTimestampedNodeUpdater(std::shared_ptr<TimestampedNodeUpdater> timestamped_node_updater) {
  timestamped_node_updaters_.emplace_back(std::move(timestamped_node_updater));
}

// Adapted from gtsam::BatchFixedLagSmoother
gtsam::NonlinearFactorGraph GraphOptimizer::MarginalFactors(
  const gtsam::NonlinearFactorGraph& old_factors, const gtsam::KeyVector& old_keys,
  const gtsam::GaussianFactorGraph::Eliminate& eliminate_function) const {
  // Old keys not present in old factors.  This shouldn't occur.
  if (old_keys.size() == 0) {
    LogDebug("MarginalFactors: No old keys provided.");
    return old_factors;
  }

  // Linearize Graph
  const auto linearized_graph = old_factors.linearize(graph_values_->values());
  const auto linear_marginal_factors =
    *(linearized_graph->eliminatePartialMultifrontal(old_keys, eliminate_function).second);
  return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_marginal_factors, graph_values_->values());
}

bool GraphOptimizer::SlideWindow(const boost::optional<gtsam::Marginals>& marginals, const lc::Time last_latest_time) {
  const auto graph_values_ideal_new_oldest_time = graph_values_->SlideWindowNewOldestTime();
  if (!graph_values_ideal_new_oldest_time) {
    LogDebug("SlideWindow: No states removed. ");
    return true;
  }
  // Ensure that new oldest time isn't more recent than last latest time
  // since then priors couldn't be added for the new oldest state
  if (last_latest_time < *graph_values_ideal_new_oldest_time)
    LogError("SlideWindow: Ideal oldest time is more recent than last latest time.");
  const auto new_oldest_time = std::min(last_latest_time, *graph_values_ideal_new_oldest_time);

  // Add marginal factors for marginalized values
  auto old_keys = graph_values_->OldKeys(new_oldest_time);
  // Since cumlative factors have many keys and shouldn't be marginalized, need to remove old measurements depending on
  // old keys before marginalizing and sliding window
  RemoveOldMeasurementsFromCumulativeFactors(old_keys);
  auto old_factors = graph_values_->RemoveOldFactors(old_keys, graph_);
  gtsam::KeyVector old_feature_keys;
  // Call remove old factors before old feature keys, since old feature keys depend on
  // number of factors per key remaining
  // TODO(rsoussan): Generalize this better
  old_feature_keys = graph_values_->OldFeatureKeys(graph_);
  auto old_feature_factors = graph_values_->RemoveOldFactors(old_feature_keys, graph_);
  old_keys.insert(old_keys.end(), old_feature_keys.begin(), old_feature_keys.end());
  old_factors.push_back(old_feature_factors);
  if (params_.add_marginal_factors) {
    const auto marginal_factors = MarginalFactors(old_factors, old_keys, gtsam::EliminateQR);
    for (const auto& marginal_factor : marginal_factors) {
      graph_.push_back(marginal_factor);
    }
  }

  for (auto& node_updater : timestamped_node_updaters_)
    node_updater->SlideWindow(new_oldest_time, marginals, params_.huber_k, graph_, *graph_values_);
  graph_values_->RemoveOldFeatures(old_feature_keys);

  // Remove old data from other containers
  // TODO(rsoussan): Just use new_oldest_time and don't bother getting oldest timestamp here?
  const auto oldest_timestamp = graph_values_->OldestTimestamp();
  if (!oldest_timestamp || *oldest_timestamp != new_oldest_time) {
    LogError("SlideWindow: Failed to get oldest timestamp.");
    return false;
  }

  RemoveOldBufferedFactors(*oldest_timestamp);
  SlideOtherWindows();
  return true;
}

void GraphOptimizer::SlideOtherWindows() {}

void GraphOptimizer::BufferCumulativeFactors() {}

void GraphOptimizer::RemoveOldMeasurementsFromCumulativeFactors(const gtsam::KeyVector& old_keys) {}

void GraphOptimizer::BufferFactors(const std::vector<FactorsToAdd>& factors_to_add_vec) {
  for (const auto& factors_to_add : factors_to_add_vec)
    buffered_factors_to_add_.emplace(factors_to_add.timestamp(), factors_to_add);
}

void GraphOptimizer::RemoveOldBufferedFactors(const lc::Time oldest_allowed_timestamp) {
  for (auto factors_to_add_it = buffered_factors_to_add_.begin();
       factors_to_add_it != buffered_factors_to_add_.end();) {
    auto& factors_to_add = factors_to_add_it->second.Get();
    for (auto factor_to_add_it = factors_to_add.begin(); factor_to_add_it != factors_to_add.end();) {
      bool removed_factor = false;
      for (const auto& key_info : factor_to_add_it->key_infos) {
        // Ignore static keys
        if (key_info.is_static()) continue;
        if (key_info.timestamp() < oldest_allowed_timestamp) {
          LogDebug("RemoveOldBufferedFactors: Removing old factor from buffered factors.");
          factor_to_add_it = factors_to_add.erase(factor_to_add_it);
          removed_factor = true;
          break;
        }
      }
      if (!removed_factor) ++factor_to_add_it;
    }
    if (factors_to_add_it->second.Get().empty()) {
      LogDebug("RemoveOldBufferedFactors: Removing old factors from buffered factors.");
      factors_to_add_it = buffered_factors_to_add_.erase(factors_to_add_it);
    } else {
      ++factors_to_add_it;
    }
  }
}

int GraphOptimizer::AddBufferedFactors() {
  LogDebug("AddBufferedfactors: Adding buffered factors.");
  LogDebug("AddBufferedFactors: Num buffered factors to add: " << buffered_factors_to_add_.size());

  int num_added_factors = 0;
  for (auto factors_to_add_it = buffered_factors_to_add_.begin();
       factors_to_add_it != buffered_factors_to_add_.end() && ReadyToAddFactors(factors_to_add_it->first);) {
    auto& factors_to_add = factors_to_add_it->second;
    for (auto& factor_to_add : factors_to_add.Get()) {
      for (const auto& key_info : factor_to_add.key_infos) {
        if (!UpdateNodes(key_info)) {
          LogError("AddBufferedFactors: Failed to update nodes.");
        }
      }

      if (!Rekey(factor_to_add)) {
        LogError("AddBufferedMeasurements: Failed to rekey factor to add.");
        continue;
      }
    }

    if (!DoGraphAction(factors_to_add)) {
      LogDebug("AddBufferedFactors: Failed to complete graph action.");
      factors_to_add_it = buffered_factors_to_add_.erase(factors_to_add_it);
      continue;
    }

    for (auto& factor_to_add : factors_to_add.Get()) {
      graph_.push_back(factor_to_add.factor);
      ++num_added_factors;
    }
    factors_to_add_it = buffered_factors_to_add_.erase(factors_to_add_it);
  }

  LogDebug("AddBufferedFactors: Added " << num_added_factors << " factors.");
  return num_added_factors;
}

bool GraphOptimizer::UpdateNodes(const KeyInfo& key_info) {
  // Do nothing for static nodes
  if (key_info.is_static()) return true;
  for (auto& node_updater : timestamped_node_updaters_) {
    if (node_updater->type() == key_info.node_updater_type())
      return node_updater->Update(key_info.timestamp(), graph_, *graph_values_);
  }
  LogError("UpdateNodes: No node updater found for key info.");
  return false;
}

bool GraphOptimizer::DoGraphAction(FactorsToAdd& factors_to_add) {
  if (factors_to_add.graph_action_completer_type() == GraphActionCompleterType::None) return true;
  for (auto& graph_action_completer : graph_action_completers_) {
    if (graph_action_completer->type() == factors_to_add.graph_action_completer_type())
      return graph_action_completer->DoAction(factors_to_add, graph_, *graph_values_);
  }

  LogError("DoGraphAction: No graph action completer found for factors to add.");
  return false;
}

bool GraphOptimizer::Rekey(FactorToAdd& factor_to_add) {
  gtsam::KeyVector new_keys;
  const auto& old_keys = factor_to_add.factor->keys();
  for (int i = 0; i < static_cast<int>(factor_to_add.key_infos.size()); ++i) {
    const auto& key_info = factor_to_add.key_infos[i];
    if (key_info.is_static()) {
      // Don't change static keys. Assumes static key currently in factor is correct
      new_keys.emplace_back(old_keys[i]);
    } else {
      const auto new_key = graph_values_->GetKey(key_info.key_creator_function(), key_info.timestamp());
      if (!new_key) {
        LogError("ReKey: Failed to find new key for timestamp.");
        return false;
      }
      new_keys.emplace_back(*new_key);
    }
  }
  factor_to_add.factor->keys() = new_keys;
  return true;
}

bool GraphOptimizer::ReadyToAddFactors(const localization_common::Time timestamp) const { return true; }

bool GraphOptimizer::MeasurementRecentEnough(const lc::Time timestamp) const {
  if (timestamp < graph_values_->OldestTimestamp()) return false;
  return true;
}

void GraphOptimizer::PrintFactorDebugInfo() const {}

const GraphOptimizerParams& GraphOptimizer::params() const { return params_; }

const GraphValues& GraphOptimizer::graph_values() const { return *graph_values_; }

const gtsam::NonlinearFactorGraph& GraphOptimizer::graph_factors() const { return graph_; }

gtsam::NonlinearFactorGraph& GraphOptimizer::graph_factors() { return graph_; }

void GraphOptimizer::SaveGraphDotFile(const std::string& output_path) const {
  std::ofstream of(output_path.c_str());
  graph_.saveGraph(of, graph_values_->values());
}

const GraphStatsBase& GraphOptimizer::graph_stats() const { return graph_stats_; }

void GraphOptimizer::LogOnDestruction(const bool log_on_destruction) { log_on_destruction_ = log_on_destruction; }

void GraphOptimizer::PostOptimizeActions() {}

bool GraphOptimizer::Update() {
  LogDebug("Update: Updating.");
  graph_stats_.update_timer_.Start();

  graph_stats_.add_buffered_factors_timer_.Start();
  BufferCumulativeFactors();
  const int num_added_factors = AddBufferedFactors();
  graph_stats_.add_buffered_factors_timer_.Stop();
  if (num_added_factors <= 0) {
    LogDebug("Update: No factors added.");
    return false;
  }

  // Only get marginals and slide window if optimization has already occured
  // TODO(rsoussan): Make cleaner way to check for this
  if (last_latest_time_) {
    graph_stats_.marginals_timer_.Start();
    // Calculate marginals for covariances
    try {
      marginals_ = gtsam::Marginals(graph_, graph_values_->values(), marginals_factorization_);
    } catch (gtsam::IndeterminantLinearSystemException) {
      log(params_.fatal_failures, "Update: Indeterminant linear system error during computation of marginals.");
      marginals_ = boost::none;
    } catch (...) {
      log(params_.fatal_failures, "Update: Computing marginals failed.");
      marginals_ = boost::none;
    }
    graph_stats_.marginals_timer_.Stop();

    graph_stats_.slide_window_timer_.Start();
    if (!SlideWindow(marginals_, *last_latest_time_)) {
      LogError("Update: Failed to slide window.");
      return false;
    }
    graph_stats_.slide_window_timer_.Stop();
  }

  // TODO(rsoussan): Is ordering required? if so clean these calls open and unify with marginalization
  // TODO(rsoussan): Remove this now that marginalization occurs before optimization?
  if (params_.add_marginal_factors) {
    // Add graph ordering to place keys that will be marginalized in first group
    const auto new_oldest_time = graph_values_->SlideWindowNewOldestTime();
    if (new_oldest_time) {
      const auto old_keys = graph_values_->OldKeys(*new_oldest_time);
      const auto ordering = gtsam::Ordering::ColamdConstrainedFirst(graph_, old_keys);
      levenberg_marquardt_params_.setOrdering(ordering);
    } else {
      levenberg_marquardt_params_.orderingType = gtsam::Ordering::COLAMD;
    }
  }

  // Optimize
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, graph_values_->values(), levenberg_marquardt_params_);

  graph_stats_.optimization_timer_.Start();
  // TODO(rsoussan): Indicate if failure occurs in state msg, perhaps using confidence value in msg 
  try {
    graph_values_->UpdateValues(optimizer.optimize());
  } catch (gtsam::IndeterminantLinearSystemException) {
    log(params_.fatal_failures, "Update: Indeterminant linear system error during optimization, keeping old values.");
  } catch (...) {
    log(params_.fatal_failures, "Update: Graph optimization failed, keeping old values.");
  }
  graph_stats_.optimization_timer_.Stop();

  // Calculate marginals after the first optimization iteration so covariances
  // can be used for first loc msg
  // TODO(rsoussan): Clean this up
  if (!last_latest_time_) {
    graph_stats_.marginals_timer_.Start();
    // Calculate marginals for covariances
    try {
      marginals_ = gtsam::Marginals(graph_, graph_values_->values(), marginals_factorization_);
    } catch (gtsam::IndeterminantLinearSystemException) {
      log(params_.fatal_failures, "Update: Indeterminant linear system error during computation of marginals.");
      marginals_ = boost::none;
    } catch (...) {
      log(params_.fatal_failures, "Update: Computing marginals failed.");
      marginals_ = boost::none;
    }
    graph_stats_.marginals_timer_.Stop();
  }

  last_latest_time_ = graph_values_->LatestTimestamp();

  graph_stats_.log_stats_timer_.Start();
  graph_stats_.iterations_averager_.Update(optimizer.iterations());
  graph_stats_.UpdateStats(graph_, *graph_values_);
  graph_stats_.log_stats_timer_.Stop();
  graph_stats_.log_error_timer_.Start();
  graph_stats_.UpdateErrors(graph_, *graph_values_);
  graph_stats_.log_error_timer_.Stop();

  if (params_.print_factor_info) PrintFactorDebugInfo();
  PostOptimizeActions();
  graph_stats_.update_timer_.Stop();
  return true;
}
}  // namespace graph_optimizer
