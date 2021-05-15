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

GraphOptimizer::GraphOptimizer(const GraphOptimizerParams& params, std::unique_ptr<GraphStats> graph_stats)
    : graph_stats_(std::move(graph_stats)), values_(new gtsam::Values()), log_on_destruction_(true), params_(params) {
  // Initialize lm params
  if (params_.verbose) {
    levenberg_marquardt_params_.verbosityLM = gtsam::LevenbergMarquardtParams::VerbosityLM::TERMINATION;
    levenberg_marquardt_params_.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::ERROR;
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
  if (log_on_destruction_) graph_stats_->Log();
}

void GraphOptimizer::AddGraphActionCompleter(std::shared_ptr<GraphActionCompleter> graph_action_completer) {
  graph_action_completers_.emplace_back(std::move(graph_action_completer));
}

void GraphOptimizer::AddNodeUpdater(std::shared_ptr<NodeUpdater> node_updater) {
  node_updaters_.emplace_back(std::move(node_updater));
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
  const auto linearized_graph = old_factors.linearize(*values_);
  const auto linear_marginal_factors =
    *(linearized_graph->eliminatePartialMultifrontal(old_keys, eliminate_function).second);
  return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_marginal_factors, *values_);
}

boost::optional<lc::Time> GraphOptimizer::SlideWindowNewOldestTime() const {
  boost::optional<lc::Time> new_oldest_time;
  for (const auto& node_updater : node_updaters_) {
    const auto node_new_oldest_time = node_updater->SlideWindowNewOldestTime();
    if (node_new_oldest_time) {
      new_oldest_time = new_oldest_time ? std::min(*node_new_oldest_time, *new_oldest_time) : *node_new_oldest_time;
    }
  }

  return new_oldest_time;
}

gtsam::KeyVector GraphOptimizer::OldKeys(const localization_common::Time oldest_allowed_time) const {
  gtsam::KeyVector all_old_keys;
  for (const auto& node_updater : node_updaters_) {
    const auto old_keys = node_updater->OldKeys(oldest_allowed_time, graph_);
    all_old_keys.insert(all_old_keys.end(), old_keys.begin(), old_keys.end());
  }
  return all_old_keys;
}

std::pair<gtsam::KeyVector, gtsam::NonlinearFactorGraph> GraphOptimizer::OldKeysAndFactors(
  const lc::Time oldest_allowed_time) {
  const auto old_keys = OldKeys(oldest_allowed_time);
  // Since cumlative factors have many keys and shouldn't be marginalized, need to remove old measurements depending on
  // old keys before marginalizing and sliding window
  RemoveOldMeasurementsFromCumulativeFactors(old_keys);
  const auto old_factors = RemoveOldFactors(old_keys, graph_);
  return std::make_pair(old_keys, old_factors);
}

bool GraphOptimizer::SlideWindow(const boost::optional<gtsam::Marginals>& marginals, const lc::Time last_latest_time) {
  const auto ideal_new_oldest_time = SlideWindowNewOldestTime();
  if (!ideal_new_oldest_time) {
    LogDebug("SlideWindow: No states removed. ");
    return true;
  }
  // Ensure that new oldest time isn't more recent than last latest time
  // since then priors couldn't be added for the new oldest state
  if (last_latest_time < *ideal_new_oldest_time)
    LogError("SlideWindow: Ideal oldest time is more recent than last latest time.");
  const auto new_oldest_time = std::min(last_latest_time, *ideal_new_oldest_time);

  const auto old_keys_and_factors = OldKeysAndFactors(new_oldest_time);
  if (params_.add_marginal_factors) {
    const auto marginal_factors =
      MarginalFactors(old_keys_and_factors.second, old_keys_and_factors.first, gtsam::EliminateQR);
    for (const auto& marginal_factor : marginal_factors) {
      graph_.push_back(marginal_factor);
    }
  }

  for (auto& node_updater : node_updaters_)
    node_updater->SlideWindow(new_oldest_time, marginals, old_keys_and_factors.first, params_.huber_k, graph_);

  RemoveOldBufferedFactors(new_oldest_time);
  DoPostSlideWindowActions(new_oldest_time, marginals);
  return true;
}

void GraphOptimizer::DoPostSlideWindowActions(const localization_common::Time oldest_allowed_time,
                                              const boost::optional<gtsam::Marginals>& marginals) {}

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
  for (auto& node_updater : node_updaters_) {
    if (node_updater->type() == key_info.node_updater_type()) return node_updater->Update(key_info.timestamp(), graph_);
  }
  LogError("UpdateNodes: No node updater found for key info.");
  return false;
}

bool GraphOptimizer::DoGraphAction(FactorsToAdd& factors_to_add) {
  if (factors_to_add.graph_action_completer_type() == GraphActionCompleterType::None) return true;
  for (auto& graph_action_completer : graph_action_completers_) {
    if (graph_action_completer->type() == factors_to_add.graph_action_completer_type())
      return graph_action_completer->DoAction(factors_to_add, graph_);
  }

  LogError("DoGraphAction: No graph action completer found for factors to add.");
  return false;
}

boost::optional<gtsam::Key> GraphOptimizer::GetKey(KeyCreatorFunction key_creator_function,
                                                   const localization_common::Time timestamp) const {
  // This avoids a bug in the compiler that produces a false warning that key may be uninitialized
  boost::optional<gtsam::Key> key = boost::make_optional(false, gtsam::Key());
  for (const auto& node_updater : node_updaters_) {
    const auto node_key = node_updater->GetKey(key_creator_function, timestamp);
    if (node_key) {
      if (!key)
        key = node_key;
      else {
        LogError("GetKey: Found key in multiple node updators.");
        return boost::none;
      }
    }
  }
  return key;
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
      const auto new_key = GetKey(key_info.key_creator_function(), key_info.timestamp());
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

boost::optional<lc::Time> GraphOptimizer::OldestTimestamp() const {
  boost::optional<lc::Time> oldest_timestamp;
  for (const auto& node_updater : node_updaters_) {
    const auto node_oldest_timestamp = node_updater->OldestTimestamp();
    if (node_oldest_timestamp) {
      oldest_timestamp =
        oldest_timestamp ? std::min(*oldest_timestamp, *node_oldest_timestamp) : *node_oldest_timestamp;
    }
  }
  return oldest_timestamp;
}

boost::optional<lc::Time> GraphOptimizer::LatestTimestamp() const {
  boost::optional<lc::Time> latest_timestamp;
  for (const auto& node_updater : node_updaters_) {
    const auto node_latest_timestamp = node_updater->LatestTimestamp();
    if (node_latest_timestamp) {
      latest_timestamp =
        latest_timestamp ? std::max(*latest_timestamp, *node_latest_timestamp) : *node_latest_timestamp;
    }
  }
  return latest_timestamp;
}

bool GraphOptimizer::MeasurementRecentEnough(const lc::Time timestamp) const {
  const auto oldest_timestamp = OldestTimestamp();
  if (!oldest_timestamp) {
    LogError("MeasurementRecentEnough: Failed to get oldest timestamp.");
    return false;
  }
  if (timestamp < *oldest_timestamp) return false;
  return true;
}

void GraphOptimizer::PrintFactorDebugInfo() const {}

const GraphOptimizerParams& GraphOptimizer::params() const { return params_; }

const gtsam::NonlinearFactorGraph& GraphOptimizer::graph_factors() const { return graph_; }

gtsam::NonlinearFactorGraph& GraphOptimizer::graph_factors() { return graph_; }

const boost::optional<gtsam::Marginals>& GraphOptimizer::marginals() const { return marginals_; }

std::shared_ptr<gtsam::Values> GraphOptimizer::values() { return values_; }

void GraphOptimizer::SaveGraphDotFile(const std::string& output_path) const {
  std::ofstream of(output_path.c_str());
  graph_.saveGraph(of, *values_);
}

const GraphStats* const GraphOptimizer::graph_stats() const { return graph_stats_.get(); }

GraphStats* GraphOptimizer::graph_stats() { return graph_stats_.get(); }

void GraphOptimizer::LogOnDestruction(const bool log_on_destruction) { log_on_destruction_ = log_on_destruction; }

bool GraphOptimizer::DoPostOptimizeActions() { return true; }

bool GraphOptimizer::Update() {
  LogDebug("Update: Updating.");
  graph_stats_->update_timer_.Start();

  graph_stats_->add_buffered_factors_timer_.Start();
  BufferCumulativeFactors();
  const int num_added_factors = AddBufferedFactors();
  graph_stats_->add_buffered_factors_timer_.Stop();
  if (num_added_factors <= 0) {
    LogDebug("Update: No factors added.");
    return false;
  }

  // Only get marginals and slide window if optimization has already occured
  // TODO(rsoussan): Make cleaner way to check for this
  if (last_latest_time_) {
    graph_stats_->marginals_timer_.Start();
    // Calculate marginals for covariances
    try {
      marginals_ = gtsam::Marginals(graph_, *values_, marginals_factorization_);
    } catch (gtsam::IndeterminantLinearSystemException) {
      log(params_.fatal_failures, "Update: Indeterminant linear system error during computation of marginals.");
      marginals_ = boost::none;
    } catch (...) {
      log(params_.fatal_failures, "Update: Computing marginals failed.");
      marginals_ = boost::none;
    }
    graph_stats_->marginals_timer_.Stop();

    graph_stats_->slide_window_timer_.Start();
    if (!SlideWindow(marginals_, *last_latest_time_)) {
      LogError("Update: Failed to slide window.");
      return false;
    }
    graph_stats_->slide_window_timer_.Stop();
  }

  // TODO(rsoussan): Is ordering required? if so clean these calls open and unify with marginalization
  // TODO(rsoussan): Remove this now that marginalization occurs before optimization?
  if (params_.add_marginal_factors) {
    // Add graph ordering to place keys that will be marginalized in first group
    const auto new_oldest_time = SlideWindowNewOldestTime();
    if (new_oldest_time) {
      const auto old_keys = OldKeys(*new_oldest_time);
      const auto ordering = gtsam::Ordering::ColamdConstrainedFirst(graph_, old_keys);
      levenberg_marquardt_params_.setOrdering(ordering);
    } else {
      levenberg_marquardt_params_.orderingType = gtsam::Ordering::COLAMD;
    }
  }

  // Optimize
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, *values_, levenberg_marquardt_params_);

  graph_stats_->optimization_timer_.Start();
  // TODO(rsoussan): Indicate if failure occurs in state msg, perhaps using confidence value in msg
  try {
    *values_ = optimizer.optimize();
  } catch (gtsam::IndeterminantLinearSystemException) {
    log(params_.fatal_failures, "Update: Graph optimization failed, indeterminant linear system, keeping old values.");
  } catch (gtsam::InvalidNoiseModel) {
    log(params_.fatal_failures, "Update: Graph optimization failed, invalid noise model, keeping old values.");
  } catch (gtsam::InvalidMatrixBlock) {
    log(params_.fatal_failures, "Update: Graph optimization failed, invalid matrix block, keeping old values.");
  } catch (gtsam::InvalidDenseElimination) {
    log(params_.fatal_failures, "Update: Graph optimization failed, invalid dense elimination, keeping old values.");
  } catch (...) {
    log(params_.fatal_failures, "Update: Graph optimization failed, keeping old values.");
  }
  graph_stats_->optimization_timer_.Stop();

  // Calculate marginals after the first optimization iteration so covariances
  // can be used for first loc msg
  // TODO(rsoussan): Clean this up
  if (!last_latest_time_) {
    graph_stats_->marginals_timer_.Start();
    // Calculate marginals for covariances
    try {
      marginals_ = gtsam::Marginals(graph_, *values_, marginals_factorization_);
    } catch (gtsam::IndeterminantLinearSystemException) {
      log(params_.fatal_failures, "Update: Indeterminant linear system error during computation of marginals.");
      marginals_ = boost::none;
    } catch (...) {
      log(params_.fatal_failures, "Update: Computing marginals failed.");
      marginals_ = boost::none;
    }
    graph_stats_->marginals_timer_.Stop();
  }

  last_latest_time_ = LatestTimestamp();

  graph_stats_->log_stats_timer_.Start();
  graph_stats_->iterations_averager_.Update(optimizer.iterations());
  graph_stats_->UpdateStats(graph_);
  graph_stats_->log_stats_timer_.Stop();
  graph_stats_->log_error_timer_.Start();
  graph_stats_->UpdateErrors(graph_);
  graph_stats_->log_error_timer_.Stop();

  if (params_.print_factor_info) PrintFactorDebugInfo();
  if (!DoPostOptimizeActions()) {
    LogError("Update: Failed to complete post optimize actions.");
    return false;
  }
  graph_stats_->update_timer_.Stop();
  return true;
}
}  // namespace graph_optimizer
