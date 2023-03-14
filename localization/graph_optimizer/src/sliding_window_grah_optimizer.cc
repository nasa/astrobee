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

#include <graph_optimizer/sliding_window_graph_optimizer.h>
#include <graph_optimizer/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <unistd.h>

#include <chrono>
#include <unordered_set>

namespace graph_optimizer {
namespace lc = localization_common;

SlidingWindowGraphOptimizer::SlidingWindowGraphOptimizer(const SlidingWindowGraphOptimizerParams& params)
    : params_(params), GraphOptimizer(params) {
  if (params_.marginals_factorization == "qr") {
    marginals_factorization_ = gtsam::Marginals::Factorization::QR;
  } else if (params_.marginals_factorization == "cholesky") {
    marginals_factorization_ = gtsam::Marginals::Factorization::CHOLESKY;
  } else {
    LogError("SlidingWindowGraphOptimizer: No marginals factorization entered, defaulting to qr.");
    marginals_factorization_ = gtsam::Marginals::Factorization::QR;
  }
}

// Adapted from gtsam::BatchFixedLagSmoother
gtsam::NonlinearFactorGraph SlidingWindowGraphOptimizer::MarginalFactors(
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

boost::optional<lc::Time> SlidingWindowGraphOptimizer::SlideWindowNewOldestTime() const {
  boost::optional<lc::Time> new_start_time;
  for (const auto& node_updater : node_updaters_) {
    const auto node_new_start_time = node_updater->SlideWindowNewOldestTime();
    if (node_new_start_time) {
      new_start_time = new_start_time ? std::min(*node_new_start_time, *new_start_time) : *node_new_start_time;
    }
  }

  return new_start_time;
}

gtsam::KeyVector SlidingWindowGraphOptimizer::OldKeys(const localization_common::Time oldest_allowed_time) const {
  gtsam::KeyVector all_old_keys;
  for (const auto& node_updater : node_updaters_) {
    const auto old_keys = node_updater->OldKeys(oldest_allowed_time, graph_);
    all_old_keys.insert(all_old_keys.end(), old_keys.begin(), old_keys.end());
  }
  return all_old_keys;
}

bool SlidingWindowGraphOptimizer::SlideWindow(const boost::optional<gtsam::Marginals>& marginals, const lc::Time last_end_time) {
  const auto ideal_new_start_time = SlideWindowNewOldestTime();
  if (!ideal_new_start_time) {
    LogDebug("SlideWindow: No states removed. ");
    return true;
  }
  // Ensure that new oldest time isn't more recent than last latest time
  // since then priors couldn't be added for the new oldest state
  if (last_end_time < *ideal_new_start_time)
    LogError("SlideWindow: Ideal oldest time is more recent than last latest time.");
  const auto new_start_time = std::min(last_end_time, *ideal_new_start_time);

  const auto old_keys = OldKeys(new_start_time);
  const auto old_factors = RemoveOldFactors(old_keys, graph_);
  if (params_.add_marginal_factors) {
    const auto marginal_factors =
      MarginalFactors(old_keys, old_factors, gtsam::EliminateQR);
    for (const auto& marginal_factor : marginal_factors) {
      graph_.push_back(marginal_factor);
    }
  }

  for (auto& node_updater : node_updaters_)
    node_updater->SlideWindow(new_start_time, marginals, old_keys, params_.huber_k, graph_);
  return true;
}

boost::optional<lc::Time> SlidingWindowGraphOptimizer::OldestTimestamp() const {
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

boost::optional<lc::Time> SlidingWindowGraphOptimizer::LatestTimestamp() const {
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

bool SlidingWindowGraphOptimizer::MeasurementRecentEnough(const lc::Time timestamp) const {
  const auto oldest_timestamp = OldestTimestamp();
  if (!oldest_timestamp) {
    LogError("MeasurementRecentEnough: Failed to get oldest timestamp.");
    return false;
  }
  if (timestamp < *oldest_timestamp) return false;
  return true;
}

const SlidingWindowGraphOptimizerParams& SlidingWindowGraphOptimizer::params() const { return params_; }

const boost::optional<gtsam::Marginals>& SlidingWindowGraphOptimizer::marginals() const { return marginals_; }

void SlidingWindowGraphOptimizer::CalculateMarginals() {
    try {
      marginals_ = gtsam::Marginals(graph_, *values_, marginals_factorization_);
    } catch (gtsam::IndeterminantLinearSystemException) {
      log(params_.fatal_failures, "Update: Indeterminant linear system error during computation of marginals.");
      marginals_ = boost::none;
    } catch (const std::exception& exception) {
      log(params_.fatal_failures, "Update: Computing marginals failed. " + std::string(exception.what()));
      marginals_ = boost::none;
    } catch (...) {
      log(params_.fatal_failures, "Update: Computing marginals failed.");
      marginals_ = boost::none;
    }
}

void SlidingWindowGraphOptimizer::SetOrdering() {
    const auto new_start_time = SlideWindowNewOldestTime();
    if (new_start_time) {
      const auto old_keys = OldKeys(*new_start_time);
      const auto ordering = gtsam::Ordering::ColamdConstrainedFirst(graph_, old_keys);
      levenberg_marquardt_params_.setOrdering(ordering);
    } else {
      levenberg_marquardt_params_.orderingType = gtsam::Ordering::COLAMD;
    }
}

bool SlidingWindowGraphOptimizer::Update() {
  LogDebug("Update: Updating.");
  graph_stats_->update_timer_.Start();

  // Only get marginals and slide window if optimization has already occured
  // TODO(rsoussan): Make cleaner way to check for this
  if (last_end_time_) {
    CalculateMarginals();
    if (!SlideWindow(marginals_, *last_end_time_)) {
      LogError("Update: Failed to slide window.");
      return false;
    }
  }

  // TODO(rsoussan): Is ordering required? if so clean these calls open and unify with marginalization
  // TODO(rsoussan): Remove this now that marginalization occurs before optimization?
  if (params_.add_marginal_factors) {
    SetOrdering();    
 }

  GraphOptimizer::Update();

  // Calculate marginals after the first optimization iteration so covariances
  // can be used for first loc msg
  // TODO(rsoussan): Clean this up
  if (!last_end_time_) {
    CalculateMarginals();
  }

  last_end_time_ = LatestTimestamp();
  return true;
}
}  // namespace graph_optimizer
