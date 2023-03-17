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

#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer.h>

#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace sliding_window_graph_optimizer {
namespace lc = localization_common;
namespace na = node_adders;

SlidingWindowGraphOptimizer::SlidingWindowGraphOptimizer(const SlidingWindowGraphOptimizerParams& params)
    : params_(params), GraphOptimizer(params) {
  AddAveragersAndTimers();
}

void SlidingWindowGraphOptimizer::AddSlidingWindowNodeAdder(
  std::shared_ptr<na::SlidingWindowNodeAdder> sliding_window_node_adder) {
  sliding_window_node_adders_.emplace_back(sliding_window_node_adder);
  AddNodeAdder(sliding_window_node_adder);
}

bool SlidingWindowGraphOptimizer::Update() {
  LogDebug("Update: Updating.");

  update_timer_.Start();
  if (marginals()) {
    if (!SlideWindow(*(marginals()), *end_time_)) {
      LogError("Update: Failed to slide window.");
      return false;
    }
  }

  GraphOptimizer::Optimize();
  end_time_ = EndTime();
  update_timer_.Stop();
  return true;
}

bool SlidingWindowGraphOptimizer::SlideWindow(const gtsam::Marginals& marginals, const lc::Time end_time) {
  const auto ideal_new_start_time = SlideWindowNewStartTime();
  if (!ideal_new_start_time) {
    LogDebug("SlideWindow: No states removed. ");
    return true;
  }
  // Ensure that new start time isn't more recent than current end time
  // since then priors couldn't be added for the new start nodes.
  if (end_time < *ideal_new_start_time) LogError("SlideWindow: Ideal start time is more recent than current end time.");
  const auto new_start_time = std::min(end_time, *ideal_new_start_time);

  const auto old_keys = OldKeys(new_start_time);
  const auto old_factors = lc::RemoveOldFactors(old_keys, factors());
  if (params_.add_marginal_factors) {
    const auto marginal_factors = MarginalFactors(old_factors, old_keys, gtsam::EliminateQR);
    for (const auto& marginal_factor : marginal_factors) {
      factors().push_back(marginal_factor);
    }
  }

  for (auto& sliding_window_node_adder : sliding_window_node_adders_)
    sliding_window_node_adder->SlideWindow(new_start_time, marginals, old_keys, params_.huber_k, factors());
  return true;
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
  const auto linearized_graph = old_factors.linearize(values());
  const auto linear_marginal_factors =
    *(linearized_graph->eliminatePartialMultifrontal(old_keys, eliminate_function).second);
  return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_marginal_factors, values());
}

boost::optional<lc::Time> SlidingWindowGraphOptimizer::SlideWindowNewStartTime() const {
  boost::optional<lc::Time> new_start_time;
  for (const auto& sliding_window_node_adder : sliding_window_node_adders_) {
    const auto node_new_start_time = sliding_window_node_adder->SlideWindowNewStartTime();
    if (node_new_start_time) {
      new_start_time = new_start_time ? std::min(*node_new_start_time, *new_start_time) : *node_new_start_time;
    }
  }

  return new_start_time;
}

gtsam::KeyVector SlidingWindowGraphOptimizer::OldKeys(const localization_common::Time oldest_allowed_time) const {
  gtsam::KeyVector all_old_keys;
  for (const auto& sliding_window_node_adder : sliding_window_node_adders_) {
    const auto old_keys = sliding_window_node_adder->OldKeys(oldest_allowed_time, factors());
    all_old_keys.insert(all_old_keys.end(), old_keys.begin(), old_keys.end());
  }
  return all_old_keys;
}

boost::optional<lc::Time> SlidingWindowGraphOptimizer::EndTime() const {
  boost::optional<lc::Time> end_time;
  for (const auto& sliding_window_node_adder : sliding_window_node_adders_) {
    const auto node_end_time = sliding_window_node_adder->EndTime();
    if (node_end_time) {
      end_time = end_time ? std::max(*end_time, *node_end_time) : *node_end_time;
    }
  }
  return end_time;
}

void SlidingWindowGraphOptimizer::AddAveragersAndTimers() { stats_logger().AddTimer(update_timer_); }
}  // namespace sliding_window_graph_optimizer
