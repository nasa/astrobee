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

#include <graph_factors/cumulative_factor.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer.h>

#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace sliding_window_graph_optimizer {
namespace lc = localization_common;
namespace na = node_adders;

SlidingWindowGraphOptimizer::SlidingWindowGraphOptimizer(const SlidingWindowGraphOptimizerParams& params,
                                                         std::unique_ptr<optimizers::Optimizer> optimizer)
    : params_(params), GraphOptimizer(params, std::move(optimizer)) {
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
  AddFactors(WindowStartAndEndTimes());
  if (params_.slide_window_before_optimization) SlideWindow();
  GraphOptimizer::Optimize();
  if (!params_.slide_window_before_optimization) SlideWindow();
  update_timer_.Stop();
  return true;
}

bool SlidingWindowGraphOptimizer::SlideWindow() {
  const auto new_start_time = NewStartTime();
  if (!new_start_time) {
    LogDebug(
      "SlideWindow: Failed to get new start time, node adders may not have enough nodes to slide window. Not sliding "
      "window.");
    return false;
  }
  const auto old_keys = OldKeys(*new_start_time);
  const auto old_factors = RemoveFactors(old_keys);
  if (params_.add_marginal_factors) {
    const auto marginal_factors = MarginalFactors(old_factors, old_keys, gtsam::EliminateQR);
    for (const auto& marginal_factor : marginal_factors) {
      factors().push_back(marginal_factor);
    }
  }

  for (auto& sliding_window_node_adder : sliding_window_node_adders_)
    sliding_window_node_adder->SlideWindow(*new_start_time, marginals(), old_keys, params_.huber_k, factors());
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
  const auto linearized_graph = old_factors.linearize(gtsam_values());
  const auto linear_marginal_factors =
    *(linearized_graph->eliminatePartialMultifrontal(old_keys, eliminate_function).second);
  return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_marginal_factors, gtsam_values());
}

gtsam::KeyVector SlidingWindowGraphOptimizer::OldKeys(const localization_common::Time oldest_allowed_time) const {
  gtsam::KeyVector all_old_keys;
  for (const auto& sliding_window_node_adder : sliding_window_node_adders_) {
    const auto old_keys = sliding_window_node_adder->OldKeys(oldest_allowed_time, factors());
    all_old_keys.insert(all_old_keys.end(), old_keys.begin(), old_keys.end());
  }
  return all_old_keys;
}

std::pair<lc::Time, lc::Time> SlidingWindowGraphOptimizer::WindowStartAndEndTimes() const {
  auto start_time = EarliestNodeAdderStartTime();
  // Shouldn't occur since node adders initialize their first nodes when added to the graph.
  if (!start_time) start_time = 0;
  // Add all new factors possible.
  // When sliding window before optimization, this will keep the latest factors and
  // compute the relative start time wrt them.
  // When sliding window after optimization, this may increase the window size too much.
  // TODO(rsoussan): Add param to set window size (start time) if sliding window after optimization?
  const lc::Time end_time = std::numeric_limits<double>::max();
  return {*start_time, end_time};
}

boost::optional<lc::Time> SlidingWindowGraphOptimizer::NewStartTime() const {
  const auto ideal_new_start_time = IdealNodeAddersNewStartTime();
  if (!ideal_new_start_time) {
    LogDebug("NewStartTime: No start times available.");
    return boost::none;
  }
  // Ensure that new start time isn't more recent than current end time
  // since then priors couldn't be added for the new start nodes.
  const auto end_time = LatestNodeAdderEndTime();
  if (!end_time) {
    LogDebug("NewStartTime: No end time available.");
    return boost::none;
  }
  if (*end_time < *ideal_new_start_time)
    LogError("NewStartTime: Ideal start time is more recent than current end time.");
  return std::min(*end_time, *ideal_new_start_time);
}

boost::optional<lc::Time> SlidingWindowGraphOptimizer::IdealNodeAddersNewStartTime() const {
  boost::optional<lc::Time> new_start_time;
  for (const auto& sliding_window_node_adder : sliding_window_node_adders_) {
    const auto node_adder_new_start_time = sliding_window_node_adder->SlideWindowNewStartTime();
    if (node_adder_new_start_time) {
      // Use max here so the latest start time is chosen and window sizes for all node adders
      // are <= desired sizes.
      new_start_time =
        new_start_time ? std::max(*node_adder_new_start_time, *new_start_time) : *node_adder_new_start_time;
    }
  }

  return new_start_time;
}

boost::optional<lc::Time> SlidingWindowGraphOptimizer::EarliestNodeAdderStartTime() const {
  boost::optional<lc::Time> earliest_start_time;
  for (const auto& sliding_window_node_adder : sliding_window_node_adders_) {
    const auto node_adder_start_time = sliding_window_node_adder->StartTime();
    if (node_adder_start_time) {
      earliest_start_time =
        earliest_start_time ? std::min(*earliest_start_time, *node_adder_start_time) : *node_adder_start_time;
    }
  }
  return earliest_start_time;
}

boost::optional<lc::Time> SlidingWindowGraphOptimizer::LatestNodeAdderEndTime() const {
  boost::optional<lc::Time> latest_end_time;
  for (const auto& sliding_window_node_adder : sliding_window_node_adders_) {
    const auto node_adder_end_time = sliding_window_node_adder->EndTime();
    if (node_adder_end_time) {
      latest_end_time = latest_end_time ? std::max(*latest_end_time, *node_adder_end_time) : *node_adder_end_time;
    }
  }
  return latest_end_time;
}

double SlidingWindowGraphOptimizer::Duration() const {
  double duration;
  const auto latest_end_time = LatestNodeAdderEndTime();
  const auto earliest_start_time = EarliestNodeAdderStartTime();
  if (!latest_end_time || !earliest_start_time) return 0;
  return *latest_end_time - *earliest_start_time;
}

gtsam::NonlinearFactorGraph SlidingWindowGraphOptimizer::RemoveFactors(const gtsam::KeyVector& keys_to_remove) {
  gtsam::NonlinearFactorGraph removed_factors;
  if (keys_to_remove.empty()) return removed_factors;
  // Create set for quick lookup
  std::unordered_set<gtsam::Key> keys_to_remove_set;
  for (const auto& key : keys_to_remove) {
    keys_to_remove_set.emplace(key);
  }

  for (auto factor_it = factors().begin(); factor_it != factors().end();) {
    const auto cumulative_factor = dynamic_cast<const gtsam::CumulativeFactor*>(factor_it->get());
    // TODO(rsoussan): Add function to create factor from keys and measurements that are removed!
    // Return this with removed factors.
    // Try to remove old keys from cumulative factors
    if (cumulative_factor) {
      *factor_it = cumulative_factor->PrunedCopy(keys_to_remove_set);
      // Remove pruned copy if it is invalid
      if (!(*factor_it)) {
        factor_it = factors().erase(factor_it);
        continue;
      } else {
        ++factor_it;
        continue;
      }
    }
    bool remove_factor = false;
    const auto& factor_keys = (*factor_it)->keys();
    for (const auto& factor_key : factor_keys) {
      if (keys_to_remove_set.count(factor_key) > 0) {
        remove_factor = true;
      }
    }
    if (!remove_factor) {
      ++factor_it;
    } else {
      removed_factors.push_back(*factor_it);
      factor_it = factors().erase(factor_it);
    }
  }
  return removed_factors;
}

void SlidingWindowGraphOptimizer::AddAveragersAndTimers() { stats_logger().AddTimer(update_timer_); }

const localization_common::Timer& SlidingWindowGraphOptimizer::update_timer() const { return update_timer_; }

}  // namespace sliding_window_graph_optimizer
