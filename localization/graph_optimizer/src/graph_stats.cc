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
#include <graph_optimizer/graph_stats.h>
#include <graph_optimizer/utilities.h>

#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace graph_optimizer {
GraphStats::GraphStats() {
  timers_.emplace_back(optimization_timer_);
  timers_.emplace_back(update_timer_);
  timers_.emplace_back(marginals_timer_);
  timers_.emplace_back(slide_window_timer_);
  timers_.emplace_back(add_buffered_factors_timer_);
  timers_.emplace_back(log_error_timer_);
  timers_.emplace_back(log_stats_timer_);

  AddStatsAverager(iterations_averager_);
  AddStatsAverager(num_states_averager_);
  AddStatsAverager(duration_averager_);
  AddStatsAverager(num_marginal_factors_averager_);
  AddStatsAverager(num_factors_averager_);
  AddStatsAverager(num_features_averager_);

  AddErrorAverager(total_error_averager_);
}

void GraphStats::AddStatsAverager(localization_common::Averager& stats_averager) {
  stats_averagers_.emplace_back(stats_averager);
}

void GraphStats::AddErrorAverager(localization_common::Averager& error_averager) {
  error_averagers_.emplace_back(error_averager);
}

void GraphStats::UpdateErrors(const gtsam::NonlinearFactorGraph& graph_factors, const GraphValues& graph_values) {}

void GraphStats::UpdateStats(const gtsam::NonlinearFactorGraph& graph_factors, const GraphValues& graph_values) {
  num_states_averager_.Update(graph_values.NumStates());
  duration_averager_.Update(graph_values.Duration());
  num_marginal_factors_averager_.Update(NumFactors<gtsam::LinearContainerFactor>(graph_factors));
  num_factors_averager_.Update(graph_factors.size());
  num_features_averager_.Update(graph_values.NumFeatures());
}

void GraphStats::Log() const {
  Log(timers_);
  Log(stats_averagers_);
  Log(error_averagers_);
}

void GraphStats::LogToFile(std::ofstream& ofstream) const {
  LogToFile(timers_, ofstream);
  LogToFile(stats_averagers_, ofstream);
  LogToFile(error_averagers_, ofstream);
}

void GraphStats::LogToCsv(std::ofstream& ofstream) const {
  ofstream << "name,avg,min,max,stddev" << std::endl;
  LogToCsv(timers_, ofstream);
  LogToCsv(stats_averagers_, ofstream);
  LogToCsv(error_averagers_, ofstream);
}
}  // namespace graph_optimizer
