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
#include <graph_localizer/graph_stats_base.h>

#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace graph_localizer {
GraphStatsBase::GraphStatsBase() {
  timers_.emplace_back(optimization_timer_);
  timers_.emplace_back(update_timer_);
  timers_.emplace_back(marginals_timer_);
  timers_.emplace_back(slide_window_timer_);
  timers_.emplace_back(add_buffered_factors_timer_);
  timers_.emplace_back(log_error_timer_);
  timers_.emplace_back(log_stats_timer_);

  stats_averagers_.emplace_back(iterations_averager_);
  stats_averagers_.emplace_back(num_states_averager_);
  stats_averagers_.emplace_back(duration_averager_);
  stats_averagers_.emplace_back(num_marginal_factors_averager_);
  stats_averagers_.emplace_back(num_factors_averager_);
  stats_averagers_.emplace_back(num_features_averager_);

  error_averagers_.emplace_back(total_error_averager_);
}

void GraphStatsBase::UpdateErrors(const gtsam::NonlinearFactorGraph& graph_factors, const GraphValues& graph_values) {}

void GraphStatsBase::UpdateStats(const gtsam::NonlinearFactorGraph& graph_factors, const GraphValues& graph_values) {
  num_states_averager_.Update(graph_values.NumStates());
  duration_averager_.Update(graph_values.Duration());
  num_marginal_factors_averager_.Update(NumFactors<gtsam::LinearContainerFactor>(graph_factors));
  num_factors_averager_.Update(graph_factors.size());
}

void GraphStatsBase::Log() const {
  Log(timers_);
  Log(stats_averagers_);
  Log(error_averagers_);
}

void GraphStatsBase::LogToFile(std::ofstream& ofstream) const {
  LogToFile(timers_, ofstream);
  LogToFile(stats_averagers_, ofstream);
  LogToFile(error_averagers_, ofstream);
}

void GraphStatsBase::LogToCsv(std::ofstream& ofstream) const {
  ofstream << "name,avg,min,max,stddev" << std::endl;
  LogToCsv(timers_, ofstream);
  LogToCsv(stats_averagers_, ofstream);
  LogToCsv(error_averagers_, ofstream);
}
}  // namespace graph_localizer
