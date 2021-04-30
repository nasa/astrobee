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
#ifndef GRAPH_OPTIMIZER_GRAPH_STATS_H_
#define GRAPH_OPTIMIZER_GRAPH_STATS_H_

#include <graph_optimizer/graph_values.h>
#include <localization_common/averager.h>
#include <localization_common/timer.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

namespace graph_optimizer {
class GraphStats {
 public:
  GraphStats();
  void AddStatsAverager(localization_common::Averager& stats_averager);
  void AddErrorAverager(localization_common::Averager& error_averager);
  virtual void UpdateErrors(const gtsam::NonlinearFactorGraph& graph_factors, const GraphValues& graph_values);
  virtual void UpdateStats(const gtsam::NonlinearFactorGraph& graph_factors, const GraphValues& graph_values);
  void Log() const;
  void LogToFile(std::ofstream& ofstream) const;
  void LogToCsv(std::ofstream& ofstream) const;

  std::vector<std::reference_wrapper<localization_common::Timer>> timers_;
  std::vector<std::reference_wrapper<localization_common::Averager>> stats_averagers_;
  std::vector<std::reference_wrapper<localization_common::Averager>> error_averagers_;

  // Timers
  localization_common::Timer optimization_timer_ = localization_common::Timer("Optimization");
  localization_common::Timer update_timer_ = localization_common::Timer("Update");
  localization_common::Timer marginals_timer_ = localization_common::Timer("Marginals");
  localization_common::Timer slide_window_timer_ = localization_common::Timer("Slide Window");
  localization_common::Timer add_buffered_factors_timer_ = localization_common::Timer("Add Buffered Factors");
  localization_common::Timer log_error_timer_ = localization_common::Timer("Log Error");
  localization_common::Timer log_stats_timer_ = localization_common::Timer("Log Stats");

  // Graph Stats Averagers
  localization_common::Averager iterations_averager_ = localization_common::Averager("Iterations");
  localization_common::Averager num_states_averager_ = localization_common::Averager("Num States");
  localization_common::Averager duration_averager_ = localization_common::Averager("Duration");
  localization_common::Averager num_marginal_factors_averager_ = localization_common::Averager("Num Marginal Factors");
  localization_common::Averager num_factors_averager_ = localization_common::Averager("Num Factors");
  localization_common::Averager num_features_averager_ = localization_common::Averager("Num Features");
  // Factor Error Averagers
  localization_common::Averager total_error_averager_ = localization_common::Averager("Total Factor Error");

 private:
  template <typename Logger>
  void LogToFile(const std::vector<std::reference_wrapper<Logger>>& loggers, std::ofstream& ofstream) const {
    for (const auto& logger : loggers) logger.get().LogToFile(ofstream);
  }

  template <typename Logger>
  void LogToCsv(const std::vector<std::reference_wrapper<Logger>>& loggers, std::ofstream& ofstream) const {
    for (const auto& logger : loggers) logger.get().LogToCsv(ofstream);
  }

  template <typename Logger>
  void Log(const std::vector<std::reference_wrapper<Logger>>& loggers) const {
    for (const auto& logger : loggers) logger.get().Log();
  }
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_STATS_H_
