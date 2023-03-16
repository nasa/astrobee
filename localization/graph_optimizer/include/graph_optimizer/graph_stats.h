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

#include <localization_common/averager.h>
#include <localization_common/timer.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

namespace graph_optimizer {
// Helper object that logs various statistics and runtimes to a desired output (command line, CSV file, text file).
// Uses averager and timer objects that generate min/max/mean/stddev values and runtimes respectively for each of these.
//
// Default logs optimization runtime, number of optimization iterations, and total factor error.
// More statistics and runtimes can be used for logging by calling the AddStatsAverager and AddTimer methods.
// More detailed statistics can be logged for a factor graph by overriding the UpdateStats method.
class GraphStats {
 public:
  explicit GraphStats(const bool log_on_destruction = true);
  // Add averager
  void AddAverager(localization_common::Averager& stats_averager);
  void AddTimer(localization_common::Timer& timer);
  virtual void UpdateStats(const gtsam::NonlinearFactorGraph& graph_factors);

  // Write statistics and runtimes to command line.
  void Log() const;

  // Write statistics and runtimes to a text file.
  void LogToFile(std::ofstream& ofstream) const;

  // Write statistics and runtimes to a CSV file.
  void LogToCsv(std::ofstream& ofstream) const;

  std::vector<std::reference_wrapper<localization_common::Timer>> timers_;
  std::vector<std::reference_wrapper<localization_common::Averager>> stats_averagers_;

  // Timers
  localization_common::Timer optimization_timer_ = localization_common::Timer("Optimization");
  // Averagers
  localization_common::Averager iterations_averager_ = localization_common::Averager("Iterations");
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
