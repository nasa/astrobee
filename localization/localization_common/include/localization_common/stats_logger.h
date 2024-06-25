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
#ifndef LOCALIZATION_COMMON_STATS_LOGGER_H_
#define LOCALIZATION_COMMON_STATS_LOGGER_H_

#include <localization_common/averager.h>
#include <localization_common/timer.h>

#include <vector>

namespace localization_common {
// Logs various statistics and runtimes to a desired output (command line, CSV file, text file) using averager and timer
// objects. Uses references to provided averagers and timers to log min/max/mean/stddev values and runtimes
// respectively.
class StatsLogger {
 public:
  explicit StatsLogger(const bool log_on_destruction = true);

  ~StatsLogger();

  // Add averager for logging
  void AddAverager(localization_common::Averager& averager);

  // Add timer for logging
  void AddTimer(localization_common::Timer& timer);

  // Log averages and times to command line.
  void Log() const;

  // Log averages and times to a text file.
  void LogToFile(std::ofstream& ofstream) const;

  // Log averages and times to a CSV file.
  void LogToCsv(std::ofstream& ofstream) const;

  std::vector<std::reference_wrapper<localization_common::Timer>> timers_;
  std::vector<std::reference_wrapper<localization_common::Averager>> averagers_;

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

  bool log_on_destruction_;
};
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_STATS_LOGGER_H_
