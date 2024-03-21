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
#include <localization_common/stats_logger.h>

namespace localization_common {
StatsLogger::StatsLogger(const bool log_on_destruction) : log_on_destruction_(log_on_destruction) {}

StatsLogger::~StatsLogger() {
  if (log_on_destruction_) Log();
}

void StatsLogger::AddAverager(localization_common::Averager& averager) { averagers_.emplace_back(averager); }

void StatsLogger::AddTimer(localization_common::Timer& timer) { timers_.emplace_back(timer); }

void StatsLogger::Log() const {
  Log(timers_);
  Log(averagers_);
}

void StatsLogger::LogToFile(std::ofstream& ofstream) const {
  LogToFile(timers_, ofstream);
  LogToFile(averagers_, ofstream);
}

void StatsLogger::LogToCsv(std::ofstream& ofstream) const {
  ofstream << "name,avg,min,max,stddev" << std::endl;
  LogToCsv(timers_, ofstream);
  LogToCsv(averagers_, ofstream);
}
}  // namespace localization_common
