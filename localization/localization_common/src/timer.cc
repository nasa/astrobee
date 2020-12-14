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

#include <localization_common/timer.h>

#include <glog/logging.h>

namespace localization_common {
Timer::Timer(const std::string& timer_name) : averager_(timer_name, "time", "seconds") {}
void Timer::Start() { start_time_ = std::chrono::steady_clock::now(); }
void Timer::Stop() {
  const auto end_time = std::chrono::steady_clock::now();
  const double elapsed_time = std::chrono::duration<double>(end_time - start_time_).count();
  averager_.Update(elapsed_time);
}

void Timer::Log() const { averager_.Log(); }

void Timer::LogEveryN(const int num_events_per_log) const { averager_.LogEveryN(num_events_per_log); }

void Timer::StopAndLog() {
  Stop();
  Log();
}

void Timer::StopAndLogEveryN(const int num_events_per_log) {
  Stop();
  LogEveryN(num_events_per_log);
}

void Timer::Vlog(const int level) const { averager_.Vlog(level); }
}  // namespace localization_common
