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

#include <localization_common/rate_timer.h>

#include <glog/logging.h>

namespace localization_common {
RateTimer::RateTimer(const std::string& timer_name) : averager_(timer_name, "rate", "seconds"), num_events_(0) {}
void RateTimer::Record() {
  const auto end_time = std::chrono::steady_clock::now();
  ++num_events_;
  // First Recording, nothing to compare to yet
  if (num_events_ == 1) {
    start_time_ = end_time;
    return;
  }

  const double elapsed_time = std::chrono::duration<double>(end_time - start_time_).count();
  averager_.Update(elapsed_time);
  start_time_ = end_time;
}
void RateTimer::RecordAndLog() {
  Record();
  Log();
}
void RateTimer::Log() const {
  if (num_events_ <= 1) return;
  averager_.Log();
}

void RateTimer::Vlog(const int level) const {
  if (num_events_ <= 1) return;
  averager_.Vlog(level);
}
}  // namespace localization_common
