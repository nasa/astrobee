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
RateTimer::RateTimer(const std::string& timer_name)
    : name_(timer_name), last_elapsed_time_(0), average_elapsed_time_(0), num_timing_events_(0) {}
void RateTimer::Record() {
  const auto end_time = std::chrono::steady_clock::now();
  ++num_timing_events_;
  // First Recording, nothing to compare to yet
  if (num_timing_events_ == 1) {
    start_time_ = end_time;
    return;
  }

  last_elapsed_time_ = std::chrono::duration<double>(end_time - start_time_).count();
  // Compute moving average to avoid overflow
  // Subtract one since first timing event starts the timing
  average_elapsed_time_ += (last_elapsed_time_ - average_elapsed_time_) / (num_timing_events_ - 1);
  start_time_ = end_time;
}
void RateTimer::RecordAndLog() {
  Record();
  Log();
}
void RateTimer::Log() {
  if (num_timing_events_ <= 1) return;
  LOG(INFO) << name_ + " rate: " << last_elapsed_time_ << " seconds.";
  LOG(INFO) << "Average " + name_ + " rate: " << average_elapsed_time_ << " seconds.";
}

void RateTimer::Vlog(const int level) {
  if (num_timing_events_ <= 1) return;
  VLOG(level) << name_ + " rate: " << last_elapsed_time_ << " seconds.";
  VLOG(level) << "Average " + name_ + " rate: " << average_elapsed_time_ << " seconds.";
}
}  // namespace localization_common
