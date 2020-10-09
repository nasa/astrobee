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
Timer::Timer(const std::string& timer_name)
    : name_(timer_name), last_elapsed_time_(0), average_elapsed_time_(0), num_timing_events_(0) {}
void Timer::Start() { start_time_ = std::chrono::steady_clock::now(); }
void Timer::Stop() {
  const auto end_time = std::chrono::steady_clock::now();
  last_elapsed_time_ = std::chrono::duration<double>(end_time - start_time_).count();
  ++num_timing_events_;
  // Compute moving average to avoid overflow
  average_elapsed_time_ += (last_elapsed_time_ - average_elapsed_time_) / num_timing_events_;
}
void Timer::Log() {
  LOG(INFO) << name_ + " time: " << last_elapsed_time_ << " seconds.";
  LOG(INFO) << "Average " + name_ + " time: " << average_elapsed_time_ << " seconds.";
}
void Timer::Vlog(const int level) {
  VLOG(level) << name_ + " time: " << last_elapsed_time_ << " seconds.";
  VLOG(level) << "Average " + name_ + " time: " << average_elapsed_time_ << " seconds.";
}
}  // namespace localization_common
