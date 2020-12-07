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

#include <localization_common/averager.h>

#include <glog/logging.h>

namespace localization_common {
Averager::Averager(const std::string& name) : name_(name), last_value_(0), average_(0), num_events_(0) {}
void Averager::Update(const double value) {
  last_value_ = value;
  ++num_events_;
  // Compute moving average to avoid overflow
  average_ += (value - average_) / num_events_;
}

double Averager::average() const { return average_; }

void Averager::Log() {
  LOG(INFO) << name_ + ": " << last_value_;
  LOG(INFO) << "Average " + name_ + ": " << average_;
}
void Averager::UpdateAndLog(const double value) {
  Update(value);
  Log();
}

void Averager::Vlog(const int level) {
  VLOG(level) << name_ + ": " << last_value_;
  VLOG(level) << "Average " + name_ + ": " << average_;
}
}  // namespace localization_common
