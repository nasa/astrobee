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
Averager::Averager(const std::string& name, const std::string& type_name, const std::string& units)
    : name_(name), type_name_(type_name), units_(units), last_value_(0) {
  // Don't add a space if type name isn't specified
  spacer_ = type_name_ == "" ? "" : " ";
}
void Averager::Update(const double value) {
  accumulator_(value);
  last_value_ = value;
}

double Averager::average() const { return boost::accumulators::mean(accumulator_); }

int Averager::count() const { return boost::accumulators::count(accumulator_); }

std::string Averager::LastValueString() const {
  std::stringstream ss;
  ss << name_ + spacer_ + type_name_ + ": " << last_value_ << spacer_ + units_;
  return ss.str();
}

std::string Averager::StatsString() const {
  std::stringstream ss;
  ss << "Average " + name_ + spacer_ + type_name_ + ": " << average() << spacer_ + units_
     << ", min: " << boost::accumulators::min(accumulator_) << ", max: " << boost::accumulators::max(accumulator_)
     << ", stddev: " << std::sqrt(boost::accumulators::variance(accumulator_));
  return ss.str();
}

void Averager::Log() const {
  LOG(INFO) << LastValueString();
  LOG(INFO) << StatsString();
}

void Averager::UpdateAndLog(const double value) {
  Update(value);
  Log();
}

void Averager::UpdateAndLogEveryN(const double value, const int num_events_per_log) {
  Update(value);
  LogEveryN(num_events_per_log);
}

void Averager::Vlog(const int level) const {
  VLOG(level) << LastValueString();
  VLOG(level) << StatsString();
}

void Averager::LogEveryN(const int num_events_per_log) const {
  if (count() % num_events_per_log == 0) {
    Log();
  }
}

void Averager::VlogEveryN(const int num_events_per_log, const int level) const {
  if (count() % num_events_per_log == 0) {
    Vlog(level);
  }
}

}  // namespace localization_common
