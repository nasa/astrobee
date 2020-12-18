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

#include <localization_common/ros_timer.h>
#include <localization_common/utilities.h>

namespace localization_common {
RosTimer::RosTimer(const std::string& timer_name) : averager_(timer_name, "time", "seconds"), start_time_(0) {}
void RosTimer::Start() { start_time_ = ros::Time::now(); }
void RosTimer::HeaderDiff(const std_msgs::Header& header) {
  start_time_ = RosTimeFromHeader(header);
  Stop();
}

void RosTimer::Stop() {
  const auto end_time = ros::Time::now();
  const double elapsed_time = (end_time - start_time_).toSec();
  averager_.Update(elapsed_time);
}
void RosTimer::Log() const { averager_.Log(); }
void RosTimer::Vlog(const int level) const { averager_.Vlog(); }
void RosTimer::LogEveryN(const int num_timing_events_per_log) const { averager_.LogEveryN(num_timing_events_per_log); }
void RosTimer::VlogEveryN(const int num_timing_events_per_log, const int level) const {
  averager_.VlogEveryN(num_timing_events_per_log, level);
}
}  // namespace localization_common
