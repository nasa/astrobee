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

#ifndef FF_UTIL_PERF_TIMER_H_
#define FF_UTIL_PERF_TIMER_H_

#include <ros/ros.h>

#include <ff_msgs/Performance.h>

#include <chrono>
#include <string>

namespace ff_util {

class PerfTimer {
 public:
  void Initialize(std::string const& name) {
    ros::NodeHandle nh("/performance");
    pub_ = nh.advertise<ff_msgs::Performance>(name, 5);
    init_ = true;
  }
  void Clear() {
    if (!init_) return;
    msg_.count = msg_.mean = msg_.var = msg_.stddev = 0.0;
    msg_.last = msg_.max = msg_.min = -1.0;
  }
  void Tick() {
    if (!init_) return;
    start_ = std::chrono::system_clock::now();
  }
  void Tock() {
    if (!init_) return;
    std::chrono::time_point<std::chrono::system_clock> end;
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> dt = end - start_;
    // Add the measurement and the count
    msg_.stamp = ros::Time::now();
    msg_.count += 1.0;
    msg_.last = dt.count();
    if (msg_.last > msg_.max || msg_.min < 0.0) msg_.max = msg_.last;
    if (msg_.last < msg_.min || msg_.min < 0.0) msg_.min = msg_.last;
    if (msg_.count > 1) {
      // See: http://math.stackexchange.com/questions/102978/incremental-computation-of-standard-deviation
      msg_.var = (msg_.count - 2.0) / (msg_.count - 1.0) * msg_.var
               + (msg_.last - msg_.mean) * (msg_.last - msg_.mean) / msg_.count;
      msg_.mean = ((msg_.count - 1.0) * msg_.mean + msg_.last) / msg_.count;
    } else {
      msg_.mean = msg_.last;
      msg_.var = 0;
    }
    msg_.stddev = sqrt(msg_.var);
  }
  void Send() {
    if (!init_) return;
    pub_.publish(msg_);
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start_;
  ff_msgs::Performance msg_;
  ros::Publisher pub_;
  bool init_;
};

}  // namespace ff_util

#endif  // FF_UTIL_PERF_TIMER_H_
