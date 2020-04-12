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

#ifndef TRAJ_OPT_PRO_TIMERS_H_
#define TRAJ_OPT_PRO_TIMERS_H_

#include <chrono>
#include <iostream>
#include <string>
// #include <ros/ros.h>

namespace traj_opt {
// scoped timer
class ScopedTimer {
 public:
  explicit ScopedTimer(const std::string &str) {
    msg_ = str;
    start_ = std::chrono::system_clock::now();
  }
  ~ScopedTimer() {
    std::chrono::time_point<std::chrono::system_clock> end;
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> dt = end - start_;
    std::cout << msg_ << " took " << dt.count() << " seconds!" << std::endl;
  }

 private:
  std::string msg_;
  std::chrono::time_point<std::chrono::system_clock> start_;
};
// tic toc timer
class Timer {
 public:
  Timer() { tic(); }
  void tic() { start_ = std::chrono::system_clock::now(); }
  double toc() {
    std::chrono::time_point<std::chrono::system_clock> end;
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> dt = end - start_;
    return dt.count();
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start_;
};
}  // namespace traj_opt
#endif  // TRAJ_OPT_PRO_TIMERS_H_
