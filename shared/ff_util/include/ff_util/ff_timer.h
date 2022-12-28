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

#ifndef FF_UTIL_FF_TIMER_H_
#define FF_UTIL_FF_TIMER_H_

// ROS includes
#include <ff_common/ff_ros.h>

// C++ includes
#include <chrono>

namespace ff_util {

////////////////////////////// TIMER CODE //////////////////////////////////////

// This is a simple wrapper around the ROS2 timer. The ROS2 timer API is a bit
// different than the ROS1 timer. This wrapper will attempt to

class FreeFlyerTimer {
 public:
  // Callback types
  typedef std::function < void (void) > TimerCallbackType;

  // Setters for callbacks
  void SetTimerCallback(TimerCallbackType cb_timer) { cb_timer_ = cb_timer; }

  // Constructor
  FreeFlyerTimer() : one_shot_(false),
    auto_start_(true),
    created_(false) {}

  // Destructor
  ~FreeFlyerTimer() {}

  void createTimer(double period_sec,
                   TimerCallbackType cb_timer,
                   rclcpp::Node::SharedPtr node,
                   bool one_shot = false,
                   bool auto_start = true) {
    period_ = period_sec;
    cb_timer_ = cb_timer;
    one_shot_ = one_shot;
    auto_start_ = auto_start;
    node_ = node;
    clock_ = node_->get_clock();
    if (auto_start_) {
      createStartTimer();
    }
  }

  void createTimer(double period_sec,
                   TimerCallbackType cb_timer,
                   rclcpp::Node::SharedPtr node,
                   rclcpp::Clock::SharedPtr clock,
                   bool one_shot = false,
                   bool auto_start = true) {
    period_ = period_sec;
    cb_timer_ = cb_timer;
    one_shot_ = one_shot;
    auto_start_ = auto_start;
    node_ = node;
    clock_ = clock;
    if (auto_start_) {
      createStartTimer();
    }
  }

  void reset() {
    if (created_) {
      timer_->reset();
    }
  }

  // Follows the ros1 one convention that it doesn't do anything if the timer
  // is already started
  void start() {
    if (!auto_start_) {
      createStartTimer();
    } else if (timer_->is_canceled()) {
      timer_->reset();
    }
  }

  void stop() {
    if (created_) {
      timer_->cancel();
    }
  }

 protected:
  void createStartTimer() {
    timer_ = rclcpp::create_timer(node_,
                                  clock_,
                                  rclcpp::Duration::from_seconds(period_),
                                  std::bind(&FreeFlyerTimer::TimerCallback, this));
    created_ = true;
  }

  // Called when the timer times out
  void TimerCallback() {
    if (one_shot_) {
      timer_->cancel();
    }
    if (cb_timer_) {
      cb_timer_();
    }
  }

 protected:
  bool one_shot_, auto_start_, created_;
  double period_;
  TimerCallbackType cb_timer_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ff_util

#endif  // FF_UTIL_FF_TIMER_H_
