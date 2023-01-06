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
// different than the ROS1 timer. This wrapper provides ROS1 timer functionality
// using ROS2 timers.

class FreeFlyerTimer {
 public:
  // Callback types
  typedef std::function < void (void) > TimerCallbackType;

  // Setters for callbacks
  void SetTimerCallback(TimerCallbackType cb_timer) { cb_timer_ = cb_timer; }

  // Constructor
  FreeFlyerTimer() : one_shot_(false),
    created_(false) {}

  // Destructor
  ~FreeFlyerTimer() {}

  void createTimer(double period_sec,
                   TimerCallbackType cb_timer,
                   rclcpp::Node::SharedPtr node,
                   bool one_shot = false,
                   bool auto_start = true) {
    createTimer(std::make_shared<rclcpp::Duration>(period_sec, 0),
                cb_timer,
                node,
                node->get_clock(),
                one_shot,
                auto_start);
  }

  void createTimer(double period_sec,
                   TimerCallbackType cb_timer,
                   rclcpp::Node::SharedPtr node,
                   rclcpp::Clock::SharedPtr clock,
                   bool one_shot = false,
                   bool auto_start = true) {
    createTimer(std::make_shared<rclcpp::Duration>(period_sec, 0),
                cb_timer,
                node,
                clock,
                one_shot,
                auto_start);
  }

  void createTimer(std::shared_ptr<rclcpp::Duration> period,
                   TimerCallbackType cb_timer,
                   rclcpp::Node::SharedPtr node,
                   bool one_shot = false,
                   bool auto_start = true) {
    createTimer(period,
                cb_timer,
                node,
                node->get_clock(),
                one_shot,
                auto_start);
  }

  void createTimer(std::shared_ptr<rclcpp::Duration> period,
                   TimerCallbackType cb_timer,
                   rclcpp::Node::SharedPtr node,
                   rclcpp::Clock::SharedPtr clock,
                   bool one_shot = false,
                   bool auto_start = true) {
    period_ = period;
    cb_timer_ = cb_timer;
    one_shot_ = one_shot;
    clock_ = clock;
    node_ = node;
    if (auto_start) {
      createStartTimer();
    }
  }

  void reset() {
    if (created_) {
      timer_->reset();
    }
  }

  // The ROS1 timer has a reset parameter that was set to true by default.
  // ARS never passed false for reset so the following two functions don't
  // take a reset argument and implement the reset is true functionality
  // from ROS1
  void setPeriod(double period_sec) {
    setPeriod(std::make_shared<rclcpp::Duration>(period_sec, 0));
  }

  void setPeriod(std::shared_ptr<rclcpp::Duration> period) {
    period_ = period;
    // See if the timer has been created
    if (created_) {
      // Check if the timer is stopped
      if (timer_->is_canceled()) {
        // Don't want to start the timer if it isn't running
        created_ = false;
      } else {
        timer_->cancel();
        createStartTimer();
      }
    }
  }

  // Follows the ros1 one convention that it doesn't do anything if the timer
  // is already started
  void start() {
    if (!created_) {
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
                                  *period_,
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
  bool one_shot_, created_;
  std::shared_ptr<rclcpp::Duration> period_;
  TimerCallbackType cb_timer_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ff_util

#endif  // FF_UTIL_FF_TIMER_H_
