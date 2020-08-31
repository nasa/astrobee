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

#ifndef FF_UTIL_FF_SERVICE_H_
#define FF_UTIL_FF_SERVICE_H_

// ROS includes
#include <ros/ros.h>

// C++11 includes
#include <functional>
#include <memory>
#include <string>

namespace ff_util {

//////////////////////////////////////// ACTION CLIENT CODE
///////////////////////////////////////////////////

// This is a simple wrapper around a ROS service client, which forces the
// connection to be persistent, and
// handles reconnection if the connection is dropped. This improves the
// performance of the service call as
// the connection is not established on each query, at the expense of a little
// more complexity.

template <class ServiceSpec>
class FreeFlyerServiceClient {
 protected:
  enum State {
    WAITING_FOR_CREATE  = 0,  // Connect() has not been called
    WAITING_FOR_CONNECT = 1,  // Waiting to connect to server
    WAITING_FOR_CALL    = 2   // Connection established, waiting on call
  };
  static constexpr double DEFAULT_TIMEOUT_CONNECTED = 10.0;
  static constexpr double DEFAULT_POLL_DURATION     = 0.1;

 public:
  // Callback types
  typedef std::function<void(void)> ConnectedCallbackType;
  typedef std::function<void(void)> TimeoutCallbackType;

  // Setters for callbacks
  void SetTimeoutCallback(TimeoutCallbackType cb_timeout) { cb_timeout_ = cb_timeout; }
  void SetConnectedCallback(ConnectedCallbackType cb_connected) { cb_connected_ = cb_connected; }

  // Setters for timeouts
  void SetConnectedTimeout(double to_connected) { to_connected_ = ros::Duration(to_connected); }

  // Constructor
  FreeFlyerServiceClient()
      : state_(WAITING_FOR_CREATE), to_connected_(DEFAULT_TIMEOUT_CONNECTED), to_poll_(DEFAULT_POLL_DURATION) {}

  // Destructor
  ~FreeFlyerServiceClient() {}

  // Try and connect to the service client, and return whether the connection is
  // active. In the blocking
  // case (NOT RECOMMENDED) if the server exists, then the return value should
  // be true. The non-blocking
  // case will always return false, and you will receive a callback when the
  // connection is ready.
  bool Create(ros::NodeHandle* nh, std::string const& topic) {
    // Create a timer to poll to see if the server is connected [autostart]
    timer_connected_ = nh->createTimer(to_connected_, &FreeFlyerServiceClient::TimeoutCallback, this, true, false);
    timer_poll_      = nh->createTimer(to_poll_, &FreeFlyerServiceClient::ConnectPollCallback, this, false, false);
    // Save the node handle and topic to support reconnects
    nh_    = nh;
    topic_ = topic;
    // Create a persistent connection to the service
    return IsConnected();
  }

  // Check that we are connected to the server
  bool IsConnected() {
    if (service_.isValid()) return true;
    ConnectPollCallback(ros::TimerEvent());
    return false;
  }

  // Call triggers a check for connection in order to prevent continual polling
  bool Call(ServiceSpec& service) {
    if (!IsConnected()) return false;
    return service_.call(service);
  }

 protected:
  // Simple wrapper around an optional timer
  void StartOptionalTimer(ros::Timer& timer, ros::Duration const& duration) {
    if (duration.isZero()) return;
    timer.stop();
    timer.setPeriod(duration);
    timer.start();
  }

  // Called periodically until the server is connected
  void ConnectPollCallback(ros::TimerEvent const& event) {
    // Case: connected
    if (service_.isValid()) {
      if (state_ != WAITING_FOR_CALL) {
        state_ = WAITING_FOR_CALL;
        timer_connected_.stop();
        timer_poll_.stop();
        if (cb_connected_) cb_connected_();
      }
      // Case: disconnected
    } else {
      service_ = nh_->serviceClient<ServiceSpec>(topic_, true);
      state_   = WAITING_FOR_CONNECT;
      StartOptionalTimer(timer_connected_, to_connected_);
      StartOptionalTimer(timer_poll_, to_poll_);
    }
  }

  // Called when the service doesn't go active or a response isn't received
  void TimeoutCallback(ros::TimerEvent const& event) {
    timer_connected_.stop();
    timer_poll_.stop();
    if (cb_timeout_) cb_timeout_();
  }

 protected:
  State                 state_;
  ros::Duration         to_connected_;
  ros::Duration         to_poll_;
  TimeoutCallbackType   cb_timeout_;
  ConnectedCallbackType cb_connected_;
  ros::ServiceClient    service_;
  ros::Timer            timer_connected_;
  ros::Timer            timer_poll_;
  std::string           topic_;
  ros::NodeHandle*      nh_;
};

}  // namespace ff_util

#endif  // FF_UTIL_FF_SERVICE_H_
