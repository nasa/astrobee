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

#ifndef LOCALIZATION_MANAGER_LOCALIZATION_PIPELINE_H_
#define LOCALIZATION_MANAGER_LOCALIZATION_PIPELINE_H_

// ROS
#include <rclcpp/rclcpp.hpp>

// Message types
#include <ff_msgs/msg/camera_registration.hpp>
#include <ff_msgs/msg/visual_landmarks.hpp>
#include <ff_msgs/msg/depth_landmarks.hpp>
#include <ff_msgs/msg/localization_pipeline.hpp>
#include <ff_msgs/msg/ekf_state.hpp>
#include <ff_msgs/srv/set_bool.hpp>
namespace ff_msgs {
  typedef msg::CameraRegistration CameraRegistration;
  typedef msg::VisualLandmarks VisualLandmarks;
  typedef msg::DepthLandmarks DepthLandmarks;
  typedef msg::LocalizationPipeline LocalizationPipeline;
  typedef msg::EkfState EkfState;
  typedef srv::SetBool SetBool;
}  // namespace ff_msgs

// C++ includes
#include <functional>
#include <string>
#include <map>

namespace localization_manager {

enum PipelineError : uint8_t {
  ERROR_REGISTRATION_TIMEOUT    = 0,      // Registration timeout
  ERROR_VISUAL_TIMEOUT          = 1,      // Visual feature timeout
  ERROR_DEPTH_TIMEOUT           = 2,      // Depth timeout
  ERROR_FILTER_TIMEOUT          = 3,      // Filter timeout
  ERROR_FILTER_BIAS             = 4       // Filter is sampling the bias
};

typedef std::function<void(PipelineError)> PipelineCallbackType;

struct Pipeline {
  enum Components : uint8_t {
    COMPONENT_FILTER            = (1<<0),    // Filter needed
    COMPONENT_OPTICAL_FLOW      = (1<<1),    // Optical flow needed
    COMPONENT_ENABLE            = (1<<2),    // Needs optical flow
    COMPONENT_SET_INPUT         = (1<<3),    // Needs EKF input to be set
    COMPONENT_REGISTRATIONS     = (1<<4),    // Needs registration pulses
    COMPONENT_VISUAL_FEATURES   = (1<<5),    // Needs viusal features
    COMPONENT_DEPTH_FEATURES    = (1<<6)     // Needs depth features
  };

  // Constructor
  Pipeline(uint8_t mode, std::string const& name)
    : mode_(mode), components_(0), name_(name) {}

  // If the pipeline requires
  Pipeline& NeedsFilter(uint32_t max_conf, bool optical_flow, double timeout) {
    components_ |= COMPONENT_FILTER;
    max_filter_ = max_conf;
    if (optical_flow)
      components_ |= COMPONENT_OPTICAL_FLOW;
    timeout_filter_ = timeout;
    return *this;
  }

  // Needs to be enabled/disabled
  Pipeline& NeedsEnable(std::string const& topic, double timeout) {
    components_ |= COMPONENT_ENABLE;
    topic_enable_ = topic;
    timeout_enable_ = timeout;
    return *this;
  }

  // Needs measurements to be streamed
  Pipeline& NeedsRegistrations(std::string const& topic, double timeout) {
    components_ |= COMPONENT_REGISTRATIONS;
    topic_reg_ = topic;
    timeout_reg_ = timeout;
    return *this;
  }

  // Needs features
  Pipeline& NeedsVisualFeatures(
    std::string const& topic, double timeout, uint32_t min_features) {
    components_ |= COMPONENT_VISUAL_FEATURES;
    topic_visual_ = topic;
    timeout_visual_ = timeout;
    min_visual_ = min_features;
    return *this;
  }

  // Needs features
  Pipeline& NeedsDepthFeatures(
    std::string const& topic, double timeout, uint32_t min_features) {
    components_ |= COMPONENT_DEPTH_FEATURES;
    topic_depth_ = topic;
    timeout_depth_ = timeout;
    min_depth_ = min_features;
    return *this;
  }

  // Get the name of the pipeline
  std::string const& GetTopic() {
    return topic_enable_;
  }

  // Get the name of the pipeline
  std::string const& GetName() {
    return name_;
  }

  // Get the name of the pipeline
  uint8_t GetMode() {
    return mode_;
  }

  // Get whether the pipeline needs optical flow
  bool RequiresOpticalFlow() {
    return (components_ & COMPONENT_OPTICAL_FLOW);
  }

  // Get whether the pipeline needs optical flow
  bool RequiresFilter() {
    return (components_ & COMPONENT_FILTER);
  }

  // Initialize the pipeline
  bool Initialize(rclcpp::Node::SharedPtr nh,
    PipelineCallbackType cb_error,
    std::function<void(void)> cb_connected,
    std::function<void(void)> cb_timeout) {
    // Copy the callback and node handle
    callback_ = cb_error;
    nh_ = nh;
    // If we need to enable
    if (components_ & COMPONENT_ENABLE) {
      service_.SetConnectedTimeout(timeout_enable_);
      service_.SetConnectedCallback(cb_connected);
      service_.SetTimeoutCallback(cb_timeout);
      service_.Create(nh, topic_enable_);
    }
    // Disable by default
    return true;
  }

  // Check if the pipeline is initialized / connected to its services
  bool IsConnected() {
    if ((components_ & COMPONENT_ENABLE) && !service_.IsConnected())
      return false;
    return true;
  }

  // Enable or disable this pipeline
  bool Toggle(bool enable) {
    // If we are enabling this pipeline
    if (enable) {
      if (components_ & COMPONENT_REGISTRATIONS) {
        sub_reg_ = FF_CREATE_SUBSCRIBER(nh_, ff_msgs::CameraRegistration, topic_reg_, 1,
          std::bind(&Pipeline::RegistrationCallback, this, std::placeholders::_1));
        timer_reg_.createTimer(timeout_reg_,
          std::bind(&Pipeline::RegistrationTimeoutCallback, this), nh_);
      }
      if (components_ & COMPONENT_VISUAL_FEATURES) {
        sub_visual_ = FF_CREATE_SUBSCRIBER(nh_, ff_msgs::VisualLandmarks, topic_visual_, 1,
          std::bind(&Pipeline::VisualCallback, this, std::placeholders::_1));
        timer_visual_.createTimer(timeout_visual_,
          std::bind(&Pipeline::VisualTimeoutCallback, this), nh_, false, false);
      }
      if (components_ & COMPONENT_DEPTH_FEATURES) {
        sub_depth_ = FF_CREATE_SUBSCRIBER(nh_, ff_msgs::DepthLandmarks, topic_depth_, 1,
          std::bind(&Pipeline::DepthCallback, this, std::placeholders::_1));
        timer_depth_.createTimer(timeout_depth_,
          std::bind(&Pipeline::DepthTimeoutCallback, this), nh_);
      }
    } else {
      // Make sure we don't keep monitoring the filter when the pipeline
      // is shutdown.
      Use(false);
      // Stop monitoring the raw pipeline stream.
      if (components_ & COMPONENT_VISUAL_FEATURES) {
        sub_visual_.reset();
        timer_visual_.stop();
      }
      if (components_ & COMPONENT_DEPTH_FEATURES) {
        sub_depth_.reset();
        timer_depth_.stop();
      }
      if (components_ & COMPONENT_REGISTRATIONS) {
        sub_reg_.reset();
        timer_reg_.stop();
      }
    }
    // Enable or disable the pipeline if needed
    if (components_ & COMPONENT_ENABLE) {
      ff_msgs::SetBool::Request req;
      req.enable = enable;
      auto res = std::make_shared<ff_msgs::SetBool::Response>();
      if (!service_.Call(req, res))
        return false;
    }
    return true;
  }

  // Start actually using the pipeline
  bool Use(bool enable) {
    // If we are enabling this pipeline
    if (enable) {
      if (components_ & COMPONENT_FILTER) {
        sub_filter_ = FF_CREATE_SUBSCRIBER(nh_, ff_msgs::EkfState, TOPIC_GNC_EKF, 1,
          std::bind(&Pipeline::FilterCallback, this, std::placeholders::_1));
        timer_filter_.createTimer(timeout_filter_,
          std::bind(&Pipeline::FilterTimeoutCallback, this), nh_, false, false);
      }
    } else {
      if (components_ & COMPONENT_FILTER) {
        sub_filter_.reset();
        timer_filter_.stop();
      }
    }
    return true;
  }

 protected:
  // Called back when registration pulses arrive for processing
  void RegistrationCallback(const std::shared_ptr<ff_msgs::CameraRegistration> msg) {
    timer_reg_.stop();
    timer_reg_.start();
  }

  // Called when registration pulses don't arrive
  void RegistrationTimeoutCallback() {
    callback_(ERROR_REGISTRATION_TIMEOUT);
  }

  // Called back when visual features arrive for processing
  void VisualCallback(const std::shared_ptr<ff_msgs::VisualLandmarks> msg) {
    if (msg->landmarks.size() < min_visual_)
      return;
    timer_visual_.stop();
    timer_visual_.start();
  }

  // Called when visual features don't arrive
  void VisualTimeoutCallback() {
    callback_(ERROR_VISUAL_TIMEOUT);
  }

  // Called back when depth features arrive for processing
  void DepthCallback(const std::shared_ptr<ff_msgs::DepthLandmarks> msg) {
    if (msg->landmarks.size() < min_depth_)
      return;
    timer_depth_.stop();
    timer_depth_.start();
  }

  // Called when depth features don't arrive
  void DepthTimeoutCallback() {
    callback_(ERROR_DEPTH_TIMEOUT);
  }

  // Called when filter estimates arrive
  void FilterCallback(const std::shared_ptr<ff_msgs::EkfState> msg) {
    if (msg->estimating_bias)
      callback_(ERROR_FILTER_BIAS);
    else if (msg->confidence > max_filter_)
      return;
    timer_filter_.stop();
    timer_filter_.start();
  }

  // Called when filter estimates don't arrive
  void FilterTimeoutCallback() {
    callback_(ERROR_FILTER_TIMEOUT);
  }

 private:
  uint8_t mode_, components_;
  std::string name_, topic_enable_, topic_reg_, topic_visual_, topic_depth_;
  double timeout_reg_, timeout_visual_, timeout_depth_, timeout_enable_, timeout_filter_;
  rclcpp::Subscription<ff_msgs::VisualLandmarks>::SharedPtr sub_visual_;
  rclcpp::Subscription<ff_msgs::DepthLandmarks>::SharedPtr sub_depth_;
  rclcpp::Subscription<ff_msgs::CameraRegistration>::SharedPtr sub_reg_;
  rclcpp::Subscription<ff_msgs::EkfState>::SharedPtr sub_filter_;
  uint32_t min_visual_, min_depth_, max_filter_;
  ff_util::FreeFlyerServiceClient<ff_msgs::SetBool> service_;
  rclcpp::Node::SharedPtr nh_;
  ff_util::FreeFlyerTimer timer_visual_, timer_filter_, timer_depth_, timer_reg_;
  PipelineCallbackType callback_;
};


// Shorthand for id:pipeline relation
typedef std::map<std::string, Pipeline> PipelineMap;

}  // namespace localization_manager

#endif  // LOCALIZATION_MANAGER_LOCALIZATION_PIPELINE_H_
