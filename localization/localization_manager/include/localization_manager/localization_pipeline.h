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
#include <ros/ros.h>

// FSW
#include <ff_util/ff_names.h>
#include <ff_util/ff_service.h>

// STL includes
#include <functional>
#include <string>

namespace localization_manager {

// Callback contains the name od the pipeline and the event type
typedef std::function<void(std::string const&, bool)> PipelineCallbackType;

// An abstract class representing a localization pipeline
class Pipeline {
 public:
  // Create a new pipeline
  Pipeline(uint8_t mode, PipelineCallbackType cb, std::string const& name, std::string const& desc,
    bool req_of = false) : mode_(mode), cb_(cb), name_(name), desc_(desc), req_of_(req_of) {
  }

  // Get the pipeline name
  std::string const& GetName() { return name_; }

  // Get the pipeline description
  std::string const& GetDesc() { return desc_; }

  // Get the pipeline EKF mode
  uint8_t GetMode() { return mode_; }

  // Does this pipeline need optical flow>
  bool NeedsOpticalFlow() { return req_of_; }

  // Enable the pipeline -- this is implementation specific
  virtual bool Enable(bool enable) = 0;

  // Start a watchdog timer
  static void StartTimer(ros::Timer & timer, double secs) {
    timer.stop();
    timer.setPeriod(ros::Duration(secs));
    timer.start();
  }

  // Stop a watchdog timer
  static void StopTimer(ros::Timer & timer) {
    timer.stop();
  }

 protected:
  // Push a localization event
  void MarkStable(bool stable) {
    if (cb_)
      cb_(name_, stable);
  }

 private:
  uint8_t mode_;
  PipelineCallbackType cb_;
  std::string name_;
  std::string desc_;
  bool req_of_;
};

}  // namespace localization_manager

#endif  // LOCALIZATION_MANAGER_LOCALIZATION_PIPELINE_H_
