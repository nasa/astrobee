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

#include <localization_manager/localization_pipeline_ar.h>

namespace localization_manager {

ARPipeline::ARPipeline(ros::NodeHandle *nh, ros::NodeHandle *nhp, uint8_t mode, PipelineCallbackType cb)
  : Pipeline(mode, cb, "ar", "AR tag localization", true, true), nh_(nh) {
  // Create a private nodehandle
  ros::NodeHandle nh_pvt_(*nhp, GetName());
  // Initialize configuration manager
  cfg_.Initialize(&nh_pvt_, "localization/localization_pipeline_ar.config");
  cfg_.Listen(boost::bind(&ARPipeline::ReconfigureCallback, this, _1));
  // Create the timers
  timer_m_ = nh->createTimer(ros::Duration(cfg_.Get<double>("timeout_measurements")),
    &ARPipeline::MeasurementWatchdog, this, false, false);
  timer_f_ = nh->createTimer(ros::Duration(cfg_.Get<double>("timeout_features")),
    &ARPipeline::FeatureWatchdog, this, false, false);
  // The time required to wait for stability before sending a switch event
  timer_stable_ = nh->createTimer(ros::Duration(cfg_.Get<double>("timeout_stability")),
    &ARPipeline::StableCallback, this, true, false);
  // Enable or disable the handrail detecttor
  service_.SetConnectedTimeout(cfg_.Get<double>("timeout_service_enable"));
  service_.SetTimeoutCallback(std::bind(&ARPipeline::EnableTimeoutCallback, this));
  service_.Create(nh, SERVICE_LOCALIZATION_AR_ENABLE);
}

bool ARPipeline::ReconfigureCallback(dynamic_reconfigure::Config &config) {
  if (!cfg_.Reconfigure(config)) {
    ROS_WARN_STREAM("Received an invalid reconfiguration request");
    return false;
  }
  return true;
}

void ARPipeline::EnableTimeoutCallback() {
  ROS_ERROR("SERVICE_LOCALIZATION_AR_ENABLE client timeout");
}

bool ARPipeline::Enable(bool enable) {
  // Enable or disable the AR feature detector
  ff_msgs::SetBool msg;
  msg.request.enable = enable;
  if (!service_.Call(msg))
    return false;
  // monitoring to ensure the system remains stable
  if (enable) {
    // Create the subscribers
    sub_m_ = nh_->subscribe(TOPIC_LOCALIZATION_AR_REGISTRATION,
      1, &ARPipeline::MeasurementCallback, this);
    sub_f_ = nh_->subscribe(TOPIC_LOCALIZATION_AR_FEATURES,
      1, &ARPipeline::FeatureCallback, this);
    // Enable watchdog timers with the given durations
    StartTimer(timer_m_, cfg_.Get<double>("timeout_measurements"));
    StartTimer(timer_f_, cfg_.Get<double>("timeout_features"));
    // The time required to wait for stability before sending a switch event
    StartTimer(timer_stable_, cfg_.Get<double>("timeout_stability"));
  } else {
    // The time required to wait for stability before sending a switch event
    StopTimer(timer_stable_);
    StopTimer(timer_timeout_);
    // Disable watchdog timers
    StopTimer(timer_m_);
    StopTimer(timer_f_);
    // Shutdown subscribers
    sub_m_.shutdown();
    sub_f_.shutdown();
  }
  // Success
  return true;
}

void ARPipeline::MeasurementCallback(ff_msgs::CameraRegistration::ConstPtr const& msg) {
  StartTimer(timer_m_, cfg_.Get<double>("timeout_measurements"));
}

void ARPipeline::FeatureCallback(ff_msgs::VisualLandmarks::ConstPtr const& msg) {
  if (static_cast<int>(msg->landmarks.size()) >= cfg_.Get<int>("minimum_features"))
    StartTimer(timer_f_, cfg_.Get<double>("timeout_features"));
}

void ARPipeline::MeasurementWatchdog(ros::TimerEvent const& event) {
  MarkStable(false);
  StartTimer(timer_stable_, cfg_.Get<double>("timeout_stability"));
}

void ARPipeline::FeatureWatchdog(ros::TimerEvent const& event) {
  MarkStable(false);
  StartTimer(timer_stable_, cfg_.Get<double>("timeout_stability"));
}

void ARPipeline::StableCallback(ros::TimerEvent const& event) {
  MarkStable(true);
  StartTimer(timer_stable_, cfg_.Get<double>("timeout_stability"));
}


}  // namespace localization_manager
