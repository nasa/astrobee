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

#ifndef LOCALIZATION_MANAGER_LOCALIZATION_PIPELINE_AR_H_
#define LOCALIZATION_MANAGER_LOCALIZATION_PIPELINE_AR_H_

#include <localization_manager/localization_pipeline.h>

// FSW messages
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/VisualLandmarks.h>
#include <ff_msgs/SetBool.h>

// FSW
#include <ff_util/ff_service.h>
#include <ff_util/config_server.h>

namespace localization_manager {

class ARPipeline : public Pipeline {
 public:
  // Create the pipeline
  ARPipeline(ros::NodeHandle *nh, ros::NodeHandle *nhp, uint8_t mode, PipelineCallbackType cb);

  // Enable the pipeline (turn it on and off)
  bool Enable(bool enable);

 protected:
  // Callback when the pipeline gets reconfigured
  bool ReconfigureCallback(dynamic_reconfigure::Config &config);

  // Callback when service connects
  void ConnectedCallback();

  // Callback when service times out
  void EnableTimeoutCallback();

  // Callback resets watchdog timer and performs some extra sanity checks
  void MeasurementCallback(ff_msgs::CameraRegistration::ConstPtr const& msg);

  // Callback resets watchdog timer and performs some extra sanity checks
  void FeatureCallback(ff_msgs::VisualLandmarks::ConstPtr const& msg);

  // Let the localization manager know that we didn't haev enough valid measurements
  void MeasurementWatchdog(ros::TimerEvent const& event);

  // Let the localization manager know that we didn't have enough v'
  void FeatureWatchdog(ros::TimerEvent const& event);

  // Let the localization manager know that we didn;t have enough events
  void StableCallback(ros::TimerEvent const& event);

  // Let the localization manager know that we didn;t have enough events
  void TimeoutCallback(ros::TimerEvent const& event);

 private:
  ros::NodeHandle *nh_;                                        // NodeHandle
  ros::NodeHandle nh_pvt_;                                     // Private NodeHandle
  ff_util::ConfigServer cfg_;                                  // Configuration server
  ff_util::FreeFlyerServiceClient<ff_msgs::SetBool> service_;  // EKF set input service
  ros::Subscriber sub_m_;                                      // Subscriber: measurements
  ros::Subscriber sub_f_;                                      // Subscriber: features
  ros::Timer timer_m_;                                         // Watchdog: measurements
  ros::Timer timer_f_;                                         // Watchdog: features
  ros::Timer timer_stable_;                                    // Stability timer
  ros::Timer timer_timeout_;                                   // Timeout timer
};

}  // namespace localization_manager

#endif  // LOCALIZATION_MANAGER_LOCALIZATION_PIPELINE_AR_H_

