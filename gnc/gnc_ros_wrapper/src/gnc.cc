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

#include <gnc_ros_wrapper/gnc.h>

#include <ros/package.h>

namespace gnc_ros_wrapper {

GNC::GNC(ros::NodeHandle* nh, std::string const& platform_name,
  std::string const& name) : ekf_(&cmc_), ctl_(&cmc_, nh, name), fam_(nh) {
  config_.AddFile("gnc.config");
  config_.AddFile("cameras.config");
  config_.AddFile("geometry.config");
  ReadParams();
  ekf_.Initialize(nh, platform_name);  // have to wait until parameters are read or can have race condition
  // Initialize timers
  config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
      config_.CheckFilesUpdated(std::bind(&GNC::ReadParams, this));}, false, true);
  // Initialize performance timers
  pt_ekf_.Initialize("ekf");
  pt_ctl_.Initialize("ctl");
  pt_fam_.Initialize("fam");
}

GNC::~GNC() {}

void GNC::Run() {
  while (ros::ok()) {
    ros::spinOnce();
    // EKF
    pt_ekf_.Tick();
    if (!ekf_.Step())
      continue;
    pt_ekf_.Tock();
    // CTL
    pt_ctl_.Tick();
    ctl_.Step(ekf_.GetOutput());
    pt_ctl_.Tock();
    // FAM
    pt_fam_.Tick();
    fam_.Step(ctl_.GetTimeMsg(), ctl_.GetCmdMsg(), ctl_.GetCtlMsg());
    pt_fam_.Tock();
    // Send off timing measurements
    pt_ekf_.Send();
    pt_ctl_.Send();
    pt_fam_.Send();
  }
}

void GNC::ReadParams(void) {
  if (!config_.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }
  ekf_.ReadParams(&config_);
  ctl_.ReadParams(&config_);
  fam_.ReadParams(&config_);
}

}  // end namespace gnc_ros_wrapper
