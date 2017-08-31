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

#include <common/init.h>
#include <ff_util/ff_nodelet.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <gnc_autocode/autocode.h>

#include <thread>
#include <memory>

namespace gnc_ros_wrapper {

class GncWrapperNodelet : public ff_util::FreeFlyerNodelet {
 public:
  GncWrapperNodelet() : ff_util::FreeFlyerNodelet(NODE_GNC, true) {}
  // This is called when the nodelet is loaded into the nodelet manager
  virtual void Initialize(ros::NodeHandle *nh) {
    // Bootstrap our environment
    common::InitFreeFlyerApplication(getMyArgv());
    gnc_autocode::InitializeAutocode(this);
    // Initialise GNC with a MT node handle that points to the
    // /platform/subsystem/node
    gnc_.reset(new gnc_ros_wrapper::GNC(GetPlatformHandle(true),
      GetPlatform(), getName()));
    // start a new thread to run everything
    thread_.reset(new std::thread(&gnc_ros_wrapper::GNC::Run, gnc_.get()));
  }

 private:
  std::shared_ptr<gnc_ros_wrapper::GNC> gnc_;
  std::shared_ptr<std::thread> thread_;
};

}  // end namespace gnc_ros_wrapper

// Declare the plugin
PLUGINLIB_DECLARE_CLASS(gnc_ros_wrapper, GncWrapperNodelet,
                        gnc_ros_wrapper::GncWrapperNodelet, nodelet::Nodelet);
