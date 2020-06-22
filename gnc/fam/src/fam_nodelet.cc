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

#include <fam/fam.h>

#include <ff_common/init.h>
#include <ff_util/ff_nodelet.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <gnc_autocode/autocode.h>

#include <memory>

namespace fam {

class FamNodelet : public ff_util::FreeFlyerNodelet {
 public:
  FamNodelet() : ff_util::FreeFlyerNodelet(NODE_FAM, true) {}
  ~FamNodelet() {}
  // This is called when the nodelet is loaded into the nodelet manager
  void Initialize(ros::NodeHandle *nh) {
    // Bootstrap our environment
    ff_common::InitFreeFlyerApplication(getMyArgv(), false);
    gnc_autocode::InitializeAutocode(this);
    fam_.reset(new fam::Fam(this->GetPlatformHandle(true)));
  }

 private:
  std::shared_ptr<fam::Fam> fam_;
};

}  // end namespace fam

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(fam::FamNodelet, nodelet::Nodelet);
