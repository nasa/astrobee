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

#include <localization_manager/localization_pipeline_no.h>

namespace localization_manager {

NOPipeline::NOPipeline(ros::NodeHandle *nh, ros::NodeHandle *nhp,
  uint8_t mode, PipelineCallbackType cb)
  : Pipeline(mode, cb, "no", "No localization", false, false), nh_(nh) {
  // Create a private nodehandle
  ros::NodeHandle nh_pvt_(*nhp, GetName());
  // Initialize configuration manager
  cfg_.Initialize(&nh_pvt_, "localization/localization_pipeline_no.config");
  cfg_.Listen(boost::bind(&NOPipeline::ReconfigureCallback, this, _1));
}

bool NOPipeline::ReconfigureCallback(dynamic_reconfigure::Config &config) {
  return cfg_.Reconfigure(config);
}

bool NOPipeline::Enable(bool enable) {
  return true;
}

}  // namespace localization_manager
