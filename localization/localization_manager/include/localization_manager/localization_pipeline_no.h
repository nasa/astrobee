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

#ifndef LOCALIZATION_MANAGER_LOCALIZATION_PIPELINE_NO_H_
#define LOCALIZATION_MANAGER_LOCALIZATION_PIPELINE_NO_H_

#include <localization_manager/localization_pipeline.h>

// FSW
#include <ff_util/ff_service.h>
#include <ff_util/config_server.h>

namespace localization_manager {

class NOPipeline : public Pipeline {
 public:
  // Create the pipeline
  NOPipeline(ros::NodeHandle *nh, ros::NodeHandle *nhp,
    uint8_t mode, PipelineCallbackType cb);

  // Enable the pipeline (turn it on and off)
  bool Enable(bool enable);

 protected:
  // Callback when the pipeline gets reconfigured
  bool ReconfigureCallback(dynamic_reconfigure::Config &config);

 private:
  ros::NodeHandle *nh_;
  ros::NodeHandle nh_pvt_;
  ff_util::ConfigServer cfg_;
};

}  // namespace localization_manager

#endif  // LOCALIZATION_MANAGER_LOCALIZATION_PIPELINE_NO_H_

