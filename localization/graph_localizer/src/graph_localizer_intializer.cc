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

#include <graph_localizer/graph_localizer_initializer.h>
#include <graph_localizer/parameter_reader.h>
#include <graph_localizer/utilities.h>
#include <localization_common/utilities.h>

#include <iostream>
#include <string>

namespace graph_localizer {
namespace lc = localization_common;
namespace lm = localization_measurements;
GraphLocalizerInitializer::GraphLocalizerInitializer()
      has_start_pose_(false),
      has_params_(false),

void GraphLocalizerInitializer::SetStartPose(const lm::TimestampedPose& timestamped_pose) {
  params_.graph_initializer.start_time = timestamped_pose.time;
  params_.graph_initializer.global_T_body_start = timestamped_pose.pose;
  has_start_pose_ = true;
}

void GraphLocalizerInitializer::ResetStartPose() { has_start_pose_ = false; }

void GraphLocalizerInitializer::LoadGraphLocalizerParams(config_reader::ConfigReader& config) {
  graph_localizer::LoadGraphLocalizerParams(config, params_);
  has_params_ = true;
}

bool GraphLocalizerInitializer::ReadyToInitialize() const {
  return HasStartPose() && HasParams();
}

bool GraphLocalizerInitializer::HasStartPose() const { return has_start_pose_; }

bool GraphLocalizerInitializer::HasParams() const { return has_params_; }

const GraphLocalizerParams& GraphLocalizerInitializer::params() const { return params_; }
}  // namespace graph_localizer
