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
#ifndef PARAMETER_READER_VISION_COMMON_H_
#define PARAMETER_READER_VISION_COMMON_H_

#include <config_reader/config_reader.h>
#include <vision_common/feature_tracker_params.h>
#include <vision_common/spaced_feature_tracker_params.h>
#include <vision_common/standstill_params.h>

#include <string>

namespace parameter_reader {
void LoadStandstillParams(config_reader::ConfigReader& config, vision_common::StandstillParams& params,
                          const std::string& prefix = "");

void LoadFeatureTrackerParams(config_reader::ConfigReader& config, vision_common::FeatureTrackerParams& params,
                              const std::string& prefix = "");

void LoadSpacedFeatureTrackerParams(config_reader::ConfigReader& config,
                                    vision_common::SpacedFeatureTrackerParams& params, const std::string& prefix = "");
}  // namespace parameter_reader

#endif  // PARAMETER_READER_VISION_COMMON_H_
