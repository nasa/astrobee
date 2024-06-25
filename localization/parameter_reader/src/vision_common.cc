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

#include <parameter_reader/vision_common.h>
#include <msg_conversions/msg_conversions.h>

namespace parameter_reader {
namespace mc = msg_conversions;
namespace vc = vision_common;

void LoadStandstillParams(config_reader::ConfigReader& config, vc::StandstillParams& params,
                          const std::string& prefix) {
  LOAD_PARAM(params.min_num_points_per_track, config, prefix);
  LOAD_PARAM(params.duration, config, prefix);
  LOAD_PARAM(params.max_avg_distance_from_mean, config, prefix);
}

void LoadFeatureTrackerParams(config_reader::ConfigReader& config, vc::FeatureTrackerParams& params,
                              const std::string& prefix) {
  LOAD_PARAM(params.remove_undetected_feature_tracks, config, prefix);
}

void LoadSpacedFeatureTrackerParams(config_reader::ConfigReader& config, vc::SpacedFeatureTrackerParams& params,
                                    const std::string& prefix) {
  LoadFeatureTrackerParams(config, params, prefix);
  LOAD_PARAM(params.measurement_spacing, config, prefix);
}
}  // namespace parameter_reader
