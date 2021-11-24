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

#include <localization_common/logger.h>
#include <msg_conversions/msg_conversions.h>
#include <vision_common/parameter_reader.h>

namespace vision_common {
namespace mc = msg_conversions;

void LoadBriskFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                              BriskFeatureDetectorAndMatcherParams& params) {
  params.brisk_threshold = mc::LoadInt(config, "brisk_threshold");
  params.brisk_octaves = mc::LoadInt(config, "brisk_octaves");
  params.brisk_float_pattern_scale = mc::LoadFloat(config, "brisk_float_pattern_scale");
  params.max_match_hamming_distance = mc::LoadInt(config, "brisk_max_match_hamming_distance");
  params.flann_table_number = mc::LoadInt(config, "brisk_flann_table_number");
  params.flann_key_size = mc::LoadInt(config, "brisk_flann_key_size");
  params.flann_multi_probe_level = mc::LoadInt(config, "brisk_flann_multi_probe_level");
}

void LoadLKOpticalFlowFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                                      LKOpticalFlowFeatureDetectorAndMatcherParams& params) {
  params.max_iterations = mc::LoadInt(config, "lk_max_iterations");
  params.termination_epsilon = mc::LoadDouble(config, "lk_termination_epsilon");
  params.window_width = mc::LoadInt(config, "lk_window_width");
  params.window_height = mc::LoadInt(config, "lk_window_height");
  params.max_level = mc::LoadInt(config, "lk_max_level");
  params.min_eigen_threshold = mc::LoadDouble(config, "lk_min_eigen_threshold");
  params.max_flow_distance = mc::LoadDouble(config, "lk_max_flow_distance");
  params.max_backward_match_distance = mc::LoadDouble(config, "lk_max_backward_match_distance");
}

void LoadSurfFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                             SurfFeatureDetectorAndMatcherParams& params) {
  params.surf_threshold = mc::LoadInt(config, "surf_threshold");
  params.max_match_distance = mc::LoadDouble(config, "surf_max_match_distance");
}
}  // namespace vision_common
