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
#ifndef GRAPH_LOCALIZER_FACTOR_PARAMS_H_
#define GRAPH_LOCALIZER_FACTOR_PARAMS_H_

#include <string>

namespace graph_localizer {
struct FactorParams {
  // Optical flow factors
  double min_valid_feature_track_avg_distance_from_mean;
  double max_standstill_feature_track_avg_distance_from_mean;
  bool optical_flow_standstill_velocity_prior;
  bool enable_EPI;
  double landmark_distance_threshold;
  double dynamic_outlier_rejection_threshold;
  double retriangulation_threshold;
  std::string degeneracy_mode;
  std::string linearization_mode;
  bool verbose_cheirality;
  bool robust_smart_factor;
  int max_num_optical_flow_factors;
  bool enable_rotation_only_fallback;
  bool smart_factor_splitting;
  bool use_smart_factors;
  // Loc factors
  bool loc_pose_priors;
  bool loc_projections;
  int min_num_matches;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_FACTOR_PARAMS_H_
