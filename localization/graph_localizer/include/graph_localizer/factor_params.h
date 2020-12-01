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

#include <graph_localizer/loc_factor_adder_params.h>
#include <graph_localizer/rotation_factor_adder_params.h>

#include <string>

namespace graph_localizer {
struct FactorParams {
  bool optical_flow_standstill_velocity_prior;
  RotationFactorAdderParams rotation_adder;
  // Optical flow factors
  double min_valid_feature_track_avg_distance_from_mean;
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
  bool add_smart_factors;
  // OF Projection Factors
  bool add_projection_factors;
  int min_num_measurements_for_triangulation;
  int max_num_optical_flow_features;
  bool add_point_priors;
  bool triangulation_enable_EPI;
  double triangulation_landmark_distance_threshold;
  double triangulation_dynamic_outlier_rejection_threshold;
  LocFactorAdderParams loc_adder;
  LocFactorAdderParams ar_tag_loc_adder;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_FACTOR_PARAMS_H_
