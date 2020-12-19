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
#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_PARAMS_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_PARAMS_H_

#include <graph_localizer/calibration_params.h>
#include <graph_localizer/factor_params.h>
#include <graph_localizer/feature_tracker_params.h>
#include <graph_localizer/graph_initialization_params.h>
#include <graph_localizer/graph_values_params.h>
#include <graph_localizer/noise_params.h>

#include <string>

namespace graph_localizer {
struct GraphLocalizerParams {
  CalibrationParams calibration;
  FactorParams factor;
  FeatureTrackerParams feature_tracker;
  GraphValuesParams graph_values;
  NoiseParams noise;
  GraphInitializationParams graph_initialization;
  bool verbose;
  bool fatal_failures;
  bool print_factor_info;
  bool use_ceres_params;
  int max_iterations;
  std::string marginals_factorization;
  bool limit_imu_factor_spacing;
  double max_imu_factor_spacing;
  bool add_priors;
  bool add_marginal_factors;
  double max_standstill_feature_track_avg_distance_from_mean;
  int standstill_min_num_points_per_track;
  double huber_k;
  int log_rate;
  bool estimate_world_T_dock_using_loc;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_PARAMS_H_
