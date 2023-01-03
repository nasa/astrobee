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
#ifndef GRAPH_VIO_GRAPH_VIO_PARAMS_H_
#define GRAPH_VIO_GRAPH_VIO_PARAMS_H_

#include <graph_vio/calibration_params.h>
#include <graph_vio/combined_nav_state_node_updater_params.h>
#include <graph_vio/factor_params.h>
#include <graph_vio/feature_point_node_updater_params.h>
#include <vision_common/feature_tracker_params.h>
#include <graph_vio/graph_initializer_params.h>
#include <graph_optimizer/graph_optimizer_params.h>
#include <localization_measurements/fan_speed_mode.h>

#include <string>

namespace graph_vio {
struct GraphVIOParams {
  CombinedNavStateNodeUpdaterParams combined_nav_state_node_updater;
  CalibrationParams calibration;
  FactorParams factor;
  FeaturePointNodeUpdaterParams feature_point_node_updater;
  FeatureTrackerParams feature_tracker;
  graph_optimizer::GraphOptimizerParams graph_optimizer;
  GraphInitializerParams graph_initializer;
  double max_standstill_feature_track_avg_distance_from_mean;
  int standstill_min_num_points_per_track;
  double huber_k;
  double standstill_feature_track_duration;
  localization_measurements::FanSpeedMode initial_fan_speed_mode;
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_GRAPH_VIO_PARAMS_H_
