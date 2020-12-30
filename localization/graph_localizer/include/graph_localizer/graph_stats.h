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
#ifndef GRAPH_LOCALIZER_GRAPH_STATS_H_
#define GRAPH_LOCALIZER_GRAPH_STATS_H_

#include <localization_common/averager.h>
#include <localization_common/timer.h>

namespace graph_localizer {
// Forward declaration since GraphLocalizer will have a GraphStats object
class GraphLocalizer;

class GraphStats {
 public:
  void UpdateErrors(const GraphLocalizer& graph);
  void UpdateStats(const GraphLocalizer& graph);
  // Log everything
  void Log() const;
  void LogToFile(std::ofstream& ofstream) const;
  void LogTimers() const;
  void LogStatsAveragers() const;
  void LogErrorAveragers() const;

  // Timers
  localization_common::Timer optimization_timer_ = localization_common::Timer("Optimization");
  localization_common::Timer update_timer_ = localization_common::Timer("Update");
  localization_common::Timer marginals_timer_ = localization_common::Timer("Marginals");
  localization_common::Timer slide_window_timer_ = localization_common::Timer("Slide Window");
  localization_common::Timer add_buffered_factors_timer_ = localization_common::Timer("Add Buffered Factors");
  localization_common::Timer log_error_timer_ = localization_common::Timer("Log Error");
  localization_common::Timer log_stats_timer_ = localization_common::Timer("Log Stats");

  // Graph Stats Averagers
  localization_common::Averager iterations_averager_ = localization_common::Averager("Iterations");
  localization_common::Averager num_states_averager_ = localization_common::Averager("Num States");
  localization_common::Averager duration_averager_ = localization_common::Averager("Duration");
  localization_common::Averager num_optical_flow_factors_averager_ =
    localization_common::Averager("Num Optical Flow Factors");
  localization_common::Averager num_loc_proj_factors_averager_ = localization_common::Averager("Num Loc Proj Factors");
  localization_common::Averager num_loc_pose_factors_averager_ = localization_common::Averager("Num Loc Pose Factors");
  localization_common::Averager num_imu_factors_averager_ = localization_common::Averager("Num Imu Factors");
  localization_common::Averager num_rotation_factors_averager_ = localization_common::Averager("Num Rotation Factors");
  localization_common::Averager num_standstill_between_factors_averager_ =
    localization_common::Averager("Num Standstill Between Factors");
  localization_common::Averager num_vel_prior_factors_averager_ =
    localization_common::Averager("Num Vel Prior Factors");
  localization_common::Averager num_marginal_factors_averager_ = localization_common::Averager("Num Marginal Factors");
  localization_common::Averager num_factors_averager_ = localization_common::Averager("Num Factors");
  localization_common::Averager num_features_averager_ = localization_common::Averager("Num Features");
  // Factor Error Averagers
  localization_common::Averager total_error_averager_ = localization_common::Averager("Total Factor Error");
  localization_common::Averager of_error_averager_ = localization_common::Averager("OF Factor Error");
  localization_common::Averager loc_proj_error_averager_ = localization_common::Averager("Loc Proj Factor Error");
  localization_common::Averager loc_pose_error_averager_ = localization_common::Averager("Loc Pose Factor Error");
  localization_common::Averager imu_error_averager_ = localization_common::Averager("Imu Factor Error");
  localization_common::Averager rotation_error_averager_ = localization_common::Averager("Rotation Factor Error");
  localization_common::Averager standstill_between_error_averager_ =
    localization_common::Averager("Standstill Between Error");
  localization_common::Averager pose_prior_error_averager_ = localization_common::Averager("Pose Prior Error");
  localization_common::Averager velocity_prior_error_averager_ = localization_common::Averager("Velocity Prior Error");
  localization_common::Averager bias_prior_error_averager_ = localization_common::Averager("Bias Prior Error");
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_STATS_H_
