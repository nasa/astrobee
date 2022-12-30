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
#ifndef GRAPH_VIO_GRAPH_VIO_STATS_H_
#define GRAPH_VIO_GRAPH_VIO_STATS_H_

#include <graph_vio/combined_nav_state_graph_values.h>
#include <graph_optimizer/graph_stats.h>

namespace graph_vio {
class GraphVIOStats : public graph_optimizer::GraphStats {
 public:
  GraphVIOStats();
  void SetCombinedNavStateGraphValues(
    std::shared_ptr<const CombinedNavStateGraphValues> combined_nav_state_graph_values);
  void UpdateErrors(const gtsam::NonlinearFactorGraph& graph_factors) final;
  void UpdateStats(const gtsam::NonlinearFactorGraph& graph_factors) final;

  // Graph Stats Averagers
  localization_common::Averager num_states_averager_ = localization_common::Averager("Num States");
  localization_common::Averager duration_averager_ = localization_common::Averager("Duration");
  localization_common::Averager num_marginal_factors_averager_ = localization_common::Averager("Num Marginal Factors");
  localization_common::Averager num_factors_averager_ = localization_common::Averager("Num Factors");
  localization_common::Averager num_features_averager_ = localization_common::Averager("Num Features");
  localization_common::Averager num_depth_odometry_rel_pose_factors_averager_ =
    localization_common::Averager("Num Depth Odometry Rel Pose Factors");
  localization_common::Averager num_depth_odometry_rel_point_factors_averager_ =
    localization_common::Averager("Num Depth Odometry Rel Point Factors");
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
  // Factor Error Averagers
  localization_common::Averager depth_odom_rel_pose_error_averager_ =
    localization_common::Averager("Depth Odom Rel Pose Factor Error");
  localization_common::Averager depth_odom_rel_point_error_averager_ =
    localization_common::Averager("Depth Odom Rel Point Factor Error");
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

 private:
  std::shared_ptr<const CombinedNavStateGraphValues> combined_nav_state_graph_values_;
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_GRAPH_VIO_STATS_H_
