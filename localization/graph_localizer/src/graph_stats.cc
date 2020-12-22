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
#include <graph_localizer/graph_localizer.h>
#include <graph_localizer/graph_stats.h>
#include <graph_localizer/loc_projection_factor.h>
#include <graph_localizer/loc_pose_factor.h>
#include <graph_localizer/pose_rotation_factor.h>

#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace graph_localizer {
void GraphStats::UpdateErrors(const GraphLocalizer& graph) {
  log_error_timer_.Start();
  double total_error = 0;
  double optical_flow_factor_error = 0;
  double loc_proj_error = 0;
  double imu_factor_error = 0;
  double rotation_factor_error = 0;
  double standstill_between_factor_error = 0;
  double pose_prior_error = 0;
  double velocity_prior_error = 0;
  double bias_prior_error = 0;
  for (const auto& factor : graph.factor_graph()) {
    const double error = factor->error(graph.graph_values().values());
    total_error += error;
    if (graph.params().factor.smart_projection_adder.enabled) {
      const auto smart_factor = dynamic_cast<const RobustSmartFactor*>(factor.get());
      if (smart_factor) {
        optical_flow_factor_error += error;
      }
    }
    if (graph.params().factor.projection_adder.enabled) {
      const auto projection_factor = dynamic_cast<const ProjectionFactor*>(factor.get());
      if (projection_factor) {
        optical_flow_factor_error += error;
      }
    }
    const auto imu_factor = dynamic_cast<gtsam::CombinedImuFactor*>(factor.get());
    if (imu_factor) {
      imu_factor_error += error;
    }
    const auto loc_factor = dynamic_cast<gtsam::LocProjectionFactor<>*>(factor.get());
    const auto loc_pose_factor = dynamic_cast<gtsam::LocPoseFactor*>(factor.get());
    if (loc_factor || loc_pose_factor) {
      loc_proj_error += error;
    }
    const auto rotation_factor = dynamic_cast<gtsam::PoseRotationFactor*>(factor.get());
    if (rotation_factor) {
      rotation_factor_error += error;
    }
    const auto standstill_between_factor = dynamic_cast<gtsam::BetweenFactor<gtsam::Pose3>*>(factor.get());
    if (standstill_between_factor) {
      standstill_between_factor_error += error;
    }

    // Prior Factors
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factor.get());
    if (pose_prior_factor && !loc_pose_factor) {
      pose_prior_error += error;
    }
    const auto velocity_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Velocity3>*>(factor.get());
    if (velocity_prior_factor) {
      velocity_prior_error += error;
    }
    const auto bias_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>*>(factor.get());
    if (bias_prior_factor) {
      bias_prior_error += error;
    }
  }
  total_error_averager_.Update(total_error);
  of_error_averager_.Update(optical_flow_factor_error);
  loc_proj_error_averager_.Update(loc_proj_error);
  imu_error_averager_.Update(imu_factor_error);
  rotation_error_averager_.Update(rotation_factor_error);
  standstill_between_error_averager_.Update(standstill_between_factor_error);
  pose_prior_error_averager_.Update(pose_prior_error);
  velocity_prior_error_averager_.Update(velocity_prior_error);
  bias_prior_error_averager_.Update(bias_prior_error);
  log_error_timer_.Stop();
}

void GraphStats::UpdateStats(const GraphLocalizer& graph) {
  log_stats_timer_.Start();
  num_states_averager_.Update(graph.graph_values().NumStates());
  duration_averager_.Update(graph.graph_values().Duration());
  num_optical_flow_factors_averager_.Update(graph.NumOFFactors());
  num_loc_factors_averager_.Update(graph.NumVLFactors());
  num_imu_factors_averager_.Update(graph.NumFactors<gtsam::CombinedImuFactor>());
  num_rotation_factors_averager_.Update(graph.NumFactors<gtsam::PoseRotationFactor>());
  num_standstill_between_factors_averager_.Update(graph.NumFactors<gtsam::BetweenFactor<gtsam::Pose3>>());
  num_vel_prior_factors_averager_.Update(graph.NumFactors<gtsam::PriorFactor<gtsam::Velocity3>>());
  num_marginal_factors_averager_.Update(graph.NumFactors<gtsam::LinearContainerFactor>());
  if (graph.params().factor.projection_adder.enabled) num_features_averager_.Update(graph.NumFeatures());
  num_factors_averager_.Update(graph.factor_graph().size());
  log_stats_timer_.Stop();
}

void GraphStats::Log() {
  LogTimers();
  LogStatsAveragers();
  LogErrorAveragers();
}

void GraphStats::LogTimers() {
  optimization_timer_.Log();
  update_timer_.Log();
  marginals_timer_.Log();
  slide_window_timer_.Log();
  add_buffered_factors_timer_.Log();
  log_error_timer_.Log();
  log_stats_timer_.Log();
}

void GraphStats::LogStatsAveragers() {
  iterations_averager_.Log();
  num_states_averager_.Log();
  duration_averager_.Log();
  num_optical_flow_factors_averager_.Log();
  num_loc_factors_averager_.Log();
  num_imu_factors_averager_.Log();
  num_rotation_factors_averager_.Log();
  num_standstill_between_factors_averager_.Log();
  num_vel_prior_factors_averager_.Log();
  num_marginal_factors_averager_.Log();
  num_factors_averager_.Log();
  num_features_averager_.Log();
}

void GraphStats::LogErrorAveragers() {
  total_error_averager_.Log();
  of_error_averager_.Log();
  loc_proj_error_averager_.Log();
  imu_error_averager_.Log();
  rotation_error_averager_.Log();
  standstill_between_error_averager_.Log();
  pose_prior_error_averager_.Log();
  velocity_prior_error_averager_.Log();
  bias_prior_error_averager_.Log();
}
}  // namespace graph_localizer
