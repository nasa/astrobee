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

#include <graph_localizer/combined_nav_state_node_updater.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/PriorFactor.h>

namespace graph_localizer {
namespace sym = gtsam::symbol_shorthand;
explicit CombinedNavStateNodeUpdater::CombinedNavStateNodeUpdater(const CombinedNavStateNodeUpdaterParams& params)
    : params_(params) {}

void CombinedNavStateNodeUpdater::AddInitialValuesAndPriors(const localization_common::CombinedNavState& global_N_body,
                                                            const localization_common::CombinedNavStateNoise& noise,
                                                            gtsam::NonlinearFactorGraph& graph,
                                                            GraphValues& graph_values) {
  // Assumes zero initial velocity
  const lc::CombinedNavState global_N_body_start(params_.graph_initializer.global_T_body_start,
                                                 gtsam::Velocity3::Zero(), params_.graph_initializer.initial_imu_bias,
                                                 params_.graph_initializer.start_time);

  // Add first nav state and priors to graph
  const int key_index = GenerateKeyIndex();
  graph_values_->AddCombinedNavState(global_N_body_start, key_index);

  const gtsam::Vector6 pose_prior_noise_sigmas(
    (gtsam::Vector(6) << params_.noise.starting_prior_translation_stddev,
     params_.noise.starting_prior_translation_stddev, params_.noise.starting_prior_translation_stddev,
     params_.noise.starting_prior_quaternion_stddev, params_.noise.starting_prior_quaternion_stddev,
     params_.noise.starting_prior_quaternion_stddev)
      .finished());
  const gtsam::Vector3 velocity_prior_noise_sigmas((gtsam::Vector(3) << params_.noise.starting_prior_velocity_stddev,
                                                    params_.noise.starting_prior_velocity_stddev,
                                                    params_.noise.starting_prior_velocity_stddev)
                                                     .finished());
  const gtsam::Vector6 bias_prior_noise_sigmas(
    (gtsam::Vector(6) << params_.noise.starting_prior_accel_bias_stddev, params_.noise.starting_prior_accel_bias_stddev,
     params_.noise.starting_prior_accel_bias_stddev, params_.noise.starting_prior_gyro_bias_stddev,
     params_.noise.starting_prior_gyro_bias_stddev, params_.noise.starting_prior_gyro_bias_stddev)
      .finished());
  lc::CombinedNavStateNoise noise;
  noise.pose_noise = Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_prior_noise_sigmas)), params_.huber_k);
  noise.velocity_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)),
           params_.huber_k);
  noise.bias_noise = Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(bias_prior_noise_sigmas)), params_.huber_k);
  AddPriors(global_N_body_start, noise, key_index, graph);
}

void CombinedNavStateNodeUpdater::AddPriors(const localization_common::CombinedNavState& global_N_body,
                                            const localization_common::CombinedNavStateNoise& noise,
                                            const GraphValues& graph_values, gtsam::NonlinearFactorGraph& factors) {
  const auto key_index = graph_values_.KeyIndex(global_N_body.timestamp());
  if (!key_index) {
    LogError("AddPriors: Failed to get key index.");
    return;
  }
  // TODO(rsoussan): Ensure symbols not used by other node updaters
  gtsam::PriorFactor<gtsam::Pose3> pose_prior_factor(sym::P(*key_index), global_N_body.pose(), noise.pose_noise);
  factors.push_back(pose_prior_factor);
  gtsam::PriorFactor<gtsam::Velocity3> velocity_prior_factor(sym::V(*key_index), global_N_body.velocity(),
                                                             noise.velocity_noise);
  factors.push_back(velocity_prior_factor);
  gtsam::PriorFactor<gtsam::imuBias::ConstantBias> bias_prior_factor(sym::B(*key_index), global_N_body.bias(),
                                                                     noise.bias_noise);
  factors.push_back(bias_prior_factor);
}

void CombinedNavStateNodeUpdater::AddFactors(const FactorToAdd& measurement, gtsam::NonlinearFactorGraph& graph,
                                             GraphValues& graph_values);

void CombinedNavStateNodeUpdater::SlideWindow(const localization_common::Timestamp oldest_allowed_timestamp,
                                              gtsam::NonlinearFactorGraph& factors, GraphValues& graph_values);
int CombinedNavStateNodeUpdater::GenerateKeyIndex() { return key_index++; }
}  // namespace graph_localizer
