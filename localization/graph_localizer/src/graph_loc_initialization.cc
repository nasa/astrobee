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

#include <graph_localizer/graph_loc_initialization.h>
#include <graph_localizer/parameter_reader.h>
#include <graph_localizer/utilities.h>
#include <localization_common/utilities.h>

#include <iostream>

namespace graph_localizer {
namespace lc = localization_common;
void GraphLocInitialization::SetBiases(const Eigen::Vector3d& accelerometer_bias, const Eigen::Vector3d& gyro_bias) {
  params_.graph_initialization.initial_imu_bias = gtsam::imuBias::ConstantBias(accelerometer_bias, gyro_bias);
  has_biases_ = true;
  estimate_biases_ = false;
  RemoveGravityFromBiasIfPossibleAndNecessary();
}

void GraphLocInitialization::SetStartPose(const Eigen::Isometry3d& global_T_body_start, const double timestamp) {
  params_.graph_initialization.start_time = timestamp;
  params_.graph_initialization.global_T_body_start = lc::GtPose(global_T_body_start);
  has_start_pose_ = true;
  RemoveGravityFromBiasIfPossibleAndNecessary();
}

void GraphLocInitialization::RemoveGravityFromBiasIfPossibleAndNecessary() {
  if (RemovedGravityFromBiasIfNecessary() || !HasParams()) return;
  if (params_.graph_initialization.gravity.isZero()) {
    removed_gravity_from_bias_if_necessary_ = true;
    return;
  }
  if (!HasStartPose() || !HasBiases()) return;
  // Biases, start pose and params are available and gravity is non zero, gravity can and should now be removed
  // from the initial bias estimates.
  VLOG(2) << "RemoveGravityFromBiasIfPossibleAndNecessary: Removing gravity from initial biases.";
  RemoveGravityFromBias(params_.graph_initialization.gravity, params_.graph_initialization.body_T_imu,
                        params_.graph_initialization.global_T_body_start,
                        params_.graph_initialization.initial_imu_bias);
  removed_gravity_from_bias_if_necessary_ = true;
  return;
}

void GraphLocInitialization::ResetBiasesAndStartPose() {
  ResetBiases();
  ResetStartPose();
}

void GraphLocInitialization::ResetStartPose() { has_start_pose_ = false; }

void GraphLocInitialization::ResetBiases() {
  has_biases_ = false;
  StartBiasEstimation();
}

void GraphLocInitialization::LoadGraphLocalizerParams(config_reader::ConfigReader& config) {
  graph_localizer::LoadGraphLocalizerParams(config, params_);
  has_params_ = true;
}

bool GraphLocInitialization::ReadyToInitialize() const {
  return HasBiases() && HasStartPose() && HasParams() && RemovedGravityFromBiasIfNecessary();
}

void GraphLocInitialization::StartBiasEstimation() { estimate_biases_ = true; }

bool GraphLocInitialization::HasBiases() const { return has_biases_; }
bool GraphLocInitialization::HasStartPose() const { return has_start_pose_; }
bool GraphLocInitialization::HasParams() const { return has_params_; }
bool GraphLocInitialization::EstimateBiases() const { return estimate_biases_; }
bool GraphLocInitialization::RemovedGravityFromBiasIfNecessary() const {
  return removed_gravity_from_bias_if_necessary_;
}

const GraphLocalizerParams& GraphLocInitialization::params() const { return params_; }

}  // namespace graph_localizer
