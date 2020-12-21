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

#include <graph_localizer/graph_localizer_initialization.h>
#include <graph_localizer/parameter_reader.h>
#include <graph_localizer/utilities.h>
#include <localization_common/utilities.h>

#include <iostream>

namespace graph_localizer {
namespace lc = localization_common;
void GraphLocalizerInitialization::SetBiases(const gtsam::imuBias::ConstantBias& imu_bias) {
  params_.graph_initialization.initial_imu_bias = imu_bias;
  has_biases_ = true;
  estimate_biases_ = false;
  RemoveGravityFromBiasIfPossibleAndNecessary();
}

void GraphLocalizerInitialization::SetStartPose(const gtsam::Pose3& global_T_body_start, const double timestamp) {
  params_.graph_initialization.start_time = timestamp;
  params_.graph_initialization.global_T_body_start = global_T_body_start;
  has_start_pose_ = true;
  RemoveGravityFromBiasIfPossibleAndNecessary();
}

void GraphLocalizerInitialization::RemoveGravityFromBiasIfPossibleAndNecessary() {
  if (RemovedGravityFromBiasIfNecessary() || !HasParams()) return;
  if (params_.graph_initialization.gravity.isZero()) {
    removed_gravity_from_bias_if_necessary_ = true;
    return;
  }
  if (!HasStartPose() || !HasBiases()) return;
  // Biases, start pose and params are available and gravity is non zero, gravity can and should now be removed
  // from the initial bias estimates.
  LogInfo("RemoveGravityFromBiasIfPossibleAndNecessary: Removing gravity from initial biases.");
  RemoveGravityFromBias(params_.graph_initialization.gravity, params_.graph_initialization.body_T_imu,
                        params_.graph_initialization.global_T_body_start,
                        params_.graph_initialization.initial_imu_bias);

  LogInfo("RemoveGravityFromBiasIfPossibleAndNecessary: New gravity corrected accelerometer bias: "
          << params_.graph_initialization.initial_imu_bias.accelerometer().matrix());
  removed_gravity_from_bias_if_necessary_ = true;
  return;
}

void GraphLocalizerInitialization::ResetBiasesAndStartPose() {
  ResetBiases();
  ResetStartPose();
}

void GraphLocalizerInitialization::ResetStartPose() { has_start_pose_ = false; }

void GraphLocalizerInitialization::ResetBiases() {
  has_biases_ = false;
  StartBiasEstimation();
}

void GraphLocalizerInitialization::LoadGraphLocalizerParams(config_reader::ConfigReader& config) {
  graph_localizer::LoadGraphLocalizerParams(config, params_);
  has_params_ = true;
}

bool GraphLocalizerInitialization::ReadyToInitialize() const {
  return HasBiases() && HasStartPose() && HasParams() && RemovedGravityFromBiasIfNecessary();
}

void GraphLocalizerInitialization::StartBiasEstimation() { estimate_biases_ = true; }

bool GraphLocalizerInitialization::HasBiases() const { return has_biases_; }
bool GraphLocalizerInitialization::HasStartPose() const { return has_start_pose_; }
bool GraphLocalizerInitialization::HasParams() const { return has_params_; }
bool GraphLocalizerInitialization::EstimateBiases() const { return estimate_biases_; }
bool GraphLocalizerInitialization::RemovedGravityFromBiasIfNecessary() const {
  return removed_gravity_from_bias_if_necessary_;
}

const GraphLocalizerParams& GraphLocalizerInitialization::params() const { return params_; }

}  // namespace graph_localizer
