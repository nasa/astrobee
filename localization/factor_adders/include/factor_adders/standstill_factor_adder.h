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

#ifndef FACTOR_ADDERS_STANDSTILL_FACTOR_ADDER_H_
#define FACTOR_ADDERS_STANDSTILL_FACTOR_ADDER_H_

#include <factor_adders/single_measurement_based_factor_adder.h>
#include <factor_adders/standstill_factor_adder_params.h>
#include <localization_measurements/standstill_measurement.h>

#include <vector>

namespace factor_adders {
// SingleMeasurementBasedFactorAdder that generates standstill factors (zero velocity prior and zero relative pose
// between factors) based on provided params.
template <typename PoseVelocityNodeAdderType>
class StandstillFactorAdder
    : public SingleMeasurementBasedFactorAdder<localization_measurements::StandstillMeasurement> {
  using Base = SingleMeasurementBasedFactorAdder<localization_measurements::StandstillMeasurement>;

 public:
  StandstillFactorAdder(const StandstillFactorAdderParams& params,
                        const std::shared_ptr<PoseVelocityNodeAdderType> node_adder);

  // Adds zero velocity and/or zero relative pose factors depending on params.
  int AddFactors(const localization_measurements::StandstillMeasurement& standstill_measurement,
                 gtsam::NonlinearFactorGraph& factors) final;

 private:
  // Adds a velocity prior at the provided timestamp with zero velocity.
  bool AddZeroVelocityPrior(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors);
  // Adds a relative pose between factor with zero relative movement between nodes at timestamp a and b.
  bool AddZeroRelativePoseFactor(const localization_common::Time timestamp_a,
                                 const localization_common::Time timestamp_b, gtsam::NonlinearFactorGraph& factors);

  std::shared_ptr<PoseVelocityNodeAdderType> node_adder_;
  StandstillFactorAdderParams params_;
  gtsam::SharedNoiseModel zero_velocity_noise_;
  gtsam::SharedNoiseModel zero_relative_pose_noise_;
};

// Implementation
template <typename PoseVelocityNodeAdderType>
StandstillFactorAdder::StandstillFactorAdder(const StandstillFactorAdderParams& params,
                                             const std::shared_ptr<PoseVelocityNodeAdderType> node_adder)
    : Base(params), params_(params), node_adder_(node_adder) {
  // Create zero velocity noise
  const gtsam::Vector3 velocity_prior_noise_sigmas(
    (gtsam::Vector(3) << params().prior_velocity_stddev, params().prior_velocity_stddev, params().prior_velocity_stddev)
      .finished());
  zero_velocity_noise_ =
    go::Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)),
               params().huber_k);

  // Create zero relative pose noise
  const gtsam::Vector6 pose_between_noise_sigmas(
    (gtsam::Vector(6) << params().pose_between_factor_rotation_stddev, params().pose_between_factor_rotation_stddev,
     params().pose_between_factor_rotation_stddev, params().pose_between_factor_translation_stddev,
     params().pose_between_factor_translation_stddev, params().pose_between_factor_translation_stddev)
      .finished());
  zero_relative_pose_noise =
    go::Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_between_noise_sigmas)),
               params().huber_k);
}

template <typename PoseVelocityNodeAdderType>
int StandstillFactorAdder::AddFactors(const localization_measurements::StandstillMeasurement& standstill_measurement,
                                      gtsam::NonlinearFactorGraph& factors) {
  int num_factors_added = 0;
  if (params().add_velocity_prior) {
    if (AddZeroVelocityPrior(standstill_measurement.timestamp(), factors)) ++num_factors_added;
  }
  if (params().add_pose_between_factor) {
    if (AddZeroRelativePoseFactor(standstill_measurement.previous_timestamp, standstill_measurement.timestamp(),
                                  factors))
      ++num_factors_added;
  }
}

template <typename PoseVelocityNodeAdderType>
bool StandstillFactorAdder::AddZeroVelocityPrior(const localization_common::Time timestamp,
                                                 gtsam::NonlinearFactorGraph& factors) {
  if (!node_adder_->AddNode(timestamp, factors)) {
    return false;
  }
  const auto keys = node_adder_->Keys(timestamp);
  if (keys.empty()) {
    LogError("AddZeroVelocityPrior: Failed to get keys for timestamp.");
    return false;
  }
  // Assumes second key is velocity
  const auto& velocity_key = keys[1];

  // Create factor
  gtsam::PriorFactor<gtsam::Velocity3>::shared_ptr velocity_prior_factor(
    new gtsam::PriorFactor<gtsam::Velocity3>(velocity_key, gtsam::Velocity3::Zero(), zero_velocity_noise_));
  factors.push_back(velocity_prior_factor);
  LogDebug("AddFactors: Added standstill velocity prior factor.");
  return true;
}

template <typename PoseVelocityNodeAdderType>
bool StandstillFactorAdder::AddZeroRelativePoseFactor(const localization_common::Time timestamp_a,
                                                      const localization_common::Time timestamp_b,
                                                      gtsam::NonlinearFactorGraph& factors) {
  if (!(node_adder_->AddNode(timestamp_a, factors) && node_adder_->AddNode(timestamp_b, factors))) {
    return false;
  }

  const auto keys_a = node_adder_->Keys(timestamp_a);
  if (keys_a.empty()) {
    LogError("AddZeroVelocityPrior: Failed to get keys for timestamp_a.");
    return false;
  }
  const auto keys_b = node_adder_->Keys(timestamp_b);
  if (keys_b.empty()) {
    LogError("AddZeroVelocityPrior: Failed to get keys for timestamp_b.");
    return false;
  }
  // Assumes first key is pose
  const auto& pose_key_a = keys_a[0];
  const auto& pose_key_b = keys_b[0];

  gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr pose_between_factor(new gtsam::BetweenFactor<gtsam::Pose3>(
    pose_key_a, pose_key_b, gtsam::Pose3::identity(), zero_relative_pose_noise_));
  factors.push_back(pose_between_factor);
  LogDebug("AddFactors: Added standstill pose between factor.");
}
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_STANDSTILL_FACTOR_ADDER_H_
