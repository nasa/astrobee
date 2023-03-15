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

#include <factor_adders/standstill_factor_adder.h>

namespace factor_adders {
// AddFactors specialization
template <typename MeasurementType, bool SingleMeasurementPerFactor>
int MeasurementBasedFactorAdder::AddFactors(
  const localization_measurements::StandstillMeasurement& standstill_measurement,
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

StandstillFactorAdder::StandstillFactorAdder(const StandstillFactorAdderParams& params,
                                             const std::shared_ptr<NodeAdder> node_adder)
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

bool StandstillFactorAdder::AddZeroVelocityPrior(const localization_common::Time timestamp,
                                                 gtsam::NonlinearFactorGraph& factors) {
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

bool StandstillFactorAdder::AddZeroRelativePoseFactor(const localization_common::Time timestamp_a,
                                                      const localization_common::Time timestamp_b,
                                                      gtsam::NonlinearFactorGraph& factors) {
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
