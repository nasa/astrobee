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

#include <graph_localizer/standstill_factor_adder.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/PriorFactor.h>

namespace graph_localizer {
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
StandstillFactorAdder::StandstillFactorAdder(const StandstillFactorAdderParams& params,
                                             std::shared_ptr<const FeatureTracker> feature_tracker)
    : StandstillFactorAdder::Base(params), feature_tracker_(feature_tracker) {}

std::vector<FactorsToAdd> StandstillFactorAdder::AddFactors(
  const lm::FeaturePointsMeasurement& feature_points_measurement) {
  std::vector<FactorsToAdd> factors_to_add;
  if (params().add_velocity_prior) {
    FactorsToAdd standstill_prior_factors_to_add;
    const gtsam::Vector3 velocity_prior_noise_sigmas((gtsam::Vector(3) << params().prior_velocity_stddev,
                                                      params().prior_velocity_stddev, params().prior_velocity_stddev)
                                                       .finished());
    const auto velocity_noise =
      Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)),
             params().huber_k);

    const KeyInfo velocity_key_info(&sym::V, NodeUpdater::CombinedNavState, feature_points_measurement.timestamp);
    gtsam::PriorFactor<gtsam::Velocity3>::shared_ptr velocity_prior_factor(new gtsam::PriorFactor<gtsam::Velocity3>(
      velocity_key_info.UninitializedKey(), gtsam::Velocity3::Zero(), velocity_noise));
    standstill_prior_factors_to_add.push_back({{velocity_key_info}, velocity_prior_factor});
    standstill_prior_factors_to_add.SetTimestamp(feature_points_measurement.timestamp);
    LogDebug("AddFactors: Added " << standstill_prior_factors_to_add.size() << " standstill velocity prior factors.");
    factors_to_add.emplace_back(standstill_prior_factors_to_add);
  }
  if (params().add_pose_between_factor) {
    const auto previous_timestamp = feature_tracker_->PreviousTimestamp();
    if (previous_timestamp) {
      FactorsToAdd pose_between_factors_to_add;
      const gtsam::Vector6 pose_between_noise_sigmas(
        (gtsam::Vector(6) << params().pose_between_factor_rotation_stddev, params().pose_between_factor_rotation_stddev,
         params().pose_between_factor_rotation_stddev, params().pose_between_factor_translation_stddev,
         params().pose_between_factor_translation_stddev, params().pose_between_factor_translation_stddev)
          .finished());
      const auto pose_between_noise =
        Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_between_noise_sigmas)),
               params().huber_k);
      const KeyInfo previous_between_key_info(&sym::P, NodeUpdater::CombinedNavState, *previous_timestamp);
      const KeyInfo current_between_key_info(&sym::P, NodeUpdater::CombinedNavState,
                                             feature_points_measurement.timestamp);
      gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr pose_between_factor(new gtsam::BetweenFactor<gtsam::Pose3>(
        previous_between_key_info.UninitializedKey(), current_between_key_info.UninitializedKey(),
        gtsam::Pose3::identity(), pose_between_noise));
      pose_between_factors_to_add.push_back(
        {{previous_between_key_info, current_between_key_info}, pose_between_factor});
      pose_between_factors_to_add.SetTimestamp(feature_points_measurement.timestamp);
      LogDebug("AddFactors: Added " << pose_between_factors_to_add.size() << " standstill pose between factors.");
      factors_to_add.emplace_back(pose_between_factors_to_add);
    }
  }
  return factors_to_add;
}
}  // namespace graph_localizer
