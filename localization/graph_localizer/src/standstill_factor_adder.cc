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

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <glog/logging.h>

namespace graph_localizer {
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
StandstillFactorAdder::StandstillFactorAdder(const StandstillFactorAdderParams& params)
    : StandstillFactorAdder::Base(params) {}

std::vector<FactorsToAdd> StandstillFactorAdder::AddFactors(
  const lm::FeaturePointsMeasurement& feature_points_measurement) {
  FactorsToAdd standstill_prior_factors_to_add;
  const gtsam::Vector3 velocity_prior_noise_sigmas(
    (gtsam::Vector(3) << params().prior_velocity_stddev, params().prior_velocity_stddev, params().prior_velocity_stddev)
      .finished());
  const auto velocity_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)),
           params().huber_k);

  const KeyInfo velocity_key_info(&sym::V, feature_points_measurement.timestamp);
  gtsam::PriorFactor<gtsam::Velocity3>::shared_ptr velocity_prior_factor(new gtsam::PriorFactor<gtsam::Velocity3>(
    velocity_key_info.UninitializedKey(), gtsam::Velocity3::Zero(), velocity_noise));
  standstill_prior_factors_to_add.push_back({{velocity_key_info}, velocity_prior_factor});
  standstill_prior_factors_to_add.SetTimestamp(feature_points_measurement.timestamp);
  VLOG(2) << "AddFactors: Added " << standstill_prior_factors_to_add.size() << " standstill factors.";
  return {standstill_prior_factors_to_add};
}
}  // namespace graph_localizer
