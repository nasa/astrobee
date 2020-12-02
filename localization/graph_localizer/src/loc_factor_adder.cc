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

#include <graph_localizer/loc_factor_adder.h>
#include <graph_localizer/loc_pose_factor.h>
#include <graph_localizer/loc_projection_factor.h>
#include <graph_localizer/utilities.h>

#include <gtsam/base/Vector.h>

#include <glog/logging.h>

namespace graph_localizer {
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
LocFactorAdder::LocFactorAdder(const LocFactorAdderParams& params, const GraphAction graph_action)
    : LocFactorAdder::Base(params), graph_action_(graph_action) {}

std::vector<FactorsToAdd> LocFactorAdder::AddFactors(
  const lm::MatchedProjectionsMeasurement& matched_projections_measurement) const {
  if (matched_projections_measurement.matched_projections.empty()) {
    LOG(WARNING) << "LocFactorAdder::AddFactors: Empty measurement.";
    return {};
  }

  // TODO(rsoussan): Unify his with ValidVLMsg call
  if (matched_projections_measurement.matched_projections.size() < params().min_num_matches) {
    LOG(WARNING) << "LocFactorAdder::AddFactors: Not enough matches in projection measurement.";
    return {};
  }

  double noise_scale = 1;
  if (params().scale_noise_with_num_landmarks) {
    noise_scale =
      params().noise_scale / static_cast<double>(matched_projections_measurement.matched_projections.size());
  }

  if (params().add_pose_priors) {
    int num_loc_pose_prior_factors = 0;
    FactorsToAdd factors_to_add(graph_action_);
    factors_to_add.reserve(1);
    const gtsam::Vector6 pose_prior_noise_sigmas((gtsam::Vector(6) << params().prior_translation_stddev,
                                                  params().prior_translation_stddev, params().prior_translation_stddev,
                                                  params().prior_quaternion_stddev, params().prior_quaternion_stddev,
                                                  params().prior_quaternion_stddev)
                                                   .finished());
    const auto pose_noise = Robust(
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(noise_scale * pose_prior_noise_sigmas)));

    const KeyInfo key_info(&sym::P, matched_projections_measurement.timestamp);
    gtsam::LocPoseFactor::shared_ptr pose_prior_factor(new gtsam::LocPoseFactor(
      key_info.UninitializedKey(), matched_projections_measurement.global_T_cam * params().body_T_cam.inverse(),
      pose_noise));
    factors_to_add.push_back({{key_info}, pose_prior_factor});
    factors_to_add.SetTimestamp(matched_projections_measurement.timestamp);
    VLOG(2) << "LocFactorAdder::AddFactors: Added " << num_loc_pose_prior_factors << " loc pose priors factors.";
    return {factors_to_add};
  } else if (params().add_projections) {
    int num_loc_projection_factors = 0;
    FactorsToAdd factors_to_add(graph_action_);
    factors_to_add.reserve(matched_projections_measurement.matched_projections.size());
    for (const auto& matched_projection : matched_projections_measurement.matched_projections) {
      const KeyInfo key_info(&sym::P, matched_projections_measurement.timestamp);
      // TODO(rsoussan): Pass sigma insted of already constructed isotropic noise
      const gtsam::SharedIsotropic scaled_noise(
        gtsam::noiseModel::Isotropic::Sigma(2, noise_scale * params().cam_noise->sigma()));
      gtsam::LocProjectionFactor<>::shared_ptr loc_projection_factor(new gtsam::LocProjectionFactor<>(
        matched_projection.image_point, matched_projection.map_point, Robust(scaled_noise), key_info.UninitializedKey(),
        params().cam_intrinsics, params().body_T_cam));
      factors_to_add.push_back({{key_info}, loc_projection_factor});
      ++num_loc_projection_factors;
    }
    factors_to_add.SetTimestamp(matched_projections_measurement.timestamp);
    VLOG(2) << "LocFactorAdder::AddFactors: Added " << num_loc_projection_factors << " loc projection factors.";
    return {factors_to_add};
  } else {
    LOG(ERROR) << "LocFactorAdder::AddFactors: LocFactorAdder enabled but neither pose nor projection factors are.";
    return {};
  }
}
}  // namespace graph_localizer
