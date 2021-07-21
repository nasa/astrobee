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
#include <localization_common/logger.h>

#include <gtsam/base/Vector.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
LocFactorAdder::LocFactorAdder(const LocFactorAdderParams& params,
                               const go::GraphActionCompleterType graph_action_completer_type)
    : LocFactorAdder::Base(params), graph_action_completer_type_(graph_action_completer_type) {}

std::vector<go::FactorsToAdd> LocFactorAdder::AddFactors(
  const lm::MatchedProjectionsMeasurement& matched_projections_measurement) {
  if (matched_projections_measurement.matched_projections.empty()) {
    LogDebug("AddFactors: Empty measurement.");
    return {};
  }

  if (static_cast<int>(matched_projections_measurement.matched_projections.size()) < params().min_num_matches) {
    LogDebug("AddFactors: Not enough matches in projection measurement.");
    return {};
  }

  const int num_landmarks = matched_projections_measurement.matched_projections.size();
  num_landmarks_averager_.Update(num_landmarks);
  std::vector<go::FactorsToAdd> factors_to_add;
  if (params().add_pose_priors) {
    double noise_scale = params().pose_noise_scale;
    if (params().scale_pose_noise_with_num_landmarks) {
      noise_scale *= std::pow((num_landmarks_averager_.average() / static_cast<double>(num_landmarks)), 2);
    }

    go::FactorsToAdd pose_factors_to_add;
    pose_factors_to_add.reserve(1);
    const gtsam::Vector6 pose_prior_noise_sigmas((gtsam::Vector(6) << params().prior_translation_stddev,
                                                  params().prior_translation_stddev, params().prior_translation_stddev,
                                                  params().prior_quaternion_stddev, params().prior_quaternion_stddev,
                                                  params().prior_quaternion_stddev)
                                                   .finished());
    const auto pose_noise = Robust(
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(noise_scale * pose_prior_noise_sigmas)),
      params().huber_k);

    const go::KeyInfo key_info(&sym::P, go::NodeUpdaterType::CombinedNavState,
                               matched_projections_measurement.timestamp);
    gtsam::LocPoseFactor::shared_ptr pose_prior_factor(new gtsam::LocPoseFactor(
      key_info.UninitializedKey(), matched_projections_measurement.global_T_cam * params().body_T_cam.inverse(),
      pose_noise));
    pose_factors_to_add.push_back({{key_info}, pose_prior_factor});
    pose_factors_to_add.SetTimestamp(matched_projections_measurement.timestamp);
    LogDebug("AddFactors: Added 1 loc pose prior factor.");
    factors_to_add.emplace_back(pose_factors_to_add);
  }
  if (params().add_projections) {
    double noise_scale = params().projection_noise_scale;
    if (params().scale_projection_noise_with_num_landmarks) {
      noise_scale *= std::pow((num_landmarks_averager_.average() / static_cast<double>(num_landmarks)), 2);
    }

    int num_loc_projection_factors = 0;
    go::FactorsToAdd projection_factors_to_add(type());
    projection_factors_to_add.reserve(matched_projections_measurement.matched_projections.size());
    for (const auto& matched_projection : matched_projections_measurement.matched_projections) {
      const go::KeyInfo key_info(&sym::P, go::NodeUpdaterType::CombinedNavState,
                                 matched_projections_measurement.timestamp);
      // TODO(rsoussan): Pass sigma insted of already constructed isotropic noise
      const gtsam::SharedIsotropic scaled_noise(
        gtsam::noiseModel::Isotropic::Sigma(2, noise_scale * params().cam_noise->sigma()));
      gtsam::LocProjectionFactor<>::shared_ptr loc_projection_factor(new gtsam::LocProjectionFactor<>(
        matched_projection.image_point, matched_projection.map_point, Robust(scaled_noise, params().huber_k),
        key_info.UninitializedKey(), params().cam_intrinsics, params().body_T_cam));
      // Set world_T_cam estimate in case need to use it as a fallback
      loc_projection_factor->setWorldTCam(matched_projections_measurement.global_T_cam);
      projection_factors_to_add.push_back({{key_info}, loc_projection_factor});
      ++num_loc_projection_factors;
      if (num_loc_projection_factors >= params().max_num_factors) break;
    }
    projection_factors_to_add.SetTimestamp(matched_projections_measurement.timestamp);
    LogInfo("AddFactors: Added " << num_loc_projection_factors << " loc projection factors.");
    factors_to_add.emplace_back(projection_factors_to_add);
  }
  return factors_to_add;
}

go::GraphActionCompleterType LocFactorAdder::type() const { return graph_action_completer_type_; }
}  // namespace graph_localizer
