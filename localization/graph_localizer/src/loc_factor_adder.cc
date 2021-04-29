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

#include <graph_localizer/graph_action.h>
#include <graph_localizer/loc_factor_adder.h>
#include <graph_localizer/loc_pose_factor.h>
#include <graph_localizer/loc_projection_factor.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/base/Vector.h>

namespace graph_localizer {
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
LocFactorAdder::LocFactorAdder(const LocFactorAdderParams& params, const GraphAction projection_graph_action)
    : LocFactorAdder::Base(params), projection_graph_action_(projection_graph_action) {}

std::vector<FactorsToAdd> LocFactorAdder::AddFactors(
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
  std::vector<FactorsToAdd> factors_to_add;
  if (params().add_pose_priors) {
    double noise_scale = params().pose_noise_scale;
    if (params().scale_pose_noise_with_num_landmarks) {
      noise_scale *= std::pow((num_landmarks_averager_.average() / static_cast<double>(num_landmarks)), 2);
    }

    FactorsToAdd pose_factors_to_add;
    pose_factors_to_add.reserve(1);
    const gtsam::Vector6 pose_prior_noise_sigmas((gtsam::Vector(6) << params().prior_translation_stddev,
                                                  params().prior_translation_stddev, params().prior_translation_stddev,
                                                  params().prior_quaternion_stddev, params().prior_quaternion_stddev,
                                                  params().prior_quaternion_stddev)
                                                   .finished());
    const auto pose_noise = Robust(
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(noise_scale * pose_prior_noise_sigmas)),
      params().huber_k);

    const KeyInfo key_info(&sym::P, NodeUpdaterType::CombinedNavState, matched_projections_measurement.timestamp);
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
    FactorsToAdd projection_factors_to_add(projection_graph_action_);
    projection_factors_to_add.reserve(matched_projections_measurement.matched_projections.size());
    for (const auto& matched_projection : matched_projections_measurement.matched_projections) {
      const KeyInfo key_info(&sym::P, NodeUpdaterType::CombinedNavState, matched_projections_measurement.timestamp);
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
    LogDebug("AddFactors: Added " << num_loc_projection_factors << " loc projection factors.");
    factors_to_add.emplace_back(projection_factors_to_add);
  }
  return factors_to_add;
}

GraphActionCompleterType LocFactorAdder::type() const { return GraphActionCompleterType::LocProjectionFactor; }

bool LocFactorAdder::DoAction(FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors,
                              GraphValues& graph_values) {
  auto& factors = factors_to_add.Get();
  boost::optional<gtsam::Key> pose_key;
  boost::optional<gtsam::Pose3> world_T_cam;
  for (auto factor_it = factors.begin(); factor_it != factors.end();) {
    auto& factor = factor_it->factor;
    auto projection_factor = dynamic_cast<gtsam::LocProjectionFactor<>*>(factor.get());
    if (!projection_factor) {
      LogError("MapProjectionNoiseScaling: Failed to cast to projection factor.");
      return false;
    }

    // Save pose and pose key in case need to make loc prior
    pose_key = projection_factor->key();
    world_T_cam = projection_factor->world_T_cam();

    const auto world_T_body = graph_values.at<gtsam::Pose3>(projection_factor->key());
    if (!world_T_body) {
      LogError("MapProjectionNoiseScaling: Failed to get pose.");
      return false;
    }
    const auto error = (projection_factor->evaluateError(*world_T_body)).norm();
    const auto cheirality_error = projection_factor->cheiralityError(*world_T_body);
    if (cheirality_error) {
      factor_it = factors.erase(factor_it);
    } else if (error > params().max_inlier_weighted_projection_norm) {
      factor_it = factors.erase(factor_it);
    } else {
      if (params().weight_projections_with_distance) {
        const gtsam::Point3 nav_cam_t_landmark =
          (*world_T_body * *(projection_factor->body_P_sensor())).inverse() * projection_factor->landmark_point();
        const gtsam::SharedIsotropic scaled_noise(
          gtsam::noiseModel::Isotropic::Sigma(2, params().projection_noise_scale * 1.0 / nav_cam_t_landmark.z()));
        // Don't use robust cost here to more effectively correct a drift occurance
        gtsam::LocProjectionFactor<>::shared_ptr loc_projection_factor(new gtsam::LocProjectionFactor<>(
          projection_factor->measured(), projection_factor->landmark_point(), scaled_noise, projection_factor->key(),
          projection_factor->calibration(), *(projection_factor->body_P_sensor())));
        factor_it->factor = loc_projection_factor;
      }
      ++factor_it;
    }
  }
  // All factors have been removed due to errors, use loc pose prior instead
  if (factors.empty() && params().add_prior_if_projections_fail) {
    if (!pose_key || !world_T_cam) {
      LogError("MapProjectionNoiseScaling: Failed to get pose key and world_T_cam");
      return false;
    }
    const gtsam::Vector6 pose_prior_noise_sigmas((gtsam::Vector(6) << params().prior_translation_stddev,
                                                  params().prior_translation_stddev, params().prior_translation_stddev,
                                                  params().prior_quaternion_stddev, params().prior_quaternion_stddev,
                                                  params().prior_quaternion_stddev)
                                                   .finished());
    // TODO(rsoussan): enable scaling with num landmarks
    const int noise_scale = 1;
    const auto pose_noise = Robust(
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(noise_scale * pose_prior_noise_sigmas)),
      params().huber_k);
    gtsam::LocPoseFactor::shared_ptr pose_prior_factor(
      new gtsam::LocPoseFactor(*pose_key, *world_T_cam * params().body_T_cam.inverse(), pose_noise));
    factors_to_add.push_back(FactorToAdd(
      {KeyInfo(&sym::P, NodeUpdaterType::CombinedNavState, factors_to_add.timestamp())}, pose_prior_factor));
  }
  return true;
}

}  // namespace graph_localizer
