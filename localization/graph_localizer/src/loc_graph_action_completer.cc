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

#include <graph_localizer/loc_graph_action_completer.h>
#include <graph_localizer/loc_pose_factor.h>
#include <graph_localizer/loc_projection_factor.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/base/Vector.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
LocGraphActionCompleter::LocGraphActionCompleter(const LocFactorAdderParams& params,
                                                 const go::GraphActionCompleterType graph_action_completer_type,
                                                 std::shared_ptr<CombinedNavStateGraphValues> graph_values)
    : params_(params),
      graph_action_completer_type_(graph_action_completer_type),
      graph_values_(std::move(graph_values)) {}

go::GraphActionCompleterType LocGraphActionCompleter::type() const { return graph_action_completer_type_; }

bool LocGraphActionCompleter::DoAction(go::FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors) {
  auto& factors = factors_to_add.Get();
  LogInfo("Beginning: " << factors.size());
  boost::optional<gtsam::Key> pose_key;
  boost::optional<gtsam::Pose3> world_T_cam;
  LogInfo("Max inlier norm: " << params_.max_inlier_weighted_projection_norm);
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

    const auto world_T_body = graph_values_->at<gtsam::Pose3>(projection_factor->key());
    if (!world_T_body) {
      LogError("MapProjectionNoiseScaling: Failed to get pose.");
      return false;
    }
    const auto error = (projection_factor->evaluateError(*world_T_body)).norm();
    const auto cheirality_error = projection_factor->cheiralityError(*world_T_body);
    if (cheirality_error) {
      factor_it = factors.erase(factor_it);
      LogInfo("Cheirality error");
    } else if (error > params_.max_inlier_weighted_projection_norm && params_.max_inlier_weighted_projection_norm > 0) {
      factor_it = factors.erase(factor_it);
      LogInfo("Too much error");
    } else {
      if (params_.weight_projections_with_distance) {
        const gtsam::Point3 nav_cam_t_landmark =
          (*world_T_body * *(projection_factor->body_P_sensor())).inverse() * projection_factor->landmark_point();
        const gtsam::SharedIsotropic scaled_noise(
          gtsam::noiseModel::Isotropic::Sigma(2, params_.projection_noise_scale * 1.0 / nav_cam_t_landmark.z()));
        // Don't use robust cost here to more effectively correct a drift occurance
        gtsam::LocProjectionFactor<>::shared_ptr loc_projection_factor(new gtsam::LocProjectionFactor<>(
          projection_factor->measured(), projection_factor->landmark_point(), scaled_noise, projection_factor->key(),
          projection_factor->calibration(), *(projection_factor->body_P_sensor())));
        factor_it->factor = loc_projection_factor;
      }
      ++factor_it;
    }
  }

  LogInfo("End: " << factors.size());

  // All factors have been removed due to errors, use loc pose prior instead
  if (factors.empty() && params_.add_prior_if_projections_fail) {
    if (!pose_key || !world_T_cam) {
      LogError("MapProjectionNoiseScaling: Failed to get pose key and world_T_cam");
      return false;
    }
    const gtsam::Vector6 pose_prior_noise_sigmas((gtsam::Vector(6) << params_.prior_translation_stddev,
                                                  params_.prior_translation_stddev, params_.prior_translation_stddev,
                                                  params_.prior_quaternion_stddev, params_.prior_quaternion_stddev,
                                                  params_.prior_quaternion_stddev)
                                                   .finished());
    // TODO(rsoussan): enable scaling with num landmarks
    const int noise_scale = 1;
    const auto pose_noise = Robust(
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(noise_scale * pose_prior_noise_sigmas)),
      params_.huber_k);
    gtsam::LocPoseFactor::shared_ptr pose_prior_factor(
      new gtsam::LocPoseFactor(*pose_key, *world_T_cam * params_.body_T_cam.inverse(), pose_noise));
    factors_to_add.push_back(go::FactorToAdd(
      {go::KeyInfo(&sym::P, go::NodeUpdaterType::CombinedNavState, factors_to_add.timestamp())}, pose_prior_factor));
    LogError("Added prior factor");
  }
  return true;
}

}  // namespace graph_localizer
