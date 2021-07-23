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

#include <graph_localizer/semantic_loc_graph_action_completer.h>
#include <graph_localizer/tolerant_projection_factor.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/base/Vector.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
SemanticLocGraphActionCompleter::SemanticLocGraphActionCompleter(const LocFactorAdderParams& params,
                                                                 const go::GraphActionCompleterType graph_action_completer_type,
                                                                 std::shared_ptr<CombinedNavStateGraphValues> graph_values)
    : params_(params),
      graph_action_completer_type_(graph_action_completer_type),
      graph_values_(std::move(graph_values)) {}

go::GraphActionCompleterType SemanticLocGraphActionCompleter::type() const { return graph_action_completer_type_; }

bool SemanticLocGraphActionCompleter::DoAction(go::FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors) {
  auto& factors = factors_to_add.Get();
  boost::optional<gtsam::Key> pose_key;
  for (auto factor_it = factors.begin(); factor_it != factors.end();) {
    auto& factor = factor_it->factor;
    auto projection_factor = dynamic_cast<gtsam::TolerantProjectionFactor<>*>(factor.get());
    if (!projection_factor) {
      LogError("MapProjectionNoiseScaling: Failed to cast to tolerant projection factor.");
      return false;
    }

    // Save pose and pose key in case need to make loc prior
    pose_key = projection_factor->key();

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
        gtsam::TolerantProjectionFactor<>::shared_ptr tolerant_projection_factor(new gtsam::TolerantProjectionFactor<>(
          projection_factor->measured(), projection_factor->landmark_point(), scaled_noise, projection_factor->key(),
          projection_factor->calibration(), *(projection_factor->body_P_sensor())));
        factor_it->factor = tolerant_projection_factor;
      }
      ++factor_it;
    }
  }

  return true;
}

}  // namespace graph_localizer
