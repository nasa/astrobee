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

#include <graph_vio/projection_factor_adder.h>
#include <graph_vio/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/base/Vector.h>
#include <gtsam/slam/ProjectionFactor.h>

namespace graph_vio {
namespace go = graph_optimizer;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
ProjectionFactorAdder::ProjectionFactorAdder(const ProjectionFactorAdderParams& params,
                                             std::shared_ptr<const FeatureTracker> feature_tracker,
                                             std::shared_ptr<const FeaturePointGraphValues> feature_point_graph_values)
    : ProjectionFactorAdder::Base(params),
      feature_tracker_(feature_tracker),
      feature_point_graph_values_(std::move(feature_point_graph_values)) {}

std::vector<go::FactorsToAdd> ProjectionFactorAdder::AddFactors(
  const lm::FeaturePointsMeasurement& feature_points_measurement) {
  std::vector<go::FactorsToAdd> factors_to_add_vec;
  // Add projection factors for new measurements of already existing features
  go::FactorsToAdd projection_factors_to_add;
  for (const auto& feature_point : feature_points_measurement.feature_points) {
    if (feature_point_graph_values_->HasFeature(feature_point.feature_id)) {
      const go::KeyInfo pose_key_info(&sym::P, go::NodeUpdaterType::CombinedNavState, feature_point.timestamp);
      const go::KeyInfo static_point_key_info(&sym::F, go::NodeUpdaterType::FeaturePoint, feature_point.feature_id);
      const auto point_key = feature_point_graph_values_->FeatureKey(feature_point.feature_id);
      if (!point_key) {
        LogError("AddFactors: Failed to get point key.");
        continue;
      }
      const auto projection_factor = boost::make_shared<ProjectionFactor>(
        feature_point.image_point, Robust(params().cam_noise, params().huber_k), pose_key_info.UninitializedKey(),
        *point_key, params().cam_intrinsics, params().body_T_cam);
      projection_factors_to_add.push_back({{pose_key_info, static_point_key_info}, projection_factor});
    }
  }
  if (!projection_factors_to_add.empty()) {
    projection_factors_to_add.SetTimestamp(feature_points_measurement.timestamp);
    factors_to_add_vec.emplace_back(projection_factors_to_add);
    LogDebug("AddFactors: Added " << projection_factors_to_add.size() << " projection factors for existing features.");
  }

  // Add new feature tracks and measurements if possible
  int new_features = 0;
  for (const auto& feature_track_pair : feature_tracker_->feature_tracks()) {
    const auto& feature_track = *(feature_track_pair.second);
    if (static_cast<int>(feature_track.size()) >= params().min_num_measurements_for_triangulation &&
        !feature_point_graph_values_->HasFeature(feature_track.id()) &&
        (new_features + feature_point_graph_values_->NumFeatures()) < params().max_num_features) {
      // Create new factors to add for each feature track so the graph action can act on only that
      // feature track to triangulate a new point
      go::FactorsToAdd projection_factors_with_new_point_to_add(go::GraphActionCompleterType::ProjectionFactor);
      const auto point_key = feature_point_graph_values_->CreateFeatureKey();
      for (const auto& feature_point_pair : feature_track.points()) {
        const auto& feature_point = feature_point_pair.second;
        const go::KeyInfo pose_key_info(&sym::P, go::NodeUpdaterType::CombinedNavState, feature_point.timestamp);
        const go::KeyInfo static_point_key_info(&sym::F, go::NodeUpdaterType::FeaturePoint, feature_point.feature_id);
        const auto projection_factor = boost::make_shared<ProjectionFactor>(
          feature_point.image_point, Robust(params().cam_noise, params().huber_k), pose_key_info.UninitializedKey(),
          point_key, params().cam_intrinsics, params().body_T_cam);
        projection_factors_with_new_point_to_add.push_back({{pose_key_info, static_point_key_info}, projection_factor});
      }
      projection_factors_with_new_point_to_add.SetTimestamp(feature_points_measurement.timestamp);
      factors_to_add_vec.emplace_back(projection_factors_with_new_point_to_add);
      ++new_features;
    }
  }
  if (new_features > 0) LogDebug("AddFactors: Added " << new_features << " new features.");
  if (factors_to_add_vec.empty()) return {};
  return factors_to_add_vec;
}
}  // namespace graph_vio
