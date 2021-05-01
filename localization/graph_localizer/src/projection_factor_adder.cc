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

#include <graph_localizer/projection_factor_adder.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/base/Vector.h>
#include <gtsam/slam/ProjectionFactor.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
ProjectionFactorAdder::ProjectionFactorAdder(const ProjectionFactorAdderParams& params,
                                             std::shared_ptr<const FeatureTracker> feature_tracker,
                                             std::shared_ptr<const go::GraphValues> graph_values)
    : ProjectionFactorAdder::Base(params), feature_tracker_(feature_tracker), graph_values_(graph_values) {
  projection_triangulation_params_.rankTolerance = 1e-9;
  // TODO(rsoussan): why is Base necessary for these?
  projection_triangulation_params_.enableEPI = Base::params().enable_EPI;
  projection_triangulation_params_.landmarkDistanceThreshold = Base::params().landmark_distance_threshold;
  projection_triangulation_params_.dynamicOutlierRejectionThreshold =
    Base::params().dynamic_outlier_rejection_threshold;
}

std::vector<go::FactorsToAdd> ProjectionFactorAdder::AddFactors(
  const lm::FeaturePointsMeasurement& feature_points_measurement) {
  std::vector<go::FactorsToAdd> factors_to_add_vec;
  // Add projection factors for new measurements of already existing features
  go::FactorsToAdd projection_factors_to_add;
  for (const auto& feature_point : feature_points_measurement.feature_points) {
    if (graph_values_->HasFeature(feature_point.feature_id)) {
      const go::KeyInfo pose_key_info(&sym::P, go::NodeUpdaterType::CombinedNavState, feature_point.timestamp);
      const go::KeyInfo static_point_key_info(&sym::F, go::NodeUpdaterType::FeaturePoint, feature_point.feature_id);
      const auto point_key = graph_values_->FeatureKey(feature_point.feature_id);
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
        !graph_values_->HasFeature(feature_track.id()) &&
        (new_features + graph_values_->NumFeatures()) < params().max_num_features) {
      // Create new factors to add for each feature track so the graph action can act on only that
      // feature track to triangulate a new point
      go::FactorsToAdd projection_factors_with_new_point_to_add(type());
      const auto point_key = graph_values_->CreateFeatureKey();
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

go::GraphActionCompleterType ProjectionFactorAdder::type() const {
  return go::GraphActionCompleterType::ProjectionFactor;
}

bool ProjectionFactorAdder::DoAction(go::FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors,
                                     go::GraphValues& graph_values) {
  return TriangulateNewPoint(factors_to_add, graph_factors, graph_values);
}

bool ProjectionFactorAdder::TriangulateNewPoint(go::FactorsToAdd& factors_to_add,
                                                gtsam::NonlinearFactorGraph& graph_factors,
                                                go::GraphValues& graph_values) {
  gtsam::CameraSet<Camera> camera_set;
  Camera::MeasurementVector measurements;
  gtsam::Key point_key;
  for (const auto& factor_to_add : factors_to_add.Get()) {
    const auto& factor = factor_to_add.factor;
    const auto projection_factor = dynamic_cast<ProjectionFactor*>(factor.get());
    if (!projection_factor) {
      LogError("TriangulateNewPoint: Failed to cast to projection factor.");
      return false;
    }
    const auto world_T_body = graph_values.at<gtsam::Pose3>(projection_factor->key1());
    if (!world_T_body) {
      LogError("TriangulateNewPoint: Failed to get pose.");
      return false;
    }

    const gtsam::Pose3 world_T_camera = *world_T_body * params().body_T_cam;
    camera_set.emplace_back(world_T_camera, params().cam_intrinsics);
    measurements.emplace_back(projection_factor->measured());
    point_key = projection_factor->key2();
  }
  gtsam::TriangulationResult world_t_triangulated_point;
  // TODO(rsoussan): Gtsam shouldn't be throwing exceptions for this, but needed if enable_epi enabled.
  // Is there a build setting that prevents cheirality from being thrown in this case?
  try {
    world_t_triangulated_point = gtsam::triangulateSafe(camera_set, measurements, projection_triangulation_params_);
  } catch (...) {
    LogDebug("TriangulateNewPoint: Exception occurred during triangulation");
    return false;
  }

  if (!world_t_triangulated_point.valid()) {
    LogDebug("TriangulateNewPoint: Failed to triangulate point");
    return false;
  }
  // TODO(rsoussan): clean this up
  const auto feature_id = factors_to_add.Get().front().key_infos[1].id();
  graph_values.AddFeature(feature_id, *world_t_triangulated_point, point_key);
  if (params().add_point_priors) {
    const gtsam::Vector3 point_prior_noise_sigmas((gtsam::Vector(3) << params().point_prior_translation_stddev,
                                                   params().point_prior_translation_stddev,
                                                   params().point_prior_translation_stddev)
                                                    .finished());
    const auto point_noise =
      Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(point_prior_noise_sigmas)),
             params().huber_k);
    gtsam::PriorFactor<gtsam::Point3> point_prior_factor(point_key, *world_t_triangulated_point, point_noise);
    graph_factors.push_back(point_prior_factor);
  }
  return true;
}
}  // namespace graph_localizer
