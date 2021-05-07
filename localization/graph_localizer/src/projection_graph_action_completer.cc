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

#include <graph_localizer/projection_graph_action_completer.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/base/Vector.h>
#include <gtsam/slam/ProjectionFactor.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace sym = gtsam::symbol_shorthand;
ProjectionGraphActionCompleter::ProjectionGraphActionCompleter(
  const ProjectionFactorAdderParams& params, std::shared_ptr<const CombinedNavStateGraphValues> graph_values,
  std::shared_ptr<go::FeaturePointGraphValues> feature_point_graph_values)
    : params_(params), graph_values_(graph_values), feature_point_graph_values_(std::move(feature_point_graph_values)) {
  projection_triangulation_params_.rankTolerance = 1e-9;
  // TODO(rsoussan): why is Base necessary for these?
  projection_triangulation_params_.enableEPI = params_.enable_EPI;
  projection_triangulation_params_.landmarkDistanceThreshold = params_.landmark_distance_threshold;
  projection_triangulation_params_.dynamicOutlierRejectionThreshold = params_.dynamic_outlier_rejection_threshold;
}

go::GraphActionCompleterType ProjectionGraphActionCompleter::type() const {
  return go::GraphActionCompleterType::ProjectionFactor;
}

bool ProjectionGraphActionCompleter::DoAction(go::FactorsToAdd& factors_to_add,
                                              gtsam::NonlinearFactorGraph& graph_factors) {
  return TriangulateNewPoint(factors_to_add, graph_factors);
}

bool ProjectionGraphActionCompleter::TriangulateNewPoint(go::FactorsToAdd& factors_to_add,
                                                         gtsam::NonlinearFactorGraph& graph_factors) {
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
    const auto world_T_body = graph_values_->at<gtsam::Pose3>(projection_factor->key1());
    if (!world_T_body) {
      LogError("TriangulateNewPoint: Failed to get pose.");
      return false;
    }

    const gtsam::Pose3 world_T_camera = *world_T_body * params_.body_T_cam;
    camera_set.emplace_back(world_T_camera, params_.cam_intrinsics);
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
  feature_point_graph_values_->AddFeature(feature_id, *world_t_triangulated_point, point_key);
  if (params_.add_point_priors) {
    const gtsam::Vector3 point_prior_noise_sigmas((gtsam::Vector(3) << params_.point_prior_translation_stddev,
                                                   params_.point_prior_translation_stddev,
                                                   params_.point_prior_translation_stddev)
                                                    .finished());
    const auto point_noise =
      Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(point_prior_noise_sigmas)),
             params_.huber_k);
    gtsam::PriorFactor<gtsam::Point3> point_prior_factor(point_key, *world_t_triangulated_point, point_noise);
    graph_factors.push_back(point_prior_factor);
  }
  return true;
}
}  // namespace graph_localizer
