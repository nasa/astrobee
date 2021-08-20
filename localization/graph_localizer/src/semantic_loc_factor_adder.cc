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

#include <graph_localizer/semantic_loc_factor_adder.h>
#include <graph_localizer/tolerant_projection_factor.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <gtsam/base/Vector.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lm = localization_measurements;
namespace lc = localization_common;
namespace sym = gtsam::symbol_shorthand;
SemanticLocFactorAdder::SemanticLocFactorAdder(const SemanticLocFactorAdderParams& params,
                                               const go::GraphActionCompleterType graph_action_completer_type)
    : SemanticLocFactorAdder::Base(params), 
      graph_action_completer_type_(graph_action_completer_type) {
  assigner_ = std::make_shared<HungarianAssigner>(params.body_T_cam, *params.cam_intrinsics);
}

void SemanticLocFactorAdder::ComputeFactorsToAdd(std::vector<go::FactorsToAdd> &factors_to_add,
                                                 lm::MatchedProjectionsMeasurement &measurement) {
  if (measurement.matched_projections.empty()) {
    LogDebug("AddFactors: Empty measurement.");
    return;
  }

  if (static_cast<int>(measurement.matched_projections.size()) < params().min_num_matches) {
    LogDebug("AddFactors: Not enough matches in projection measurement.");
    return;
  }

  const int num_objects = measurement.matched_projections.size();
  num_semantic_objects_averager_.Update(num_objects);

  double noise_scale = params().projection_noise_scale;
  if (params().scale_projection_noise_with_num_landmarks) {
    noise_scale *= std::pow((num_semantic_objects_averager_.average() / static_cast<double>(num_objects)), 2);
  }

  int num_tolerant_projection_factors = 0;
  go::FactorsToAdd projection_factors_to_add(type());
  projection_factors_to_add.reserve(measurement.matched_projections.size());
  for (const auto& matched_projection : measurement.matched_projections) {
    const go::KeyInfo key_info(&sym::P, go::NodeUpdaterType::CombinedNavState,
                               measurement.timestamp);
    // TODO(rsoussan): Pass sigma insted of already constructed isotropic noise
    const gtsam::SharedIsotropic scaled_noise(
      gtsam::noiseModel::Isotropic::Sigma(2, noise_scale * params().cam_noise->sigma()));
    gtsam::TolerantProjectionFactor<>::shared_ptr tolerant_projection_factor(new gtsam::TolerantProjectionFactor<>(
      matched_projection.image_point, matched_projection.bounding_box*params().cost_tolerance, matched_projection.map_point, 
      Robust(scaled_noise, params().huber_k), key_info.UninitializedKey(), params().cam_intrinsics, params().body_T_cam));

    projection_factors_to_add.push_back({{key_info}, tolerant_projection_factor});
    ++num_tolerant_projection_factors;
    if (num_tolerant_projection_factors >= params().max_num_factors) break;
  }
  projection_factors_to_add.SetTimestamp(measurement.timestamp);
  LogInfo("AddFactors: Added " << num_tolerant_projection_factors << " tolerant projection factors.");

  factors_to_add.emplace_back(projection_factors_to_add);
}

void SemanticLocFactorAdder::SetCombinedNavState(const boost::optional<localization_common::CombinedNavState>& state) {
  if (state) {
    last_combined_nav_state_ = *state;
  }
}

std::vector<go::FactorsToAdd> SemanticLocFactorAdder::AddFactors(const lm::SemanticDetsMeasurement& semantic_dets) {
  if (std::abs(last_combined_nav_state_.timestamp() - semantic_dets.timestamp) < 0.1) {
    return std::vector<go::FactorsToAdd>(); // return empty set, no factors
  }

  last_matches_.clear();

  // Convert state from gtsam to Eigen
  Eigen::Isometry3d world_T_body = lc::EigenPose(last_combined_nav_state_);

  lm::MatchedProjectionsMeasurement matched_projections_measurement;
  matched_projections_measurement.timestamp = semantic_dets.timestamp;
  // Outer loop through objects so we only do transform/projection once
  LogDebug("SemanticLoc: adding sem loc factors");

  assigner_->assign(world_T_body, semantic_dets.semantic_dets);

  // Actually build factor
  /*
  last_matches_.push_back(SemanticMatch(cls, cam_obj_px, best_det->image_point, best_det->bounding_box));
  lm::MatchedProjection mp(best_det->image_point, best_det->bounding_box, world_T_obj.translation(), semantic_dets.timestamp);
  matched_projections_measurement.matched_projections.push_back(mp);
  */

  // only for visualizing unmatched detections
  for (const auto& det : semantic_dets.semantic_dets) {
    last_matches_.push_back(SemanticMatch(det.class_id, det.image_point, det.bounding_box));
  }

  std::vector<go::FactorsToAdd> factors_to_add;
  ComputeFactorsToAdd(factors_to_add, matched_projections_measurement);

  return factors_to_add;
}

go::GraphActionCompleterType SemanticLocFactorAdder::type() const { return graph_action_completer_type_; }
}  // namespace graph_localizer
