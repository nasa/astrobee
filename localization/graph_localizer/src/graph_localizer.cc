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

#include <graph_localizer/graph_localizer.h>

namespace graph_localizer {
namespace fa = factor_adders;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace na = node_adders;
namespace no = nodes;
namespace op = optimizers;

GraphLocalizer::GraphLocalizer(const GraphLocalizerParams& params)
    : SlidingWindowGraphOptimizer(params.sliding_window_graph_optimizer,
                                  std::make_unique<op::NonlinearOptimizer>(params.nonlinear_optimizer)),
      params_(params) {
  // Initialize sliding window node adders
  pose_node_adder_ =
    std::make_shared<na::PoseNodeAdder>(params_.pose_node_adder, params_.pose_node_adder_model, values());
  AddSlidingWindowNodeAdder(pose_node_adder_);
  // Initialize factor adders
  sparse_map_loc_factor_adder_ =
    std::make_shared<fa::LocFactorAdder<na::PoseNodeAdder>>(params_.sparse_map_loc_factor_adder, pose_node_adder_);
  AddFactorAdder(sparse_map_loc_factor_adder_);
  ar_tag_loc_factor_adder_ =
    std::make_shared<fa::LocFactorAdder<na::PoseNodeAdder>>(params_.ar_tag_loc_factor_adder, pose_node_adder_);
  AddFactorAdder(ar_tag_loc_factor_adder_);
}

void GraphLocalizer::AddPoseMeasurement(const lm::PoseWithCovarianceMeasurement& pose_measurement) {
  pose_node_adder_->AddMeasurement(pose_measurement);
}

void GraphLocalizer::AddSparseMapMatchedProjectionsMeasurement(
  const lm::MatchedProjectionsMeasurement& matched_projections_measurement) {
  if (params_.sparse_map_loc_factor_adder.enabled)
    sparse_map_loc_factor_adder_->AddMeasurement(matched_projections_measurement);
}

void GraphLocalizer::AddArTagMatchedProjectionsMeasurement(
  const lm::MatchedProjectionsMeasurement& matched_projections_measurement) {
  if (params_.ar_tag_loc_factor_adder.enabled)
    ar_tag_loc_factor_adder_->AddMeasurement(matched_projections_measurement);
}

const no::TimestampedNodes<gtsam::Pose3>& GraphLocalizer::pose_nodes() const { return pose_node_adder_->nodes(); }

void GraphLocalizer::SetPoseCovarianceInterpolater(
  const std::shared_ptr<lc::MarginalsPoseCovarianceInterpolater<no::CombinedNavStateNodes>>&
    pose_covariance_interpolater) {
  pose_node_adder_->node_adder_model().pose_interpolater().params().pose_covariance_interpolater =
    pose_covariance_interpolater;
}
}  // namespace graph_localizer
