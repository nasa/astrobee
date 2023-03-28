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

#include <graph_vio/graph_vio.h>

namespace graph_vio {
namespace fa = factor_adders;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace na = node_adders;
namespace no = nodes;
namespace op = optimizers;

GraphVIO::GraphVIO(const GraphVIOParams& params)
    : SlidingWindowGraphOptimizer(params.sliding_window_graph_optimizer,
                                  std::make_unique<op::NonlinearOptimizer>(params.nonlinear_optimizer)),
      params_(params) {
  // Initialize node adders
  combined_nav_state_node_adder_ = std::make_shared<na::CombinedNavStateNodeAdder>(
    params_.combined_nav_state_node_adder, params_.combined_nav_state_node_adder_model, nodes());
  // Initialize factor adders
  vo_smart_projection_factor_adder_ = std::make_shared<fa::VoSmartProjectionFactorAdder<na::CombinedNavStateNodeAdder>>(
    params_.vo_smart_projection_factor_adder, combined_nav_state_node_adder_);
  standstill_factor_adder_ = std::make_shared<fa::StandstillFactorAdder<na::CombinedNavStateNodeAdder>>(
    params_.standstill_factor_adder, combined_nav_state_node_adder_);
}

void GraphVIO::AddImuMeasurement(const lm::ImuMeasurement& imu_measurement) {
  combined_nav_state_node_adder_->AddMeasurement(imu_measurement);
}

void GraphVIO::AddFeaturePointsMeasurement(const lm::FeaturePointsMeasurement& feature_points_measurement) {
  vo_smart_projection_factor_adder_->AddMeasurement(feature_points_measurement);
  // TODO(rsoussan): check for standstill!
}

const no::CombinedNavStateNodes& GraphVIO::combined_nav_state_nodes() const {
  return combined_nav_state_node_adder_->nodes();
}
}  // namespace graph_vio
