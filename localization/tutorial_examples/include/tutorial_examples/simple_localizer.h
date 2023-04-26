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

#ifndef TUTORIAL_EXAMPLES_SIMPLE_LOCALIZER_H_
#define TUTORIAL_EXAMPLES_SIMPLE_LOCALIZER_H_

#include <localization_measurements/pose_measurement.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer.h>
#include <tutorial_examples/absolute_pose_factor_adder.h>
#include <tutorial_examples/relative_pose_node_adder.h>

namespace tutorial_examples {
using SimpleLocalizerParams = sliding_window_graph_optimizer::SlidingWindowGraphOptimizerParams;

class SimpleLocalizer : public sliding_window_graph_optimizer::SlidingWindowGraphOptimizer {
 public:
  explicit SimpleLocalizer(const SimpleLocalizerParams& params) : params_(params) {
    // Initialize node and factor adders
    RelativePoseNodeAdderParams node_adder_params;
    node_adder_params.start_noise_models.emplace_back(gtsam::noiseModel::Isotropic::Sigma(6, 0.1));
    node_adder_ =
      std::make_shared<RelativePoseNodeAdder>(node_adder_params, RelativePoseNodeAdderModel::Params(), values());
    factor_adder_ = std::make_shared<AbsolutePoseFactorAdder>(AbsolutePoseFactorAdderParams(), node_adder_);
    // Register node and factor adders
    AddSlidingWindowNodeAdder(node_adder_);
    AddFactorAdder(factor_adder_);
  }

  void AddRelativePoseMeasurement(const localization_measurements::PoseWithCovarianceMeasurement& measurement) {
    node_adder_->AddMeasurement(measurement);
  }

  void AddAbsolutePoseMeasurement(const localization_measurements::PoseWithCovarianceMeasurement& measurement) {
    factor_adder_->AddMeasurement(measurement);
  }

  const nodes::TimestampedNodes<gtsam::Pose3>& timestamped_nodes() const { return node_adder_->nodes(); }

 private:
  SimpleLocalizerParams params_;

  std::shared_ptr<AbsolutePoseFactorAdder> factor_adder_;
  std::shared_ptr<RelativePoseNodeAdder> node_adder_;
};
}  // namespace tutorial_examples

#endif  // TUTORIAL_EXAMPLES_SIMPLE_LOCALIZER_H_
