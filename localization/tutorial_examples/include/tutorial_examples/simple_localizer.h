/* Copyright (c) 2017, United States Government, as represented
 * by the Administrator of the National Aeronautics and Space
 * Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License,
 * Version 2.0 (the "License"); you may not use this file except
 * in compliance with the License. You may obtain a copy of the
 * License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the
 * License.
 */

#ifndef TUTORIAL_EXAMPLES_SIMPLE_LOCALIZER_H_
#define TUTORIAL_EXAMPLES_SIMPLE_LOCALIZER_H_

#include <factor_adders/pose_factor_adder.h>
#include <localization_measurements/pose_measurement.h>
#include <node_adders/pose_node_adder.h>
#include <optimizers/nonlinear_optimizer.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer.h>
#include <tutorial_examples/simple_localizer_params.h>

namespace tutorial_examples {
// Simple localizer that adds absolute pose factors using
// absolute pose measurements (for example, from matches to a
// preexisting map) and adds pose nodes and relative factors
// using relative pose measurements (for example, from an
// odometry provider). Uses a sliding window graph optimizer
// and Levenberg-Marquardt nonlinear optimization.
class SimpleLocalizer : public sliding_window_graph_optimizer::
                          SlidingWindowGraphOptimizer {
 public:
  // Initialize and register the node and factor adders.
  explicit SimpleLocalizer(const SimpleLocalizerParams& params)
      : SlidingWindowGraphOptimizer(
          params.sliding_window_graph_optimizer,
          std::make_unique<optimizers::NonlinearOptimizer>(
            params.nonlinear_optimizer)) {
    // Initialize node and factor adders
    node_adder_ = std::make_shared<node_adders::PoseNodeAdder>(
      params.pose_node_adder, params.pose_node_adder_model,
      values());
    factor_adder_ =
      std::make_shared<factor_adders::PoseFactorAdder<
        node_adders::PoseNodeAdder>>(params.pose_factor_adder,
                                     node_adder_);
    // Register node and factor adders
    AddSlidingWindowNodeAdder(node_adder_);
    AddFactorAdder(factor_adder_);
  }

  // Adds an odometry pose measurement to the pose node
  // adder. Assumes the pose is in the odometry frame.
  void AddOdometryMeasurement(
    const localization_measurements::
      PoseWithCovarianceMeasurement& measurement) {
    node_adder_->AddMeasurement(measurement);
  }

  // Adds an absolute pose measurement to the pose
  // factor adder. Assumes the pose is in the world frame.
  void AddPoseMeasurement(
    const localization_measurements::
      PoseWithCovarianceMeasurement& measurement) {
    factor_adder_->AddMeasurement(measurement);
  }

  // Const accessor for optimized pose nodes.
  const nodes::TimestampedNodes<gtsam::Pose3>&
  timestamped_nodes() const {
    return node_adder_->nodes();
  }

 private:
  std::shared_ptr<
    factor_adders::PoseFactorAdder<node_adders::PoseNodeAdder>>
    factor_adder_;
  std::shared_ptr<node_adders::PoseNodeAdder> node_adder_;
};
}  // namespace tutorial_examples

#endif  // TUTORIAL_EXAMPLES_SIMPLE_LOCALIZER_H_
