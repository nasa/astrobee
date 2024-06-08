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

#ifndef TUTORIAL_EXAMPLES_SIMPLE_ODOMETRY_H_
#define TUTORIAL_EXAMPLES_SIMPLE_ODOMETRY_H_

#include <factor_adders/relative_pose_factor_adder.h>
#include <localization_measurements/imu_measurement.h>
#include <localization_measurements/relative_pose_with_covariance_measurement.h>
#include <node_adders/combined_nav_state_node_adder.h>
#include <optimizers/nonlinear_optimizer.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer.h>
#include <tutorial_examples/simple_odometry_params.h>

namespace tutorial_examples {
// Simple Odometry that adds relative pose factors using
// relative pose measurements (for example, from matching
// successive sensor data) and adds pose, velocity, IMU bias
// nodes using IMU measurements. Uses a sliding window graph
// optimizer and Levenberg-Marquardt nonlinear optimization.
class SimpleOdometry : public sliding_window_graph_optimizer::
                         SlidingWindowGraphOptimizer {
 public:
  // Initialize and register the node and factor adders.
  explicit SimpleOdometry(const SimpleOdometryParams& params)
      : SlidingWindowGraphOptimizer(
          params.sliding_window_graph_optimizer,
          std::make_unique<optimizers::NonlinearOptimizer>(
            params.nonlinear_optimizer)) {
    // Initialize node and factor adders
    node_adder_ =
      std::make_shared<node_adders::CombinedNavStateNodeAdder>(
        params.combined_nav_state_node_adder,
        params.combined_nav_state_node_adder_model, values());
    factor_adder_ =
      std::make_shared<factor_adders::RelativePoseFactorAdder<
        node_adders::CombinedNavStateNodeAdder>>(
        params.relative_pose_factor_adder, node_adder_);
    // Register node and factor adders
    AddSlidingWindowNodeAdder(node_adder_);
    AddFactorAdder(factor_adder_);
  }

  // Adds an IMU measurement to the combined nav state
  // node adder.
  void AddImuMeasurement(
    const localization_measurements::ImuMeasurement&
      measurement) {
    node_adder_->AddMeasurement(measurement);
  }

  // Adds relative pose measurement to the relative pose node
  // adder. Assumes the pose is in the odometry frame.
  void AddRelativePoseMeasurement(
    const localization_measurements::
      RelativePoseWithCovarianceMeasurement& measurement) {
    factor_adder_->AddMeasurement(measurement);
  }

  // Const accessor for combined nav state nodes
  // containing a pose, velocity, and IMU bias in each node.
  const nodes::CombinedNavStateNodes& timestamped_nodes() const {
    return node_adder_->nodes();
  }

 private:
  std::shared_ptr<factor_adders::RelativePoseFactorAdder<
    node_adders::CombinedNavStateNodeAdder>>
    factor_adder_;
  std::shared_ptr<node_adders::CombinedNavStateNodeAdder>
    node_adder_;
};
}  // namespace tutorial_examples

#endif  // TUTORIAL_EXAMPLES_SIMPLE_ODOMETRY_H_
