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

#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <tutorial_examples/simple_localizer.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace lm = localization_measurements;
namespace te = tutorial_examples;

// Generate a random pose measurement at the provided timestamp.
lm::PoseWithCovarianceMeasurement
RandomPoseWithCovarianceMeasurement(const lc::Time time) {
  return lm::PoseWithCovarianceMeasurement(
    lc::RandomPoseWithCovariance(), time);
}

TEST(SimpleLocalizerTest, Interface) {
  te::SimpleLocalizerParams params;
  // Initialize localizer with first absolute pose measurement.
  const lc::Time initial_time = 0.0;
  const auto initial_measurement =
    RandomPoseWithCovarianceMeasurement(initial_time);
  params.relative_pose_node_adder.start_measurement =
    initial_measurement;
  params.relative_pose_node_adder.start_node =
    initial_measurement.pose;
  params.relative_pose_node_adder.start_noise_models
    .emplace_back(gtsam::noiseModel::Gaussian::Covariance(
      initial_measurement.covariance));
  te::SimpleLocalizer localizer(params);

  // Add relative and absolute pose measurements at successive
  // timestamps.
  for (int i = 1; i < 10; ++i) {
    localizer.AddRelativePoseMeasurement(
      RandomPoseWithCovarianceMeasurement(i));
    localizer.AddAbsolutePoseMeasurement(
      RandomPoseWithCovarianceMeasurement(i));
  }

  // Add nodes and factors, slide window, and optimize using
  // added measurements.
  localizer.Update();

  // Access optimized timestamped nodes
  const auto& timestamped_nodes = localizer.timestamped_nodes();
  // Access optimized GTSAM values
  const auto& values = localizer.values();
  // Access GTSAM factors
  const auto& factors = localizer.factors();
  // Compute covariance for a node at timestamp 1
  const auto keys = timestamped_nodes.Keys(1);
  const auto covariance = localizer.Covariance(keys[0]);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
