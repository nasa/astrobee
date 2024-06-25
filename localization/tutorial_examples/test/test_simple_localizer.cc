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

// Adds perfect odometry and pose measurements to the graph.
// Eventually adds enough measurements to slide the window.
TEST(SimpleLocalizerTest, Interface) {
  // Create random odometry measurements for timestamps 0 - 9.
  std::vector<lm::PoseWithCovarianceMeasurement>
    odometry_measurements;
  for (int i = 0; i < 10; ++i) {
    odometry_measurements.emplace_back(
      RandomPoseWithCovarianceMeasurement(i));
  }

  // Create random pose measurements for timestamps 0 - 9.
  std::vector<lm::PoseWithCovarianceMeasurement>
    pose_measurements;
  pose_measurements.emplace_back(
    RandomPoseWithCovarianceMeasurement(0));
  for (int i = 1; i < 10; ++i) {
    // Create perfect pose measurement using first pose
    // measurement and subsequent relative odometry.
    const auto relative_pose =
      odometry_measurements[0].pose.inverse() *
      odometry_measurements[i].pose;
    const auto pose = pose_measurements[0].pose * relative_pose;
    pose_measurements.emplace_back(
      lm::PoseWithCovarianceMeasurement(
        pose, lc::RandomPoseCovariance(), i));
  }

  // Initialize localizer using the first pose measurement.
  te::SimpleLocalizerParams params;
  params.pose_node_adder.ideal_duration = 3.0;
  params.pose_node_adder.starting_time =
    pose_measurements[0].timestamp;
  params.pose_node_adder.start_node = pose_measurements[0].pose;
  params.pose_node_adder.start_noise_models.emplace_back(
    gtsam::noiseModel::Gaussian::Covariance(
      pose_measurements[0].covariance));
  te::SimpleLocalizer localizer(params);

  // Add initial odometry estimate so it can be used
  // to compute future relative odometry estimates.
  localizer.AddOdometryMeasurement(odometry_measurements[0]);

  // Adds initial nodes and priors since no pose measurements
  // have been added to the factor adder yet. Optimizes the
  // graph.
  localizer.Update();

  // Initial graph should have the starting pose and prior
  // factor.
  // Expected graph structure:
  // I_0 -> N_0
  // Here I_i is the initial prior pose factor,
  // N_i is a pose node,
  // Index i represents the timestamp.
  // Nodes: 1
  // Factors: Initial Prior: 1, Prior Measurement: 0, Between: 0
  EXPECT_EQ(localizer.timestamped_nodes().size(), 1);
  EXPECT_EQ(lc::NumFactors<gtsam::PriorFactor<gtsam::Pose3>>(
              localizer.factors()),
            1);
  EXPECT_EQ(lc::NumFactors<gtsam::BetweenFactor<gtsam::Pose3>>(
              localizer.factors()),
            0);
  // First node should match first pose measurement pose.
  EXPECT_MATRIX_NEAR(
    localizer.timestamped_nodes().Node(0)->matrix(),
    pose_measurements[0].pose, 1e-6);

  // Add 2 subsequent pose and odometry measurements
  for (int i = 1; i < 3; ++i) {
    localizer.AddOdometryMeasurement(odometry_measurements[i]);
    localizer.AddPoseMeasurement(pose_measurements[i]);
  }

  // Adds pose nodes and pose prior factors for new pose
  // measurements. Adds relative pose factors between these nodes
  // using new odometry measurements. Optimizes the graph.
  localizer.Update();
  // Expected graph structure:
  //                      M_1           M_9
  //                       |             |
  //                       V             V
  // I_0 -> N_0 <-B_0_1-> N_1 <-B_1_2-> N_2
  // M_i is a prior pose factor from a pose measurement,
  // B_i_j is a pose between factor from odometry measurements.
  // Indices i and j represent timestamps.
  // Nodes: 3
  // Factors: Initial Prior: 1, Prior Measurement: 2, Between: 2
  EXPECT_EQ(localizer.timestamped_nodes().size(), 3);
  EXPECT_EQ(localizer.factors().size(), 5);
  EXPECT_EQ(lc::NumFactors<gtsam::PriorFactor<gtsam::Pose3>>(
              localizer.factors()),
            3);
  EXPECT_EQ(lc::NumFactors<gtsam::BetweenFactor<gtsam::Pose3>>(
              localizer.factors()),
            2);
  // Check optimized pose nodes
  EXPECT_MATRIX_NEAR(
    localizer.timestamped_nodes().Node(0)->matrix(),
    pose_measurements[0].pose, 1e-6);
  EXPECT_MATRIX_NEAR(
    localizer.timestamped_nodes().Node(1)->matrix(),
    pose_measurements[1].pose, 1e-6);
  EXPECT_MATRIX_NEAR(
    localizer.timestamped_nodes().Node(2)->matrix(),
    pose_measurements[2].pose, 1e-6);

  // Add 2 more subsequent pose and odometry measurements
  for (int i = 3; i < 5; ++i) {
    localizer.AddOdometryMeasurement(odometry_measurements[i]);
    localizer.AddPoseMeasurement(pose_measurements[i]);
  }

  // Graph has surpassed 3 second duration limit, first node
  // and between factor should be removed.
  localizer.Update();
  // Expected graph structure:
  //        M_1           M_2           M_3           M_4
  //         |             |             |             |
  //         V             V             V             V
  // I_1 -> N_1 <-B_1_2-> N_2 <-B_2_3-> N_3 <-B_3_4-> N_4
  // Nodes: 4
  // Factors: Initial Prior: 1, Prior Measurement: 4, Between: 3
  EXPECT_EQ(localizer.timestamped_nodes().size(), 4);
  EXPECT_EQ(lc::NumFactors<gtsam::PriorFactor<gtsam::Pose3>>(
              localizer.factors()),
            5);
  EXPECT_EQ(lc::NumFactors<gtsam::BetweenFactor<gtsam::Pose3>>(
              localizer.factors()),
            3);
  // Check optimized pose nodes
  EXPECT_MATRIX_NEAR(
    localizer.timestamped_nodes().Node(1)->matrix(),
    pose_measurements[1].pose, 1e-6);
  EXPECT_MATRIX_NEAR(
    localizer.timestamped_nodes().Node(2)->matrix(),
    pose_measurements[2].pose, 1e-6);
  EXPECT_MATRIX_NEAR(
    localizer.timestamped_nodes().Node(3)->matrix(),
    pose_measurements[3].pose, 1e-6);
  EXPECT_MATRIX_NEAR(
    localizer.timestamped_nodes().Node(4)->matrix(),
    pose_measurements[4].pose, 1e-6);

  // Compute covariance example for node at timestamp 2.0
  const auto keys = localizer.timestamped_nodes().Keys(2.0);
  const auto covariance = localizer.Covariance(keys[0]);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
