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
#include <tutorial_examples/simple_odometry.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace lm = localization_measurements;
namespace te = tutorial_examples;

// Adds perfect relative pose and IMU measurements to the graph.
TEST(SimpleOdometryTest, Interface) {
  // Initialize odometry at identity with a non-zero velocity and
  // IMU bias.
  te::SimpleOdometryParams params;
  const gtsam::Pose3 start_pose = gtsam::Pose3::identity();
  // Normally the initial velocity and biases estimates come
  // from some initialization method.
  const gtsam::Vector3 start_velocity = Eigen::Vector3d(1, 2, 3);
  const gtsam::Vector3 start_accel_bias = Eigen::Vector3d(0.01, 0.02, 0.03);
  const gtsam::Vector3 start_gyro_bias = Eigen::Vector3d(0.04, 0.05, 0.06);
  params.combined_nav_state_node_adder.starting_time = 0.0;
  params.combined_nav_state_node_adder.start_node = lc::CombinedNavState(
    start_pose, start_velocity, gtsam::imuBias::ConstantBias(start_accel_bias, start_gyro_bias), 0.0);
  // Noise model ordering matching node insertion order,
  // which is pose, velocity, biases for the
  // combined nav state node adder.
  params.combined_nav_state_node_adder.start_noise_models.emplace_back(gtsam::noiseModel::Isotropic::Sigma(6, 0.1));
  params.combined_nav_state_node_adder.start_noise_models.emplace_back(gtsam::noiseModel::Isotropic::Sigma(3, 0.2));
  params.combined_nav_state_node_adder.start_noise_models.emplace_back(gtsam::noiseModel::Isotropic::Sigma(6, 0.3));
  te::SimpleOdometry odometry(params);

  // Adds initial nodes and priors since no relative pose
  // measurements have been added to the factor adder yet.
  // Optimizes the graph.
  odometry.Update();

  // Initial graph should have the starting combined nav state
  // and prior factors.
  // Expected graph structure:
  // I_0 -> N_0
  // Here I_i is the initial combined nav state prior factors,
  // containing pose, velocity, and IMU bias priors, so 3 total.
  // N_i is a combined nav state node containing a
  // pose, velocity, and IMU bias.
  // Index i represents the timestamp.
  // Nodes: 1
  // Factors: Initial Priors: 3, Relative Pose Measurement: 0,
  // Combined Nav State Between: 0
  EXPECT_EQ(odometry.timestamped_nodes().size(), 1);
  EXPECT_EQ(lc::NumFactors<gtsam::PriorFactor<gtsam::Pose3>>(odometry.factors()), 1);
  EXPECT_EQ(lc::NumFactors<gtsam::PriorFactor<gtsam::Velocity3>>(odometry.factors()), 1);
  EXPECT_EQ(lc::NumFactors<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(odometry.factors()), 1);
  EXPECT_EQ(lc::NumFactors<gtsam::BetweenFactor<gtsam::Pose3>>(odometry.factors()), 0);
  EXPECT_EQ(lc::NumFactors<gtsam::CombinedImuFactor>(odometry.factors()), 0);
  // First node should match start node.
  EXPECT_MATRIX_NEAR(odometry.timestamped_nodes().Node(0)->pose(),
                     params.combined_nav_state_node_adder.start_node.pose(), 1e-6);
  EXPECT_MATRIX_NEAR(odometry.timestamped_nodes().Node(0)->velocity(),
                     params.combined_nav_state_node_adder.start_node.velocity(), 1e-6);
  EXPECT_MATRIX_NEAR(odometry.timestamped_nodes().Node(0)->bias().vector(),
                     params.combined_nav_state_node_adder.start_node.bias().vector(), 1e-6);

  // Create relative pose and IMU measurements for timestamps 0
  // - 9. Use constant acceleration and zero angular velocity
  // motion model.
  const Eigen::Vector3d acceleration(0.5, 0.6, 0.7);
  const Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  // Initialize measured biases and velocities to start values.
  // Since in this test we assume biases don't change over time
  // make these constant.
  const Eigen::Vector3d accel_bias = start_accel_bias;
  const Eigen::Vector3d gyro_bias = start_gyro_bias;
  Eigen::Vector3d velocity = start_velocity;
  std::vector<lm::RelativePoseWithCovarianceMeasurement> relative_pose_measurements;
  std::vector<lm::ImuMeasurement> imu_measurements;
  // Add initial zero acceleration measurement
  odometry.AddImuMeasurement(lm::ImuMeasurement(accel_bias, gyro_bias, 0));
  for (int i = 1; i < 3; ++i) {
    const lm::ImuMeasurement imu_measurement(acceleration + accel_bias, angular_velocity + gyro_bias, i);
    imu_measurements.emplace_back(imu_measurement);
    // Motion model for dt = 1.0 sec.
    const gtsam::Pose3 relative_pose = gtsam::Pose3(gtsam::Rot3::identity(), 0.5 * acceleration + velocity);
    velocity += acceleration;
    relative_pose_measurements.emplace_back(
      lm::RelativePoseWithCovarianceMeasurement(relative_pose, lc::RandomPoseCovariance(), i - 1, i));
  }

  // Add 2 IMU and relative pose measurements to the odometry
  // graph
  for (int i = 0; i < 2; ++i) {
    odometry.AddImuMeasurement(imu_measurements[i]);
    odometry.AddRelativePoseMeasurement(relative_pose_measurements[i]);
  }

  // Adds relative pose factors and CombinedNavState nodes for
  // new relative pose measurements. Adds CombinedIMU factors
  // between these nodes using new IMU measurements. Optimizes
  // the graph.
  odometry.Update();
  // Expected graph structure:
  //            <-B_0_1->     <-B_1_2->
  // I_0 -> N_0           N_1           N_2
  //            <-R_0_1->     <-R_1_2->
  // R_i_j is a relative pose factor from relative pose
  // measurements. B_i_j is a CombinedIMU factor from IMU
  // measurements. Indices i and j represent timestamps. Nodes: 3
  // Factors: Initial Priors: 3, Relative Pose: 2, CombinedIMU: 2
  EXPECT_EQ(odometry.timestamped_nodes().size(), 3);
  EXPECT_EQ(lc::NumFactors<gtsam::PriorFactor<gtsam::Pose3>>(odometry.factors()), 1);
  EXPECT_EQ(lc::NumFactors<gtsam::PriorFactor<gtsam::Velocity3>>(odometry.factors()), 1);
  EXPECT_EQ(lc::NumFactors<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(odometry.factors()), 1);
  EXPECT_EQ(lc::NumFactors<gtsam::BetweenFactor<gtsam::Pose3>>(odometry.factors()), 2);
  EXPECT_EQ(lc::NumFactors<gtsam::CombinedImuFactor>(odometry.factors()), 2);
  // Check optimized pose nodes
  // First node should match start node.
  EXPECT_MATRIX_NEAR(odometry.timestamped_nodes().Node(0)->pose(),
                     params.combined_nav_state_node_adder.start_node.pose(), 1e-6);
  EXPECT_MATRIX_NEAR(odometry.timestamped_nodes().Node(0)->velocity(),
                     params.combined_nav_state_node_adder.start_node.velocity(), 1e-6);
  EXPECT_MATRIX_NEAR(odometry.timestamped_nodes().Node(0)->bias().vector(),
                     params.combined_nav_state_node_adder.start_node.bias().vector(), 1e-6);
  // Second node
  EXPECT_MATRIX_NEAR(
    odometry.timestamped_nodes().Node(1)->pose(),
    gtsam::Pose3(params.combined_nav_state_node_adder.start_node.pose() * relative_pose_measurements[0].relative_pose),
    1e-6);
  EXPECT_MATRIX_NEAR(odometry.timestamped_nodes().Node(1)->velocity(),
                     params.combined_nav_state_node_adder.start_node.velocity() + acceleration, 1e-6);
  EXPECT_MATRIX_NEAR(odometry.timestamped_nodes().Node(1)->bias().vector(),
                     params.combined_nav_state_node_adder.start_node.bias().vector(), 1e-6);
  // Third node
  EXPECT_MATRIX_NEAR(
    odometry.timestamped_nodes().Node(2)->pose(),
    gtsam::Pose3(params.combined_nav_state_node_adder.start_node.pose() * relative_pose_measurements[0].relative_pose *
                 relative_pose_measurements[1].relative_pose),
    1e-6);
  EXPECT_MATRIX_NEAR(odometry.timestamped_nodes().Node(2)->velocity(),
                     gtsam::Velocity3(params.combined_nav_state_node_adder.start_node.velocity() + acceleration * 2),
                     1e-6);
  EXPECT_MATRIX_NEAR(odometry.timestamped_nodes().Node(2)->bias().vector(),
                     params.combined_nav_state_node_adder.start_node.bias().vector(), 1e-6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
