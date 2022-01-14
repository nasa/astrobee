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
#include <graph_localizer/graph_localizer_params.h>
#include <graph_localizer/test_utilities.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/imu_measurement.h>

#include <gtest/gtest.h>

namespace gl = graph_localizer;
namespace lc = localization_common;
namespace lm = localization_measurements;

TEST(CombinedNavStateNodeUpdaterTester, ConstantVelocity) {
  auto params = gl::DefaultGraphLocalizerParams();
  // Use depth odometry factor adder since it can add relative pose factors
  params.factor.depth_odometry_adder = gl::DefaultDepthOdometryFactorAdderParams();
  constexpr double kInitialVelocity = 0.1;
  params.graph_initializer.global_V_body_start = Eigen::Vector3d(kInitialVelocity, 0, 0);
  gl::GraphLocalizer graph_localizer(params);
  constexpr int kNumIterations = 100;
  constexpr double kTimeDiff = 0.1;
  lc::Time time = 0.0;
  const Eigen::Vector3d relative_translation = kTimeDiff * params.graph_initializer.global_V_body_start;
  Eigen::Isometry3d current_pose = lc::EigenPose(params.graph_initializer.global_T_body_start);
  // Add initial zero acceleration value so the imu integrator has more than one measurement when the subsequent
  // measurement is added
  const lm::ImuMeasurement initial_zero_imu_measurement(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time);
  graph_localizer.AddImuMeasurement(initial_zero_imu_measurement);
  for (int i = 0; i < kNumIterations; ++i) {
    time += kTimeDiff;
    const lm::ImuMeasurement zero_imu_measurement(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time);
    graph_localizer.AddImuMeasurement(zero_imu_measurement);
    const Eigen::Isometry3d relative_pose = lc::Isometry3d(relative_translation, Eigen::Matrix3d::Identity());
    current_pose = current_pose * relative_pose;
    const lc::Time source_time = time - kTimeDiff;
    const lc::Time target_time = time;
    const lm::DepthOdometryMeasurement constant_velocity_measurement =
      gl::DepthOdometryMeasurementFromPose(relative_pose, source_time, target_time);
    graph_localizer.AddDepthOdometryMeasurement(constant_velocity_measurement);
    graph_localizer.Update();
    const auto latest_combined_nav_state = graph_localizer.LatestCombinedNavState();
    ASSERT_TRUE(latest_combined_nav_state != boost::none);
    EXPECT_NEAR(latest_combined_nav_state->timestamp(), time, 1e-6);
    EXPECT_TRUE(lc::MatrixEquality<5>(latest_combined_nav_state->pose().matrix(), current_pose.matrix()));
  }
}

TEST(CombinedNavStateNodeUpdaterTester, ConstantAcceleration) {
  auto params = gl::DefaultGraphLocalizerParams();
  // Use depth odometry factor adder since it can add relative pose factors
  params.factor.depth_odometry_adder = gl::DefaultDepthOdometryFactorAdderParams();
  gl::GraphLocalizer graph_localizer(params);
  constexpr int kNumIterations = 100;
  constexpr double kTimeDiff = 0.1;
  lc::Time time = 0.0;
  Eigen::Isometry3d current_pose = lc::EigenPose(params.graph_initializer.global_T_body_start);
  const Eigen::Vector3d acceleration(0.01, 0.02, 0.03);
  Eigen::Vector3d velocity = params.graph_initializer.global_V_body_start;
  // Add initial zero acceleration value so the imu integrator has more than one measurement when the subsequent
  // measurement is added
  const lm::ImuMeasurement zero_imu_measurement(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time);
  graph_localizer.AddImuMeasurement(zero_imu_measurement);
  for (int i = 0; i < kNumIterations; ++i) {
    time += kTimeDiff;
    const lm::ImuMeasurement imu_measurement(acceleration, Eigen::Vector3d::Zero(), time);
    graph_localizer.AddImuMeasurement(imu_measurement);
    const Eigen::Vector3d relative_translation = velocity * kTimeDiff + 0.5 * acceleration * kTimeDiff * kTimeDiff;
    velocity += acceleration * kTimeDiff;
    const Eigen::Isometry3d relative_pose = lc::Isometry3d(relative_translation, Eigen::Matrix3d::Identity());
    current_pose = current_pose * relative_pose;
    const lc::Time source_time = time - kTimeDiff;
    const lc::Time target_time = time;
    const lm::DepthOdometryMeasurement constant_acceleration_measurement =
      gl::DepthOdometryMeasurementFromPose(relative_pose, source_time, target_time);
    graph_localizer.AddDepthOdometryMeasurement(constant_acceleration_measurement);
    graph_localizer.Update();
    const auto latest_combined_nav_state = graph_localizer.LatestCombinedNavState();
    ASSERT_TRUE(latest_combined_nav_state != boost::none);
    EXPECT_NEAR(latest_combined_nav_state->timestamp(), time, 1e-6);
    EXPECT_TRUE(lc::MatrixEquality<5>(latest_combined_nav_state->pose().matrix(), current_pose.matrix()));
  }
}

TEST(CombinedNavStateNodeUpdaterTester, ConstantAccelerationNonZeroBias) {
  const Eigen::Vector3d acceleration(0.01, 0.02, 0.03);
  const Eigen::Vector3d angular_velocity(0.04, 0.05, 0.06);
  const Eigen::Vector3d acceleration_bias = 0.5 * acceleration;
  const Eigen::Vector3d angular_velocity_bias = angular_velocity;
  const Eigen::Vector3d bias_corrected_acceleration = acceleration - acceleration_bias;
  const Eigen::Vector3d bias_corrected_angular_velocity = angular_velocity - angular_velocity_bias;
  auto params = gl::DefaultGraphLocalizerParams();
  params.graph_initializer.initial_imu_bias = gtsam::imuBias::ConstantBias(acceleration_bias, angular_velocity_bias);
  // Use depth odometry factor adder since it can add relative pose factors
  params.factor.depth_odometry_adder = gl::DefaultDepthOdometryFactorAdderParams();
  gl::GraphLocalizer graph_localizer(params);
  constexpr int kNumIterations = 100;
  constexpr double kTimeDiff = 0.1;
  lc::Time time = 0.0;
  Eigen::Isometry3d current_pose = lc::EigenPose(params.graph_initializer.global_T_body_start);
  Eigen::Vector3d velocity = params.graph_initializer.global_V_body_start;
  // Add initial zero imu value so the imu integrator has more than one measurement when the subsequent
  // measurement is added
  const lm::ImuMeasurement zero_imu_measurement(acceleration_bias, angular_velocity_bias, time);
  graph_localizer.AddImuMeasurement(zero_imu_measurement);
  for (int i = 0; i < kNumIterations; ++i) {
    time += kTimeDiff;
    const lm::ImuMeasurement imu_measurement(acceleration, angular_velocity, time);
    graph_localizer.AddImuMeasurement(imu_measurement);
    const Eigen::Vector3d relative_translation =
      velocity * kTimeDiff + 0.5 * bias_corrected_acceleration * kTimeDiff * kTimeDiff;
    velocity += bias_corrected_acceleration * kTimeDiff;
    const Eigen::Isometry3d relative_pose = lc::Isometry3d(relative_translation, Eigen::Matrix3d::Identity());
    current_pose = current_pose * relative_pose;
    const lc::Time source_time = time - kTimeDiff;
    const lc::Time target_time = time;
    const lm::DepthOdometryMeasurement constant_acceleration_measurement =
      gl::DepthOdometryMeasurementFromPose(relative_pose, source_time, target_time);
    graph_localizer.AddDepthOdometryMeasurement(constant_acceleration_measurement);
    graph_localizer.Update();
    const auto latest_combined_nav_state = graph_localizer.LatestCombinedNavState();
    ASSERT_TRUE(latest_combined_nav_state != boost::none);
    EXPECT_NEAR(latest_combined_nav_state->timestamp(), time, 1e-6);
    EXPECT_TRUE(lc::MatrixEquality<5>(latest_combined_nav_state->pose().matrix(), current_pose.matrix()));
  }
}

TEST(CombinedNavStateNodeUpdaterTester, ConstantAccelerationConstantAngularVelocityNonZeroBias) {
  const Eigen::Vector3d acceleration(0.01, 0.02, 0.03);
  const Eigen::Vector3d angular_velocity(0.04, 0.05, 0.06);
  const Eigen::Vector3d acceleration_bias = 0.5 * acceleration;
  const Eigen::Vector3d angular_velocity_bias = 0.5 * angular_velocity;
  const Eigen::Vector3d bias_corrected_acceleration = acceleration - acceleration_bias;
  const Eigen::Vector3d bias_corrected_angular_velocity = angular_velocity - angular_velocity_bias;
  auto params = gl::DefaultGraphLocalizerParams();
  params.graph_initializer.initial_imu_bias = gtsam::imuBias::ConstantBias(acceleration_bias, angular_velocity_bias);
  // Use depth odometry factor adder since it can add relative pose factors
  params.factor.depth_odometry_adder = gl::DefaultDepthOdometryFactorAdderParams();
  gl::GraphLocalizer graph_localizer(params);
  constexpr int kNumIterations = 100;
  constexpr double kTimeDiff = 0.1;
  lc::Time time = 0.0;
  Eigen::Isometry3d current_pose = lc::EigenPose(params.graph_initializer.global_T_body_start);
  Eigen::Vector3d velocity = params.graph_initializer.global_V_body_start;
  // Add initial zero imu value so the imu integrator has more than one measurement when the subsequent
  // measurement is added
  const lm::ImuMeasurement zero_imu_measurement(acceleration_bias, angular_velocity_bias, time);
  graph_localizer.AddImuMeasurement(zero_imu_measurement);
  for (int i = 0; i < kNumIterations; ++i) {
    time += kTimeDiff;
    const lm::ImuMeasurement imu_measurement(acceleration, angular_velocity, time);
    graph_localizer.AddImuMeasurement(imu_measurement);
    const Eigen::Matrix3d relative_orientation =
      (gtsam::Rot3::Expmap(bias_corrected_angular_velocity * kTimeDiff)).matrix();
    const Eigen::Vector3d relative_translation =
      velocity * kTimeDiff + 0.5 * bias_corrected_acceleration * kTimeDiff * kTimeDiff;
    velocity += bias_corrected_acceleration * kTimeDiff;
    // Put velocity in new body frame after integrating accelerations
    velocity = relative_orientation.transpose() * velocity;
    const Eigen::Isometry3d relative_pose = lc::Isometry3d(relative_translation, relative_orientation);
    current_pose = current_pose * relative_pose;
    const lc::Time source_time = time - kTimeDiff;
    const lc::Time target_time = time;
    const lm::DepthOdometryMeasurement constant_acceleration_measurement =
      gl::DepthOdometryMeasurementFromPose(relative_pose, source_time, target_time);
    graph_localizer.AddDepthOdometryMeasurement(constant_acceleration_measurement);
    graph_localizer.Update();
    const auto latest_combined_nav_state = graph_localizer.LatestCombinedNavState();
    ASSERT_TRUE(latest_combined_nav_state != boost::none);
    EXPECT_NEAR(latest_combined_nav_state->timestamp(), time, 1e-6);
    EXPECT_TRUE(lc::MatrixEquality<5>(latest_combined_nav_state->pose().matrix(), current_pose.matrix()));
  }
}

TEST(CombinedNavStateNodeUpdaterTester, SlidingWindow) {
  auto params = gl::DefaultGraphLocalizerParams();
  params.combined_nav_state_node_updater.graph_values.max_num_states = 1000000;
  // Use depth odometry factor adder since it can add relative pose factors
  params.factor.depth_odometry_adder = gl::DefaultDepthOdometryFactorAdderParams();
  constexpr double kInitialVelocity = 0.1;
  params.graph_initializer.global_V_body_start = Eigen::Vector3d(kInitialVelocity, 0, 0);
  gl::GraphLocalizer graph_localizer(params);
  constexpr int kNumIterations = 100;
  constexpr double kTimeDiff = 0.1;
  const int max_num_states_in_sliding_window =
    params.combined_nav_state_node_updater.graph_values.ideal_duration / kTimeDiff;
  lc::Time time = 0.0;
  const Eigen::Vector3d relative_translation = kTimeDiff * params.graph_initializer.global_V_body_start;
  Eigen::Isometry3d current_pose = lc::EigenPose(params.graph_initializer.global_T_body_start);
  // Add initial zero acceleration value so the imu integrator has more than one measurement when the subsequent
  // measurement is added
  const lm::ImuMeasurement initial_zero_imu_measurement(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time);
  graph_localizer.AddImuMeasurement(initial_zero_imu_measurement);
  lc::Time last_time = 0;
  for (int i = 0; i < kNumIterations; ++i) {
    last_time = time;
    time += kTimeDiff;
    const lm::ImuMeasurement zero_imu_measurement(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time);
    graph_localizer.AddImuMeasurement(zero_imu_measurement);
    const Eigen::Isometry3d relative_pose = lc::Isometry3d(relative_translation, Eigen::Matrix3d::Identity());
    current_pose = current_pose * relative_pose;
    const lc::Time source_time = last_time;
    const lc::Time target_time = time;
    const lm::DepthOdometryMeasurement constant_velocity_measurement =
      gl::DepthOdometryMeasurementFromPose(relative_pose, source_time, target_time);
    graph_localizer.AddDepthOdometryMeasurement(constant_velocity_measurement);
    graph_localizer.Update();
    const auto latest_combined_nav_state = graph_localizer.LatestCombinedNavState();
    ASSERT_TRUE(latest_combined_nav_state != boost::none);
    EXPECT_NEAR(latest_combined_nav_state->timestamp(), time, 1e-6);
    EXPECT_TRUE(lc::MatrixEquality<5>(latest_combined_nav_state->pose().matrix(), current_pose.matrix()));
    // Check num states, ensure window is sliding properly
    // i + 2 since graph is initialized with a starting state and i = 0 also adds a state
    const int total_states_added = i + 2;
    EXPECT_EQ(graph_localizer.combined_nav_state_graph_values().NumStates(),
              std::min(total_states_added, max_num_states_in_sliding_window));
    const auto& combined_nav_state_node_updater = graph_localizer.combined_nav_state_node_updater();
    // Check latest and oldest timestamps
    {
      const auto oldest_timestamp = combined_nav_state_node_updater.OldestTimestamp();
      ASSERT_TRUE(oldest_timestamp != boost::none);
      EXPECT_NEAR(*oldest_timestamp,
                  std::max(0.0, time - params.combined_nav_state_node_updater.graph_values.ideal_duration), 1e-6);
      const auto latest_timestamp = combined_nav_state_node_updater.LatestTimestamp();
      ASSERT_TRUE(latest_timestamp != boost::none);
      EXPECT_NEAR(*latest_timestamp, time, 1e-6);
    }
    // Check corect factors are in graph
    {
      // i + 1 factors for each relative pose factor and imu factor and 3 prior factors for pose, velocity, and bias
      const int num_relative_factors = std::min(30, i + 1);
      const int num_prior_factors = 3;
      // Max 30 relative pose and imu factors due to sliding graph size
      EXPECT_EQ(graph_localizer.num_factors(), 2 * num_relative_factors + num_prior_factors);
      const auto imu_factors = graph_localizer.Factors<gtsam::CombinedImuFactor>();
      EXPECT_EQ(imu_factors.size(), num_relative_factors);
      const auto rel_pose_factors = graph_localizer.Factors<gtsam::BetweenFactor<gtsam::Pose3>>();
      EXPECT_EQ(rel_pose_factors.size(), num_relative_factors);
      // check num prior factors!
      // make sure priors are on correct nodes!
    }
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
