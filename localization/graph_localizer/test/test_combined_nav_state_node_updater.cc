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

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
