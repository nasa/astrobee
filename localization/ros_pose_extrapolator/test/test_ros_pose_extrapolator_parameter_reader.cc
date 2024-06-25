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

#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <node_adders/utilities.h>
#include <ros_pose_extrapolator/parameter_reader.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace rp = ros_pose_extrapolator;

class RosPoseExtrapolatorParameterReaderTest : public ::testing::Test {
 public:
  RosPoseExtrapolatorParameterReaderTest() {}

  void SetUp() final {
    lc::SetEnvironmentConfigs();
    config_reader::ConfigReader config;
    config.AddFile("localization/imu_filter.config");
    config.AddFile("localization/imu_integrator.config");
    config.AddFile("localization/ros_pose_extrapolator.config");
    config.AddFile("transforms.config");
    if (!config.ReadFiles()) {
      LogFatal("Failed to read config files.");
    }
    rp::LoadRosPoseExtrapolatorParams(config, params_);
  }

  rp::RosPoseExtrapolatorParams params_;
};

TEST_F(RosPoseExtrapolatorParameterReaderTest, RosPoseExtrapolatorParams) {
  EXPECT_EQ(params_.standstill_enabled, true);
  // IMU Integrator
  EXPECT_MATRIX_NEAR(params_.imu_integrator.gravity, Eigen::Vector3d::Zero(), 1e-6);
  // Taken using current nav cam extrinsics
  const gtsam::Pose3 expected_body_T_imu(gtsam::Rot3(0.70710678118, 0, 0, 0.70710678118),
                                         gtsam::Point3(0.0386, 0.0247, -0.01016));
  EXPECT_MATRIX_NEAR(params_.imu_integrator.body_T_imu, expected_body_T_imu, 1e-6);
  EXPECT_NEAR(params_.imu_integrator.gyro_sigma, 0.00001, 1e-6);
  EXPECT_NEAR(params_.imu_integrator.accel_sigma, 0.0005, 1e-6);
  EXPECT_NEAR(params_.imu_integrator.accel_bias_sigma, 0.0005, 1e-6);
  EXPECT_NEAR(params_.imu_integrator.gyro_bias_sigma, 0.0000035, 1e-6);
  EXPECT_NEAR(params_.imu_integrator.integration_variance, 0.0001, 1e-6);
  EXPECT_NEAR(params_.imu_integrator.bias_acc_omega_int, 0.00015, 1e-6);
  // IMU filter
  EXPECT_EQ(params_.imu_integrator.filter.quiet_accel, "ButterO3S125Lp3N33_33");
  EXPECT_EQ(params_.imu_integrator.filter.quiet_ang_vel, "ButterO1S125Lp3N33_33");
  EXPECT_EQ(params_.imu_integrator.filter.nominal_accel, "ButterO3S125Lp3N41_66");
  EXPECT_EQ(params_.imu_integrator.filter.nominal_ang_vel, "ButterO1S125Lp3N41_66");
  EXPECT_EQ(params_.imu_integrator.filter.aggressive_accel, "ButterO3S125Lp3N46_66");
  EXPECT_EQ(params_.imu_integrator.filter.aggressive_ang_vel, "ButterO1S125Lp3N46_66");
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
