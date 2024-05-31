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
#include <ros_graph_vio/parameter_reader.h>

#include <ros/package.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace rv = ros_graph_vio;

class RosGraphVIOParameterReaderTest : public ::testing::Test {
 public:
  RosGraphVIOParameterReaderTest() {}

  void SetUp() final {
    lc::SetEnvironmentConfigs();
    config_reader::ConfigReader config;
    config.AddFile("localization/imu_bias_initializer.config");
    config.AddFile("localization/imu_filter.config");
    config.AddFile("localization/ros_graph_vio.config");
    if (!config.ReadFiles()) {
      LogFatal("Failed to read config files.");
    }

    rv::LoadImuBiasInitializerParams(config, imu_params_);
  }

  rv::ImuBiasInitializerParams imu_params_;
};

TEST_F(RosGraphVIOParameterReaderTest, ImuBiasInitializerParams) {
  const std::string astrobee_configs_path = ros::package::getPath("astrobee");
  EXPECT_EQ(imu_params_.imu_bias_filename, astrobee_configs_path + "/resources/imu_bias.config");
  EXPECT_EQ(imu_params_.num_bias_estimation_measurements, 100);
  // IMU filter
  EXPECT_EQ(imu_params_.filter.quiet_accel, "ButterO3S125Lp3N33_33");
  EXPECT_EQ(imu_params_.filter.quiet_ang_vel, "ButterO1S125Lp3N33_33");
  EXPECT_EQ(imu_params_.filter.nominal_accel, "ButterO3S125Lp3N41_66");
  EXPECT_EQ(imu_params_.filter.nominal_ang_vel, "ButterO1S125Lp3N41_66");
  EXPECT_EQ(imu_params_.filter.aggressive_accel, "ButterO3S125Lp3N46_66");
  EXPECT_EQ(imu_params_.filter.aggressive_ang_vel, "ButterO1S125Lp3N46_66");
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
