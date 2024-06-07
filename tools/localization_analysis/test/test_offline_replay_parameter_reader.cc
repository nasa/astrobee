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
#include <localization_analysis/parameter_reader.h>

#include <gtest/gtest.h>

namespace la = localization_analysis;
namespace lc = localization_common;

class OfflineReplayParameterReaderTest : public ::testing::Test {
 public:
  OfflineReplayParameterReaderTest() {}

  void SetUp() final {
    lc::SetEnvironmentConfigs();
    config_reader::ConfigReader config;
    config.AddFile("tools/offline_replay.config");
    lc::LoadGraphLocalizerConfig(config);
    lc::LoadGraphVIOConfig(config);

    la::LoadLiveMeasurementSimulatorParams(config, "test.bag", "test.map", "test_topic",
                                           live_measurement_simulator_params_);
    live_measurement_simulator_params_.use_bag_image_feature_msgs = true;
    la::LoadGraphLocalizerSimulatorParams(config, graph_localizer_simulator_params_);
    la::LoadGraphVIOSimulatorParams(config, graph_vio_simulator_params_);
    la::LoadOfflineReplayParams(config, offline_replay_params_);
  }

  la::LiveMeasurementSimulatorParams live_measurement_simulator_params_;
  la::GraphLocalizerSimulatorParams graph_localizer_simulator_params_;
  la::GraphVIOSimulatorParams graph_vio_simulator_params_;
  la::OfflineReplayParams offline_replay_params_;
};

TEST_F(OfflineReplayParameterReaderTest, LiveMeasurementSimulatorParams) {
  const auto& params = live_measurement_simulator_params_;
  EXPECT_NEAR(params.imu.msg_delay, 0, 1e-6);
  EXPECT_NEAR(params.imu.min_msg_spacing, 0, 1e-6);
  EXPECT_NEAR(params.flight_mode.msg_delay, 0, 1e-6);
  EXPECT_NEAR(params.flight_mode.min_msg_spacing, 0, 1e-6);
  EXPECT_NEAR(params.depth_odometry.msg_delay, 0, 1e-6);
  EXPECT_NEAR(params.depth_odometry.min_msg_spacing, 0, 1e-6);
  EXPECT_NEAR(params.of.msg_delay, 0, 1e-6);
  EXPECT_NEAR(params.of.min_msg_spacing, 0, 1e-6);
  EXPECT_NEAR(params.vl.msg_delay, 0, 1e-6);
  EXPECT_NEAR(params.vl.min_msg_spacing, 0, 1e-6);
  EXPECT_NEAR(params.ar.msg_delay, 0, 1e-6);
  EXPECT_NEAR(params.ar.min_msg_spacing, 0, 1e-6);
  EXPECT_NEAR(params.vio.msg_delay, 0, 1e-6);
  EXPECT_NEAR(params.vio.min_msg_spacing, 0, 1e-6);
  EXPECT_EQ(params.bag_name, "test.bag");
  EXPECT_EQ(params.map_file, "test.map");
  EXPECT_EQ(params.image_topic, "test_topic");
  EXPECT_EQ(params.use_bag_image_feature_msgs, true);
  EXPECT_EQ(params.save_optical_flow_images, false);
}

TEST_F(OfflineReplayParameterReaderTest, GraphLocalizerSimulatorParams) {
  const auto& params = graph_localizer_simulator_params_;
  EXPECT_NEAR(params.optimization_time, 0.05, 1e-6);
}

TEST_F(OfflineReplayParameterReaderTest, GraphVIOSimulatorParams) {
  const auto& params = graph_vio_simulator_params_;
  EXPECT_NEAR(params.optimization_time, 0.3, 1e-6);
}

TEST_F(OfflineReplayParameterReaderTest, OfflineReplayParams) {
  const auto& params = offline_replay_params_;
  EXPECT_EQ(params.save_optical_flow_images, false);
  EXPECT_EQ(params.log_relative_time, false);
  EXPECT_EQ(params.ar_min_num_landmarks, 5);
  EXPECT_EQ(params.sparse_mapping_min_num_landmarks, 5);
  // Taken using current nav cam extrinsics
  const gtsam::Pose3 expected_body_T_cam(gtsam::Rot3(0.500, 0.500, 0.500, 0.500),
                                         gtsam::Point3(0.1177, -0.0422, -0.0826));
  EXPECT_MATRIX_NEAR(params.body_T_nav_cam, expected_body_T_cam, 1e-6);
  // TODO(rsoussan): Check nav_cam_params
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
