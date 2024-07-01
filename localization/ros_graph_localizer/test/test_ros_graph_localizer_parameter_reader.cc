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
#include <ros_graph_localizer/parameter_reader.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace rl = ros_graph_localizer;

class RosGraphLocalizerParameterReaderTest : public ::testing::Test {
 public:
  RosGraphLocalizerParameterReaderTest() {}

  void SetUp() final {
    lc::SetEnvironmentConfigs();
    config_reader::ConfigReader config;
    config.AddFile("localization/ros_graph_localizer.config");
    if (!config.ReadFiles()) {
      LogFatal("Failed to read config files.");
    }
    rl::LoadRosGraphLocalizerNodeletParams(config, params_);
  }

  rl::RosGraphLocalizerNodeletParams params_;
};

TEST_F(RosGraphLocalizerParameterReaderTest, RosGraphLocalizerNodeletParams) {
  EXPECT_EQ(params_.max_graph_vio_state_buffer_size, 10);
  EXPECT_EQ(params_.max_vl_matched_projections_buffer_size, 10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
