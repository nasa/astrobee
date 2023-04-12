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

#include <parameter_reader/graph_vio.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace gv = graph_vio;
namespace pr = parameter_reader;

class GraphVIOParameterReaderTest : public ::testing::Test {
 public:
  GraphVIOParameterReaderTest() {}

  void SetUp() final {
    config_reader::ConfigReader config;
    lc::LoadGraphVIOConfig(config, graph_config_path_prefix);
      config.AddFile("transforms.config");
        config.AddFile("cameras.config");
        config.AddFile("geometry.config");
        if (!config.ReadFiles()) {
          LogFatal("Failed to read config files.");
        }
  pr::LoadGraphVIOParams(config, params_);
}

  gv::GraphVIOParams params_;
};

TEST_F(GraphVIOParameterReaderTest, StandstillParams) {
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
