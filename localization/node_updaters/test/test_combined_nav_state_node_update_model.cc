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

#include "test_utilities.h"  // NOLINT
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <node_updaters/combined_nav_state_node_update_model.h>
#include <node_updaters/utilities.h>

#include <gtest/gtest.h>

namespace go = graph_optimizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace nu = node_updaters;

class CombinedNavStateNodeUpdateModelTest : public ::testing::Test {
 public:
  CombinedNavStateNodeUpdateModelTest()
      : combined_nav_state_node_update_model_(nu::DefaultCombinedNavStateNodeUpdateModelParams()) {}
  void SetUp() final {}

  nu::CombinedNavStateNodeUpdateModel combined_nav_state_node_update_model_;
  gtsam::NonlinearFactorGraph factors_;
};

TEST_F(CombinedNavStateNodeUpdateModelTest, AddRemoveCanAddNode) {
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
