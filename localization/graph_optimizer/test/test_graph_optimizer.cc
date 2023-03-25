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

#include <factor_adders/factor_adder.h>
#include <graph_optimizer/graph_optimizer.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <node_adders/node_adder.h>

#include <gtest/gtest.h>

namespace fa = factor_adders;
namespace lc = localization_common;
namespace na = node_adders;

// Test node adder that just returns keys that should be used.
// Key values are calculated using the integer timestamps passed, where the
// first key is a pose key and second is a velocity key.
// Pose keys are 2*timestamp and velocity keys are 2*timestamp + 1.
/*class SimplePoseVelocityNodeAdder : public na::NodeAdder {
 public:
  void AddInitialNodesAndPriors(gtsam::NonlinearFactorGraph& graph) final{};

  bool AddNode(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final { return true; }

  bool CanAddNode(const localization_common::Time timestamp) const final { return true; }

  // Assumes integer timestamps that perfectly cast to ints.
  // First key is pose key, second is velocity key
  gtsam::KeyVector Keys(const localization_common::Time timestamp) const final {
    gtsam::KeyVector keys;
    keys.emplace_back(gtsam::Key(static_cast<int>(timestamp) * 2));
    keys.emplace_back(gtsam::Key(static_cast<int>(timestamp) * 2 + 1));
    return keys;
  }

  std::string type() const final { return "simple_pose_velocity_node_adder"; }
};*/

class GraphOptimizerTest : public ::testing::Test {
 public:
  GraphOptimizerTest() { node_adder_.reset(new SimplePoseVelocityNodeAdder()); }

  void SetUp() final {}

  void Initialize(const fa::GraphOptimizerParams& params) {
    factor_adder_.reset(new fa::GraphOptimizer<SimplePoseVelocityNodeAdder>(params, node_adder_));
  }

  fa::GraphOptimizerParams DefaultParams() {
    fa::GraphOptimizerParams params;
    params.enabled = true;
    params.huber_k = 1.345;
    params.add_velocity_prior = true;
    params.add_pose_between_factor = true;
    params.prior_velocity_stddev = 0.1;
    params.pose_between_factor_translation_stddev = 0.2;
    params.pose_between_factor_rotation_stddev = 0.3;
    return params;
  }

  std::unique_ptr<fa::GraphOptimizer<SimplePoseVelocityNodeAdder>> factor_adder_;
  std::shared_ptr<SimplePoseVelocityNodeAdder> node_adder_;
  gtsam::NonlinearFactorGraph factors_;
};

TEST_F(GraphOptimizerTest, PoseAndVelocityFactors) {
  auto params = DefaultParams();
  Initialize(params);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
