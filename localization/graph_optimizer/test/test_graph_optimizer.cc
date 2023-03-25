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
#include <nodes/nodes.h>

#include <gtest/gtest.h>

namespace fa = factor_adders;
namespace go = graph_optimizer;
namespace lc = localization_common;
namespace na = node_adders;
namespace no = nodes;

// Test node adder that just returns keys that should be used.
// Key values are calculated using the integer timestamps passed.
// TODO(rsoussan): Unify this with VO factor_adder test
class SimpleNodeAdder : public na::NodeAdder {
 public:
  explicit SimpleNodeAdder(std::shared_ptr<no::Nodes> nodes) : nodes_(nodes) {}

  void AddInitialNodesAndPriors(gtsam::NonlinearFactorGraph& graph) final{};

  bool AddNode(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final {
    nodes_->Add(gtsam::Pose3::identity());
    return true;
  }

  bool CanAddNode(const localization_common::Time timestamp) const final { return true; }

  // Assumes integer timestamps that perfectly cast to ints.
  // First key is pose key.
  gtsam::KeyVector Keys(const localization_common::Time timestamp) const final {
    gtsam::KeyVector keys;
    // Offset by 1 since node keys start at 1
    keys.emplace_back(gtsam::Key(static_cast<int>(timestamp + 1)));
    return keys;
  }

  std::string type() const final { return "simple_pose_node_adder"; }

 private:
  std::shared_ptr<no::Nodes> nodes_;
};

class SimpleFactorAdder : public fa::FactorAdder {
 public:
  SimpleFactorAdder(const fa::FactorAdderParams& params, std::shared_ptr<SimpleNodeAdder> node_adder)
      : fa::FactorAdder(params), node_adder_(node_adder) {}
  int AddFactors(const localization_common::Time oldest_allowed_time,
                 const localization_common::Time newest_allowed_time, gtsam::NonlinearFactorGraph& factors) final {
    node_adder_->AddNode(newest_allowed_time, factors);
    const auto keys = node_adder_->Keys(newest_allowed_time);
    const gtsam::Vector6 noise_sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.2, 0.2, 0.2).finished());
    const auto noise =
      lc::Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(noise_sigmas)), params_.huber_k);
    gtsam::PriorFactor<gtsam::Pose3>::shared_ptr prior_factor(
      new gtsam::PriorFactor<gtsam::Pose3>(keys[0], gtsam::Pose3::identity(), noise));
    factors.push_back(prior_factor);
    return 1;
  }

  std::shared_ptr<SimpleNodeAdder> node_adder_;
};

class GraphOptimizerTest : public ::testing::Test {
 public:
  GraphOptimizerTest() {}

  void SetUp() final {}

  void Initialize(const go::GraphOptimizerParams& params) {
    // graph_optimizer_.reset(new go::GraphOptimizer(params));
    // TODO(rsoussan): remove this! get from graph optimizer!
    std::shared_ptr<no::Nodes> nodes(new no::Nodes());
    node_adder_.reset(new SimpleNodeAdder(nodes));
    factor_adder_.reset(new SimpleFactorAdder(DefaultFactorAdderParams(), node_adder_));
    // graph_optimizer_->AddNodeAdder(node_adder_);
    // graph_optimizer_->AddFactorAdder(factor_adder_);
  }

  go::GraphOptimizerParams DefaultParams() {
    go::GraphOptimizerParams params;
    params.huber_k = 1.345;
    params.log_stats_on_destruction = false;
    params.print_after_optimization = false;
    return params;
  }

  fa::FactorAdderParams DefaultFactorAdderParams() {
    fa::FactorAdderParams params;
    params.enabled = true;
    params.huber_k = 1.345;
    return params;
  }

  std::unique_ptr<go::GraphOptimizer> graph_optimizer_;
  std::shared_ptr<SimpleFactorAdder> factor_adder_;
  std::shared_ptr<SimpleNodeAdder> node_adder_;
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
