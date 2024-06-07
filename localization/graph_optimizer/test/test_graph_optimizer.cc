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
#include <nodes/values.h>
#include <optimizers/optimizer.h>

#include <gtest/gtest.h>

namespace fa = factor_adders;
namespace go = graph_optimizer;
namespace lc = localization_common;
namespace na = node_adders;
namespace no = nodes;
namespace op = optimizers;

// Test node adder that just returns keys that should be used.
// Key values are calculated using the integer timestamps passed.
// TODO(rsoussan): Unify this with VO factor_adder test
class SimpleNodeAdder : public na::NodeAdder {
 public:
  explicit SimpleNodeAdder(std::shared_ptr<no::Values> values) : values_(values) {}

  // Add prior factor and node
  void AddInitialNodesAndPriors(gtsam::NonlinearFactorGraph& factors) final {
    AddNode(0, factors);
    const auto keys = Keys(0);
    const gtsam::Vector6 noise_sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.2, 0.2, 0.2).finished());
    const auto noise =
      lc::Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(noise_sigmas)), huber_k);
    gtsam::PriorFactor<gtsam::Pose3>::shared_ptr prior_factor(
      new gtsam::PriorFactor<gtsam::Pose3>(keys[0], gtsam::Pose3::identity(), noise));
    factors.push_back(prior_factor);
  }

  bool AddNode(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final {
    values_->Add(gtsam::Pose3::identity());
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

 private:
  std::shared_ptr<no::Values> values_;
  const double huber_k = 1.345;
};

class SimpleFactorAdder : public fa::FactorAdder {
 public:
  SimpleFactorAdder(const fa::FactorAdderParams& params, std::shared_ptr<SimpleNodeAdder> node_adder)
      : fa::FactorAdder(params), node_adder_(node_adder) {}
  int AddFactors(const localization_common::Time oldest_allowed_time,
                 const localization_common::Time newest_allowed_time, gtsam::NonlinearFactorGraph& factors) final {
    node_adder_->AddNode(oldest_allowed_time, factors);
    const auto keys = node_adder_->Keys(oldest_allowed_time);
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

class SimpleOptimizer : public op::Optimizer {
 public:
  explicit SimpleOptimizer(const op::OptimizerParams& params) : op::Optimizer(params) {}

  bool Optimize(const gtsam::NonlinearFactorGraph& factors, gtsam::Values& values) final { return true; }

  boost::optional<gtsam::Matrix> Covariance(const gtsam::Key& key) const final {
    return gtsam::Matrix(gtsam::Matrix6::Identity());
  }

  int iterations() const final { return 0; }
};

class GraphOptimizerTest : public ::testing::Test {
 public:
  GraphOptimizerTest() {}

  void SetUp() final {}

  void Initialize(const go::GraphOptimizerParams& params) {
    std::unique_ptr<SimpleOptimizer> optimizer(new SimpleOptimizer(DefaultOptimizerParams()));
    graph_optimizer_.reset(new go::GraphOptimizer(params, std::move(optimizer)));
    node_adder_.reset(new SimpleNodeAdder(graph_optimizer_->values()));
    factor_adder_.reset(new SimpleFactorAdder(DefaultFactorAdderParams(), node_adder_));
    graph_optimizer_->AddNodeAdder(node_adder_);
    graph_optimizer_->AddFactorAdder(factor_adder_);
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

  op::OptimizerParams DefaultOptimizerParams() {
    op::OptimizerParams params;
    params.marginals_factorization = "qr";
    return params;
  }

  std::unique_ptr<go::GraphOptimizer> graph_optimizer_;
  std::shared_ptr<SimpleFactorAdder> factor_adder_;
  std::shared_ptr<SimpleNodeAdder> node_adder_;
};

TEST_F(GraphOptimizerTest, AddFactors) {
  auto params = DefaultParams();
  Initialize(params);
  // Node and factor should be added for initial node adder node and prior
  EXPECT_EQ(graph_optimizer_->num_factors(), 1);
  EXPECT_EQ(graph_optimizer_->num_values(), 1);
  // Add first factors
  EXPECT_EQ(graph_optimizer_->AddFactors(0, 1), 1);
  EXPECT_EQ(graph_optimizer_->factors().size(), 2);
  EXPECT_EQ(graph_optimizer_->num_factors(), 2);
  EXPECT_EQ(graph_optimizer_->num_values(), 2);
  EXPECT_TRUE(graph_optimizer_->Optimize());
  // Add second factors
  EXPECT_EQ(graph_optimizer_->AddFactors(1, 2), 1);
  EXPECT_EQ(graph_optimizer_->factors().size(), 3);
  EXPECT_EQ(graph_optimizer_->num_factors(), 3);
  EXPECT_EQ(graph_optimizer_->num_values(), 3);
  EXPECT_TRUE(graph_optimizer_->Optimize());
}

TEST_F(GraphOptimizerTest, Covariance) {
  auto params = DefaultParams();
  Initialize(params);
  EXPECT_EQ(graph_optimizer_->num_factors(), 1);
  // Add first factors
  EXPECT_EQ(graph_optimizer_->AddFactors(0, 1), 1);
  EXPECT_EQ(graph_optimizer_->factors().size(), 2);
  EXPECT_EQ(graph_optimizer_->num_factors(), 2);
  EXPECT_EQ(graph_optimizer_->num_values(), 2);
  EXPECT_TRUE(graph_optimizer_->Optimize());
  const auto keys = node_adder_->Keys(0);
  const auto covariance = graph_optimizer_->Covariance(keys[0]);
  EXPECT_TRUE(covariance != boost::none);
  EXPECT_MATRIX_NEAR(gtsam::Matrix(*covariance), gtsam::Matrix6::Identity(), 1e-6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
