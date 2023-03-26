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

#include <factor_adders/loc_factor_adder.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <node_adders/pose_node_adder.h>
#include <node_adders/pose_node_adder_model.h>
#include <node_adders/pose_node_adder_params.h>
#include <nodes/nodes.h>
#include <optimizers/optimizer.h>

#include <gtest/gtest.h>

namespace fa = factor_adders;
namespace go = graph_optimizer;
namespace lc = localization_common;
namespace na = node_adders;
namespace no = nodes;
namespace op = optimizers;
namespace sw = sliding_window_graph_optimizer;

class SimpleOptimizer : public op::Optimizer {
 public:
  explicit SimpleOptimizer(const op::OptimizerParams& params) : op::Optimizer(params) {}

  bool Optimize(const gtsam::NonlinearFactorGraph& factors, gtsam::Values& values) final { return true; }

  boost::optional<gtsam::Matrix> Covariance(const gtsam::Key& key) const final {
    return gtsam::Matrix(gtsam::Matrix6::Identity());
  }
};

class SlidingWindowGraphOptimizerTest : public ::testing::Test {
 public:
  SlidingWindowGraphOptimizerTest() {}

  void SetUp() final { Initialize(); }

  void Initialize() {
    std::unique_ptr<SimpleOptimizer> optimizer(new SimpleOptimizer(DefaultOptimizerParams()));
    sliding_window_graph_optimizer_.reset(
      new sw::SlidingWindowGraphOptimizer(DefaultSlidingWindowGraphOptimizerParams(), std::move(optimizer)));
    std::shared_ptr<no::TimestampedNodes<gtsam::Pose3>> timestamped_pose_nodes(
      new no::TimestampedNodes<gtsam::Pose3>(sliding_window_graph_optimizer_->nodes()));
    pose_node_adder_.reset(
      new na::PoseNodeAdder(DefaultPoseNodeAdderParams(), DefaultPoseNodeAdderModelParams(), timestamped_pose_nodes));
    loc_factor_adder_.reset(new fa::LocFactorAdder<na::PoseNodeAdder>(DefaultLocFactorAdderParams(), pose_node_adder_));
    sliding_window_graph_optimizer_->AddSlidingWindowNodeAdder(pose_node_adder_);
    sliding_window_graph_optimizer_->AddFactorAdder(loc_factor_adder_);
  }

  sw::SlidingWindowGraphOptimizerParams DefaultSlidingWindowGraphOptimizerParams() {
    sw::SlidingWindowGraphOptimizerParams params;
    params.add_marginal_factors = false;
    params.huber_k = 1.345;
    params.log_stats_on_destruction = false;
    params.print_after_optimization = false;
    return params;
  }

  na::TimestampedNodeAdderModelParams DefaultPoseNodeAdderModelParams() {
    na::TimestampedNodeAdderModelParams params;
    params.huber_k = 1.345;
    return params;
  }

  // TODO(rsoussan): Get this from node adders package
  na::PoseNodeAdderParams DefaultPoseNodeAdderParams() {
    na::PoseNodeAdderParams params;
    params.starting_prior_translation_stddev = 0.1;
    params.starting_prior_quaternion_stddev = 0.2;
    params.start_node = gtsam::Pose3::identity();
    params.huber_k = 1.345;
    params.add_priors = true;
    params.starting_time = 0;
    // Only kept if there are at least min_num_states and not more than max_num_states
    params.ideal_duration = 5;
    params.min_num_states = 5;
    params.max_num_states = 20;
    params.Initialize();
    return params;
  }

  fa::LocFactorAdderParams DefaultLocFactorAdderParams() {
    fa::LocFactorAdderParams params;
    params.add_pose_priors = true;
    params.add_projection_factors = false;
    params.prior_translation_stddev = 0.1;
    params.prior_quaternion_stddev = 0.2;
    params.scale_pose_noise_with_num_landmarks = false;
    params.pose_noise_scale = 1;
    params.min_num_matches_per_measurement = 0;
    params.body_T_cam = gtsam::Pose3::identity();
    params.enabled = true;
    params.huber_k = 1.345;
    return params;
  }

  op::OptimizerParams DefaultOptimizerParams() {
    op::OptimizerParams params;
    params.marginals_factorization = "qr";
    return params;
  }

  std::unique_ptr<sw::SlidingWindowGraphOptimizer> sliding_window_graph_optimizer_;
  std::shared_ptr<fa::LocFactorAdder<na::PoseNodeAdder>> loc_factor_adder_;
  std::shared_ptr<na::PoseNodeAdder> pose_node_adder_;
};

TEST_F(SlidingWindowGraphOptimizerTest, AddFactors) {
  /*  EXPECT_EQ(graph_optimizer_->num_factors(), 0);
    // Add first factors
    EXPECT_EQ(graph_optimizer_->AddFactors(0, 1), 1);
    EXPECT_EQ(graph_optimizer_->factors().size(), 1);
    EXPECT_EQ(graph_optimizer_->num_factors(), 1);
    EXPECT_EQ(graph_optimizer_->num_nodes(), 1);
    EXPECT_TRUE(graph_optimizer_->Optimize());
    // Add second factors
    EXPECT_EQ(graph_optimizer_->AddFactors(1, 2), 1);
    EXPECT_EQ(graph_optimizer_->factors().size(), 2);
    EXPECT_EQ(graph_optimizer_->num_factors(), 2);
    EXPECT_EQ(graph_optimizer_->num_nodes(), 2);
    EXPECT_TRUE(graph_optimizer_->Optimize());*/
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
