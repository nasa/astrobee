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
#include <graph_factors/robust_smart_projection_pose_factor.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/matched_projections_measurement.h>
#include <localization_measurements/pose_measurement.h>
#include <node_adders/pose_node_adder.h>
#include <node_adders/pose_node_adder_model.h>
#include <node_adders/pose_node_adder_params.h>
#include <nodes/values.h>
#include <optimizers/optimizer.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer.h>

#include <gtest/gtest.h>

namespace fa = factor_adders;
namespace go = graph_optimizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
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

  int iterations() const final { return 0; }
};

// For testing new window start time and end time
class DummySlidingWindowNodeAdder : public na::SlidingWindowNodeAdder {
 public:
  bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                   const boost::optional<const gtsam::Marginals&>& marginals, const gtsam::KeyVector& old_keys,
                   const double huber_k, gtsam::NonlinearFactorGraph& factors) final {
    return true;
  }

  boost::optional<localization_common::Time> SlideWindowNewStartTime() const final { return new_start_time_; }

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                           const gtsam::NonlinearFactorGraph& graph) const final {
    return gtsam::KeyVector();
  }

  boost::optional<localization_common::Time> StartTime() const final { return start_time_; }

  boost::optional<localization_common::Time> EndTime() const final { return end_time_; }

  void AddInitialNodesAndPriors(gtsam::NonlinearFactorGraph& graph) final { return; }

  bool AddNode(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final { return true; }

  bool CanAddNode(const localization_common::Time timestamp) const final { return true; }

  gtsam::KeyVector Keys(const localization_common::Time timestamp) const final { return gtsam::KeyVector(); }

  lc::Time new_start_time_;
  lc::Time start_time_;
  lc::Time end_time_;
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
      new no::TimestampedNodes<gtsam::Pose3>(sliding_window_graph_optimizer_->values()));
    pose_node_adder_.reset(
      new na::PoseNodeAdder(DefaultPoseNodeAdderParams(), DefaultPoseNodeAdderModelParams(), timestamped_pose_nodes));
    // Add initial measurement to pose node adder
    const lm::PoseWithCovarianceMeasurement pose_measurement(gtsam::Pose3::identity(),
                                                             Eigen::Matrix<double, 6, 6>::Identity(), 0);
    pose_node_adder_->AddMeasurement(pose_measurement);

    dummy_node_adder_.reset(new DummySlidingWindowNodeAdder());
    loc_factor_adder_params_ = DefaultLocFactorAdderParams();
    loc_factor_adder_.reset(new fa::LocFactorAdder<na::PoseNodeAdder>(loc_factor_adder_params_, pose_node_adder_));
    sliding_window_graph_optimizer_->AddSlidingWindowNodeAdder(pose_node_adder_);
    sliding_window_graph_optimizer_->AddFactorAdder(loc_factor_adder_);
  }

  void AddLocMeasurement(const double time) {
    lm::MatchedProjectionsMeasurement loc_measurement;
    // Need at least one matched projection for measurement to be valid.
    lm::MatchedProjection matched_projection(gtsam::Point2(), gtsam::Point3(), time);
    loc_measurement.matched_projections.emplace_back(matched_projection);
    loc_measurement.global_T_cam = lc::RandomPose();
    loc_measurement.timestamp = time;
    loc_factor_adder_->AddMeasurement(loc_measurement);
    loc_measurements_.emplace_back(loc_measurement);
  }

  void AddPoseMeasurement(const double time) {
    const lm::PoseWithCovarianceMeasurement pose_measurement(lc::RandomPoseWithCovariance(), time);
    pose_node_adder_->AddMeasurement(pose_measurement);
  }

  sw::SlidingWindowGraphOptimizerParams DefaultSlidingWindowGraphOptimizerParams() {
    sw::SlidingWindowGraphOptimizerParams params;
    params.add_marginal_factors = false;
    params.slide_window_before_optimization = true;
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
    params.ideal_duration = 1;
    params.min_num_states = 1;
    params.max_num_states = 4;
    params.SetStartNoiseModels();
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

  // TODO(rsoussan): Merge this with loc factor adder test
  void EXPECT_SAME_LOC_POSE_FACTOR(const int factor_index, const int measurement_index) {
    const auto pose_factor =
      dynamic_cast<gtsam::LocPoseFactor*>(sliding_window_graph_optimizer_->factors()[factor_index].get());
    ASSERT_TRUE(pose_factor);
    const gtsam::Pose3 factor_pose = pose_factor->prior();
    const gtsam::Pose3 measurement_pose =
      loc_measurements_[measurement_index].global_T_cam * loc_factor_adder_params_.body_T_cam.inverse();
    EXPECT_MATRIX_NEAR(factor_pose, measurement_pose, 1e-6);
  }

  std::unique_ptr<sw::SlidingWindowGraphOptimizer> sliding_window_graph_optimizer_;
  std::shared_ptr<fa::LocFactorAdder<na::PoseNodeAdder>> loc_factor_adder_;
  fa::LocFactorAdderParams loc_factor_adder_params_;
  std::shared_ptr<na::PoseNodeAdder> pose_node_adder_;
  std::shared_ptr<DummySlidingWindowNodeAdder> dummy_node_adder_;
  std::vector<lm::MatchedProjectionsMeasurement> loc_measurements_;
};

TEST_F(SlidingWindowGraphOptimizerTest, SlideWindowDurationViolation) {
  // Initial node and prior should be added for pose node adder
  EXPECT_EQ(sliding_window_graph_optimizer_->num_factors(), 1);
  EXPECT_EQ(sliding_window_graph_optimizer_->num_values(), 1);
  // Add first measurements
  // Add loc measurement at pose node initial time so no new pose nodes are added.
  AddLocMeasurement(0);
  // Update graph
  EXPECT_TRUE(sliding_window_graph_optimizer_->Update());
  // Pose node times:
  // 0
  // Pose node num nodes: 1
  // Pose node duration: 0
  // Pose node limits: duration = 1, min_nodes = 1, max_nodes = 4
  // No violations, nothing should be removed.
  EXPECT_EQ(sliding_window_graph_optimizer_->num_values(), 1);
  // Expect 2 factors (prior and measurement on first node)
  EXPECT_EQ(sliding_window_graph_optimizer_->num_factors(), 2);
  EXPECT_SAME_LOC_POSE_FACTOR(1, 0);
  // Add second measurements
  AddLocMeasurement(1);
  AddPoseMeasurement(1);
  // Update graph
  EXPECT_TRUE(sliding_window_graph_optimizer_->Update());
  // Pose node times:
  // 0 1
  // Pose node num nodes: 2
  // Pose node duration: 1
  // Pose node limits: duration = 1, min_nodes = 1, max_nodes = 4
  // No violations, nothing should be removed.
  EXPECT_EQ(sliding_window_graph_optimizer_->num_values(), 2);
  // Expect 4 factors (prior and measurement on first node, between factor, measurement on second node)
  EXPECT_EQ(sliding_window_graph_optimizer_->num_factors(), 4);
  EXPECT_SAME_LOC_POSE_FACTOR(1, 0);
  EXPECT_SAME_LOC_POSE_FACTOR(3, 1);
  // Add third measurements
  AddLocMeasurement(2);
  AddPoseMeasurement(2);
  // Update graph
  EXPECT_TRUE(sliding_window_graph_optimizer_->Update());
  // Pose node times:
  // 0 1 2
  // Pose node num nodes: 3
  // Pose node duration: 2
  // Pose node limits: duration = 1, min_nodes = 1, max_nodes = 4
  // Duration too large, oldest node should be removed.
  // Pose slide window pose node times:
  // 1 2
  // Pose node num nodes: 2
  // Pose node duration: 1
  EXPECT_EQ(sliding_window_graph_optimizer_->num_values(), 2);
  // Expect 4 factors (prior and measurement on first node, between factor, measurement on second node)
  EXPECT_EQ(sliding_window_graph_optimizer_->num_factors(), 4);
  // Pose prior removed then added last
  EXPECT_SAME_LOC_POSE_FACTOR(0, 1);
  EXPECT_SAME_LOC_POSE_FACTOR(2, 2);
}

TEST_F(SlidingWindowGraphOptimizerTest, SlideWindowNumNodesViolation) {
  // Initial node and prior should be added for pose node adder
  EXPECT_EQ(sliding_window_graph_optimizer_->num_factors(), 1);
  EXPECT_EQ(sliding_window_graph_optimizer_->num_values(), 1);
  // Add 3 nodes to populate graph with 4 nodes (max)
  const double time_increment = 0.1;
  for (int i = 1; i <= 3; ++i) {
    AddLocMeasurement(time_increment * i);
    AddPoseMeasurement(time_increment * i);
  }
  EXPECT_TRUE(sliding_window_graph_optimizer_->Update());
  // Pose node times:
  // 0, 0.1, 0.2, 0.3
  // Pose node num nodes: 4
  // Pose node duration: 0.3
  // Pose node limits: duration = 1, min_nodes = 1, max_nodes = 4
  // No violations, nothing should be removed.
  EXPECT_EQ(sliding_window_graph_optimizer_->num_values(), 4);
  // Expect 7 factors (prior, three between factors, 3 measurements)
  EXPECT_EQ(sliding_window_graph_optimizer_->num_factors(), 7);
  // Violate num nodes limit
  // Add second measurements
  AddLocMeasurement(0.4);
  AddPoseMeasurement(0.4);
  EXPECT_TRUE(sliding_window_graph_optimizer_->Update());
  // Pose node times:
  // 0, 0.1, 0.2, 0.3, 0.4
  // Pose node num nodes: 5
  // Pose node duration: 0.4
  // Pose node limits: duration = 1, min_nodes = 1, max_nodes = 4
  // Too many nodes, first node should be removed.
  // Pose slide window pose node times:
  // 0.1, 0.2, 0.3, 0.4
  // Pose node num nodes: 4
  // Pose node duration: 0.3
  EXPECT_EQ(sliding_window_graph_optimizer_->num_values(), 4);
  // Expect 8 factors (prior, three between factors, 4 measurements)
  EXPECT_EQ(sliding_window_graph_optimizer_->num_factors(), 8);
}

TEST_F(SlidingWindowGraphOptimizerTest, TwoAddersNewStartTime) {
  sliding_window_graph_optimizer_->AddSlidingWindowNodeAdder(dummy_node_adder_);
  // Initial node and prior should be added for pose node adder
  EXPECT_EQ(sliding_window_graph_optimizer_->num_factors(), 1);
  EXPECT_EQ(sliding_window_graph_optimizer_->num_values(), 1);
  // Add 3 nodes to populate graph with 4 nodes (max)
  const double time_increment = 0.1;
  for (int i = 1; i <= 3; ++i) {
    AddLocMeasurement(time_increment * i);
    AddPoseMeasurement(time_increment * i);
  }
  // Set second node adder new start time to 0.2 so first two nodes should be removed
  dummy_node_adder_->new_start_time_ = 0.2;
  // Set other times to be negligable
  dummy_node_adder_->start_time_ = 0;
  dummy_node_adder_->end_time_ = 100;
  EXPECT_TRUE(sliding_window_graph_optimizer_->Update());
  // Pose node times:
  // 0.2, 0.3
  // Pose node num nodes: 2
  // Pose node duration: 0.1
  EXPECT_EQ(sliding_window_graph_optimizer_->num_values(), 2);
  // Expect 3 factors (prior, one between factor, 2 measurements)
  EXPECT_EQ(sliding_window_graph_optimizer_->num_factors(), 4);
}

TEST_F(SlidingWindowGraphOptimizerTest, SlideWindowPruneCumulativeFactor) {
  using SmartFactorCalibration = gtsam::Cal3_S2;
  using RobustSmartFactor = gtsam::RobustSmartProjectionPoseFactor<SmartFactorCalibration>;
  // Initial node and prior should be added for pose node adder
  EXPECT_EQ(sliding_window_graph_optimizer_->num_factors(), 1);
  EXPECT_EQ(sliding_window_graph_optimizer_->num_values(), 1);
  // Add 4 nodes to populate graph with 5 nodes (1 over max)
  const double time_increment = 0.1;
  for (int i = 1; i <= 4; ++i) {
    AddLocMeasurement(time_increment * i);
    AddPoseMeasurement(time_increment * i);
  }
  // Hackily add a cumulative factor (smart factor) using all pose nodes.
  {
    auto smart_factor = boost::make_shared<RobustSmartFactor>(gtsam::noiseModel::Isotropic::Sigma(2, 0.1),
                                                              boost::make_shared<gtsam::Cal3_S2>(), gtsam::Pose3(),
                                                              gtsam::SmartProjectionParams(), true, true, 1.345);
    for (int i = 1; i <= 5; ++i) {
      smart_factor->add(gtsam::Point2(i + 1, i + 2), gtsam::Key(i));
    }
    sliding_window_graph_optimizer_->factors().push_back(smart_factor);
  }
  EXPECT_TRUE(sliding_window_graph_optimizer_->Update());
  // Pose node times:
  // 0, 0.1, 0.2, 0.3, 0.4
  // Pose node num nodes: 5
  // Pose node duration: 0.4
  // Pose node limits: duration = 1, min_nodes = 1, max_nodes = 4
  // 1st node should be removed.
  EXPECT_EQ(sliding_window_graph_optimizer_->num_values(), 4);
  // Expect 9 factors (prior, three between factors, 4 measurements, 1 smart factor)
  EXPECT_EQ(sliding_window_graph_optimizer_->num_factors(), 9);
  // Smart factor should be in first index since added before updating graph.
  const auto smart_factor = dynamic_cast<const RobustSmartFactor*>(sliding_window_graph_optimizer_->factors()[0].get());
  ASSERT_TRUE(smart_factor);
  EXPECT_EQ(smart_factor->keys().size(), 4);
  for (int i = 0; i < 4; ++i) {
    // Keys are offset by 1, and 1st key is removed
    const int key = i + 2;
    EXPECT_EQ(smart_factor->keys()[i], key);
    // Point is (key+1, key+2)
    EXPECT_MATRIX_NEAR(smart_factor->measured()[i], gtsam::Point2(key + 1, key + 2), 1e-6);
  }
}

// TODO(rsoussan): Test adding marginal factors

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
