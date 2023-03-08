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
#include <node_updaters/pose_node_updater.h>

#include <gtest/gtest.h>

namespace go = graph_optimizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace nu = node_updaters;

class PoseNodeUpdaterTest : public ::testing::Test {
 public:
  PoseNodeUpdaterTest() : time_increment_(1.0), start_time_(1.0), num_measurements_(20) {
    params_ = nu::DefaultPoseNodeUpdaterParams();
  }

  void SetUp() final {
    for (int i = 0; i < num_measurements_; ++i) {
      const lc::Time time = start_time_ + time_increment_*i;
      const lc::PoseWithCovariance pose_with_covariance(lc::RandomPoseWithCovariance());
      const lm::TimestampedPoseWithCovariance pose_measurement(pose_with_covariance, time);
      pose_measurements_.emplace_back(pose_measurement);
      timestamps_.emplace_back(time);
    }
  }

  void AddMeasurements() {
    for (const auto& measurement : pose_measurements_) {
      pose_node_updater_->AddMeasurement(measurement);
    }
  }

  void RandomInitialize() {
    params_.start_node = lc::RandomPose();
    params_.starting_time = lc::RandomDouble();
    params_.Initialize();
    pose_node_updater_.reset(new nu::PoseNodeUpdater(params_));
    pose_node_updater_->AddInitialNodesAndPriors(factors_);
  }

  void DefaultInitialize() {
    pose_node_updater_.reset(new nu::PoseNodeUpdater(params_));
    pose_node_updater_->AddInitialNodesAndPriors(factors_);
  }

  void ZeroInitialize() {
    params_.start_node = gtsam::Pose3::identity();
    params_.starting_time = 0.0;
    params_.Initialize();
    pose_node_updater_.reset(new nu::PoseNodeUpdater(params_));
    pose_node_updater_->AddInitialNodesAndPriors(factors_);
  }

  std::unique_ptr<nu::PoseNodeUpdater> pose_node_updater_;
  nu::PoseNodeUpdaterParams params_;
  std::vector<lm::TimestampedPoseWithCovariance> pose_measurements_;
  std::vector<lc::Time> timestamps_;
  gtsam::NonlinearFactorGraph factors_;

 private:
  const double time_increment_;
  const lc::Time start_time_;
  const int num_measurements_;
};

TEST_F(PoseNodeUpdaterTest, AddRemoveCanAddNode) {
  DefaultInitialize();
  EXPECT_FALSE(pose_node_updater_->CanAddNode(10.1));
  constexpr double epsilon = 0.1;
  // Check initialized measurement
  EXPECT_TRUE(pose_node_updater_->CanAddNode(params_.starting_time));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(params_.starting_time + epsilon));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(params_.starting_time - epsilon));
  // Add measurement 0
  pose_node_updater_->AddMeasurement(pose_measurements_[0]);
  EXPECT_TRUE(pose_node_updater_->CanAddNode(timestamps_[0]));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(timestamps_[0] + epsilon));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(params_.starting_time - epsilon));
  // Add measurement 1
  pose_node_updater_->AddMeasurement(pose_measurements_[1]);
  EXPECT_TRUE(pose_node_updater_->CanAddNode(timestamps_[1]));
  EXPECT_TRUE(pose_node_updater_->CanAddNode((timestamps_[0] + timestamps_[1])/2.0));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(params_.starting_time - epsilon));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(timestamps_[1] + epsilon));
  // Add measurement 2
  pose_node_updater_->AddMeasurement(pose_measurements_[2]);
  EXPECT_TRUE(pose_node_updater_->CanAddNode(timestamps_[2]));
  EXPECT_TRUE(pose_node_updater_->CanAddNode(timestamps_[1] + epsilon));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(params_.starting_time - epsilon));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(timestamps_[2] + epsilon));
  // Add measurement 3
  pose_node_updater_->AddMeasurement(pose_measurements_[3]);
  EXPECT_TRUE(pose_node_updater_->CanAddNode(timestamps_[3]));
  EXPECT_TRUE(pose_node_updater_->CanAddNode(timestamps_[1] + epsilon));
  EXPECT_TRUE(pose_node_updater_->CanAddNode(timestamps_[2] + epsilon));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(params_.starting_time - epsilon));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(timestamps_[3] + epsilon));

  // Remove measurements start, 0, and 1
  pose_node_updater_->RemoveMeasurements(timestamps_[1] + epsilon);
  EXPECT_FALSE(pose_node_updater_->CanAddNode(params_.starting_time));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(timestamps_[0]));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(timestamps_[1]));
  EXPECT_TRUE(pose_node_updater_->CanAddNode(timestamps_[2]));
  EXPECT_TRUE(pose_node_updater_->CanAddNode(timestamps_[2] + epsilon));
  EXPECT_TRUE(pose_node_updater_->CanAddNode(timestamps_[3]));
  EXPECT_FALSE(pose_node_updater_->CanAddNode(timestamps_[3] + epsilon));
}


TEST_F(PoseNodeUpdaterTest, AddInitialNodesAndPriorsUsingParams) {
  RandomInitialize();
  const auto& nodes = pose_node_updater_->nodes();
  // Check node value
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_TRUE(nodes.Contains(params_.starting_time));
  const auto node = nodes.Node(params_.starting_time);
  ASSERT_TRUE(node != boost::none);
  EXPECT_MATRIX_NEAR(node->matrix(), params_.start_node, 1e-6);
  // Check factor
  EXPECT_EQ(factors_.size(), 1);
  const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factors_[0].get());
  ASSERT_TRUE(pose_prior_factor);
  EXPECT_MATRIX_NEAR(pose_prior_factor->prior(), params_.start_node, 1e-6);
  // Check noise
  const auto robust_noise_model = dynamic_cast<gtsam::noiseModel::Robust*>(pose_prior_factor->noiseModel().get());
  ASSERT_TRUE(robust_noise_model);
  const auto noise_model = dynamic_cast<gtsam::noiseModel::Gaussian*>(robust_noise_model->noise().get());
  ASSERT_TRUE(noise_model);

  const auto expected_robust_noise_model =
    dynamic_cast<gtsam::noiseModel::Robust*>(params_.start_noise_models[0].get());
  ASSERT_TRUE(expected_robust_noise_model);
  const auto expected_noise_model =
    dynamic_cast<gtsam::noiseModel::Gaussian*>(expected_robust_noise_model->noise().get());
  ASSERT_TRUE(expected_noise_model);
  EXPECT_MATRIX_NEAR(noise_model->covariance(), expected_noise_model->covariance(), 1e-6);
}

TEST_F(PoseNodeUpdaterTest, AddInitialNodesAndPriors) {
  const auto pose = lc::RandomPose();
  const auto time = lc::RandomDouble();
  pose_node_updater_.reset(new nu::PoseNodeUpdater(params_));
  pose_node_updater_->AddInitialNodesAndPriors(pose, params_.start_noise_models, time, factors_);
  const auto& nodes = pose_node_updater_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_TRUE(nodes.Contains(time));
  const auto node = nodes.Node(time);
  ASSERT_TRUE(node != boost::none);
  EXPECT_MATRIX_NEAR(node->matrix(), pose, 1e-6);
  // Check factor
  EXPECT_EQ(factors_.size(), 1);
  const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factors_[0].get());
  ASSERT_TRUE(pose_prior_factor);
  EXPECT_MATRIX_NEAR(pose_prior_factor->prior(), pose, 1e-6);
  // Check noise
  const auto robust_noise_model = dynamic_cast<gtsam::noiseModel::Robust*>(pose_prior_factor->noiseModel().get());
  ASSERT_TRUE(robust_noise_model);
  const auto noise_model = dynamic_cast<gtsam::noiseModel::Gaussian*>(robust_noise_model->noise().get());
  ASSERT_TRUE(noise_model);

  const auto expected_robust_noise_model =
    dynamic_cast<gtsam::noiseModel::Robust*>(params_.start_noise_models[0].get());
  ASSERT_TRUE(expected_robust_noise_model);
  const auto expected_noise_model =
    dynamic_cast<gtsam::noiseModel::Gaussian*>(expected_robust_noise_model->noise().get());
  ASSERT_TRUE(expected_noise_model);
  EXPECT_MATRIX_NEAR(noise_model->covariance(), expected_noise_model->covariance(), 1e-6);
}

TEST_F(PoseNodeUpdaterTest, AddNode) {
  ZeroInitialize();
  const auto& nodes = pose_node_updater_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  AddMeasurements();
  // TODO(rsoussan): add a bunch of measurements, add nodes, make sure correct between factors are added
  // Test adding nodes
  // ASSERT_TRUE(pose_node_updater_->AddNode(timestamps_[0], factors_));
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
