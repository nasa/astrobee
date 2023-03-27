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
#include <node_adders/pose_node_adder.h>
#include <node_adders/utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace lm = localization_measurements;
namespace na = node_adders;
namespace no = nodes;

class PoseNodeAdderTest : public ::testing::Test {
 public:
  PoseNodeAdderTest() : time_increment_(1.0), start_time_(1.0), num_measurements_(20) {
    params_ = na::DefaultPoseNodeAdderParams();
    node_adder_model_params_.huber_k = 1.345;
  }

  void SetUp() final {
    const Eigen::Vector3d translation_increment(1.0, 2.0, 3.0);
    Eigen::Vector3d translation(Eigen::Vector3d::Zero());
    for (int i = 0; i < num_measurements_; ++i) {
      // Increase increment over time so each relative pose is different
      translation += translation_increment * i;
      const lc::Time time = start_time_ + time_increment_ * i;
      const auto pose = lc::Isometry3d(translation, Eigen::Matrix3d::Identity());
      const lc::PoseWithCovariance pose_with_covariance(pose, lc::RandomPoseCovariance());
      const lm::TimestampedPoseWithCovariance pose_measurement(pose_with_covariance, time);
      pose_measurements_.emplace_back(pose_measurement);
      timestamps_.emplace_back(time);
    }
  }

  void AddMeasurements() {
    for (const auto& measurement : pose_measurements_) {
      pose_node_adder_->AddMeasurement(measurement);
    }
  }

  void RandomInitialize() {
    params_.start_node = lc::RandomPose();
    params_.starting_time = lc::RandomDouble();
    params_.Initialize();
    pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Nodes>()));
    pose_node_adder_->AddInitialNodesAndPriors(factors_);
  }

  void DefaultInitialize() {
    pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Nodes>()));
    pose_node_adder_->AddInitialNodesAndPriors(factors_);
  }

  void ZeroInitialize() {
    params_.start_node = gtsam::Pose3::identity();
    params_.starting_time = 0.0;
    params_.Initialize();
    pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Nodes>()));
    pose_node_adder_->AddInitialNodesAndPriors(factors_);
  }

  Eigen::Isometry3d pose(const int index) {
    if (index == -1) return lc::EigenPose(params_.start_node);
    return pose_measurements_[index].pose_with_covariance.pose;
  }

  lc::PoseCovariance covariance(const int index) {
    if (index == -1) return na::Covariance(params_.start_noise_models[0]);
    return pose_measurements_[index].pose_with_covariance.covariance;
  }

  lc::Time timestamp(const int index) {
    if (index == -1) return params_.starting_time;
    return timestamps_[index];
  }

  void EXPECT_SAME_NODE(const lc::Time timestamp, const Eigen::Isometry3d& pose) {
    const auto& nodes = pose_node_adder_->nodes();
    EXPECT_TRUE(nodes.Contains(timestamp));
    const auto node = nodes.Node(timestamp);
    ASSERT_TRUE(node != boost::none);
    EXPECT_MATRIX_NEAR(node->matrix(), pose, 1e-6);
  }

  void EXPECT_SAME_NODE(const lc::Time timestamp, const gtsam::Pose3& pose) {
    EXPECT_SAME_NODE(timestamp, lc::EigenPose(pose));
  }

  void EXPECT_SAME_NODE(const int index) { EXPECT_SAME_NODE(timestamp(index), pose(index)); }

  void EXPECT_SAME_NODE_INTERPOLATED(const int index_a, const int index_b, const double alpha) {
    const auto expected_pose = InterpolatedPose(index_a, index_b, alpha);
    const lc::Time timestamp_a_b = timestamp(index_a) * alpha + timestamp(index_b) * (1.0 - alpha);
    EXPECT_SAME_NODE(timestamp_a_b, expected_pose);
  }

  void EXPECT_SAME_PRIOR_FACTOR(const int index, const Eigen::Isometry3d& pose) {
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factors_[index].get());
    ASSERT_TRUE(pose_prior_factor);
    EXPECT_MATRIX_NEAR(pose_prior_factor->prior(), pose, 1e-6);
  }

  void EXPECT_SAME_PRIOR_FACTOR(const int index, const gtsam::Pose3& pose) {
    return EXPECT_SAME_PRIOR_FACTOR(index, lc::EigenPose(pose));
  }

  void EXPECT_SAME_BETWEEN_FACTOR(const int factor_index, const int pose_index) {
    const auto& pose_a = pose(pose_index - 1);
    const auto& pose_b = pose(pose_index);
    const auto relative_pose = pose_a.inverse() * pose_b;
    // Subtract 1 here to avoid assumption that factor 0 is a prior factor,
    // since this call eventually adds 1 and makes this assumption
    EXPECT_SAME_BETWEEN_FACTOR(factor_index - 1, relative_pose);
  }

  void EXPECT_SAME_BETWEEN_FACTOR(const int index) {
    const auto& pose_a = pose(index - 1);
    const auto& pose_b = pose(index);
    const auto relative_pose = pose_a.inverse() * pose_b;
    EXPECT_SAME_BETWEEN_FACTOR(index, relative_pose);
  }

  void EXPECT_SAME_BETWEEN_FACTOR(const int index, const Eigen::Isometry3d& pose) {
    // Add one to index to account for first prior pose
    const auto pose_between_factor = dynamic_cast<gtsam::BetweenFactor<gtsam::Pose3>*>(factors_[index + 1].get());
    ASSERT_TRUE(pose_between_factor);
    EXPECT_MATRIX_NEAR(pose_between_factor->measured(), pose, 1e-6);
  }

  void EXPECT_SAME_BETWEEN_FACTOR(const int index, const gtsam::Pose3& pose) {
    EXPECT_SAME_BETWEEN_FACTOR(index, lc::EigenPose(pose));
  }

  Eigen::Isometry3d InterpolatedPose(const int index_a, const int index_b, const double alpha) {
    return lc::Interpolate(pose(index_a), pose(index_b), alpha);
  }

  void EXPECT_SAME_BETWEEN_FACTOR_INTERPOLATED(const int index_a, const int index_b, const double alpha) {
    const auto pose_interpolated = InterpolatedPose(index_a, index_b, alpha);
    const Eigen::Isometry3d relative_pose = pose(index_a).inverse() * pose_interpolated;
    EXPECT_SAME_BETWEEN_FACTOR(index_b, relative_pose);
  }

  // Check the second between factor after an interpolated node
  void EXPECT_SAME_SECOND_BETWEEN_FACTOR_INTERPOLATED(const int index_a, const int index_b, const double alpha) {
    const auto pose_interpolated = InterpolatedPose(index_a, index_b, alpha);
    const Eigen::Isometry3d relative_pose = pose_interpolated.inverse() * pose(index_b);
    EXPECT_SAME_BETWEEN_FACTOR(index_b + 1, relative_pose);
  }

  template <typename FactorPtrType>
  void EXPECT_SAME_NOISE(const FactorPtrType factor, const lc::PoseCovariance& covariance) {
    EXPECT_MATRIX_NEAR(na::Covariance(factor->noiseModel()), covariance, 1e-6);
  }

  template <typename FactorPtrType>
  void EXPECT_SAME_NOISE(const FactorPtrType factor, const gtsam::SharedNoiseModel noise) {
    EXPECT_SAME_NOISE(factor, na::Covariance(noise));
  }

  void EXPECT_SAME_BETWEEN_NOISE(const int index, const lc::PoseCovariance& covariance) {
    // Handle special case where need prior noise (remove this once covariance computation is fixed)
    if (index == -1) {
      EXPECT_SAME_NOISE(dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factors_[0].get()), covariance);
    } else {
      // Add one to index to account for first prior pose
      const auto pose_between_factor = dynamic_cast<gtsam::BetweenFactor<gtsam::Pose3>*>(factors_[index + 1].get());
      EXPECT_SAME_NOISE(pose_between_factor, covariance);
    }
  }

  void EXPECT_SAME_BETWEEN_NOISE(const int index) {
    // TODO(rsoussan): Change this when pose interpolation covariance is adderd, use both covariances to compute
    // relative covariance
    EXPECT_SAME_BETWEEN_NOISE(index, covariance(index));
  }

  void EXPECT_SAME_BETWEEN_NOISE_INTERPOLATED(const int index_a, const int index_b, const double alpha) {
    // TODO(rsoussan): Change this when interpolation/relative computation for cov is adderd
    EXPECT_SAME_BETWEEN_NOISE(index_a);
  }

  void EXPECT_SAME_SECOND_BETWEEN_NOISE_INTERPOLATED(const int index_a, const int index_b, const double alpha) {
    // TODO(rsoussan): Change this when interpolation/relative computation for cov is adderd
    EXPECT_SAME_BETWEEN_NOISE(index_b + 1, covariance(index_b));
  }

  void EXPECT_SAME_BETWEEN_FACTOR_AND_NOISE(const int index) {
    EXPECT_SAME_BETWEEN_FACTOR(index);
    EXPECT_SAME_BETWEEN_NOISE(index);
  }

  void EXPECT_SAME_BETWEEN_FACTOR_AND_NOISE_INTERPOLATED(const int index_a, const int index_b, const double alpha) {
    EXPECT_SAME_BETWEEN_FACTOR_INTERPOLATED(index_a, index_b, alpha);
    EXPECT_SAME_BETWEEN_NOISE_INTERPOLATED(index_a, index_b, alpha);
  }

  void EXPECT_SAME_PRIOR_NOISE(const int index, const gtsam::SharedNoiseModel& noise) {
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factors_[index].get());
    EXPECT_SAME_NOISE(pose_prior_factor, noise);
  }

  void EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(const int index) {
    EXPECT_SAME_NODE(index);
    EXPECT_SAME_BETWEEN_FACTOR_AND_NOISE(index);
  }

  void EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE_INTERPOLATED(const int index_a, const int index_b,
                                                                  const double alpha) {
    EXPECT_SAME_NODE_INTERPOLATED(index_a, index_b, alpha);
    EXPECT_SAME_BETWEEN_FACTOR_AND_NOISE_INTERPOLATED(index_a, index_b, alpha);
  }

  void EXPECT_SAME_SECOND_NODE_AND_BETWEEN_FACTOR_AND_NOISE_INTERPOLATED(const int index_a, const int index_b,
                                                                         const double alpha) {
    EXPECT_SAME_NODE(index_b);
    EXPECT_SAME_SECOND_BETWEEN_NOISE_INTERPOLATED(index_a, index_b, alpha);
    EXPECT_SAME_SECOND_BETWEEN_FACTOR_INTERPOLATED(index_a, index_b, alpha);
  }

  std::unique_ptr<na::PoseNodeAdder> pose_node_adder_;
  na::PoseNodeAdderParams params_;
  na::TimestampedNodeAdderModelParams node_adder_model_params_;
  std::vector<lm::TimestampedPoseWithCovariance> pose_measurements_;
  std::vector<lc::Time> timestamps_;
  gtsam::NonlinearFactorGraph factors_;

 private:
  const double time_increment_;
  const lc::Time start_time_;
  const int num_measurements_;
};

TEST_F(PoseNodeAdderTest, AddRemoveCanAddNode) {
  DefaultInitialize();
  EXPECT_FALSE(pose_node_adder_->CanAddNode(10.1));
  constexpr double epsilon = 0.1;
  // Check initialized measurement
  EXPECT_TRUE(pose_node_adder_->CanAddNode(params_.starting_time));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(params_.starting_time + epsilon));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(params_.starting_time - epsilon));
  // Add measurement 0
  pose_node_adder_->AddMeasurement(pose_measurements_[0]);
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[0]));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[0] + epsilon));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(params_.starting_time - epsilon));
  // Add measurement 1
  pose_node_adder_->AddMeasurement(pose_measurements_[1]);
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode((timestamps_[0] + timestamps_[1]) / 2.0));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(params_.starting_time - epsilon));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[1] + epsilon));
  // Add measurement 2
  pose_node_adder_->AddMeasurement(pose_measurements_[2]);
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[2]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1] + epsilon));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(params_.starting_time - epsilon));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[2] + epsilon));
  // Add measurement 3
  pose_node_adder_->AddMeasurement(pose_measurements_[3]);
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[3]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1] + epsilon));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[2] + epsilon));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(params_.starting_time - epsilon));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[3] + epsilon));

  // Remove measurements start, 0, but keep 1 since it's the lower bound and
  // may be used for interpolation
  pose_node_adder_->RemoveMeasurements(timestamps_[1] + epsilon);
  EXPECT_FALSE(pose_node_adder_->CanAddNode(params_.starting_time));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[0]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[2]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[2] + epsilon));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[3]));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[3] + epsilon));
}

TEST_F(PoseNodeAdderTest, AddInitialNodesAndPriorsUsingParams) {
  RandomInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  // Check node value
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_SAME_NODE(params_.starting_time, params_.start_node);
  // Check factor
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_SAME_PRIOR_FACTOR(0, params_.start_node);
  // Check noise
  EXPECT_SAME_PRIOR_NOISE(0, params_.start_noise_models[0]);
}

TEST_F(PoseNodeAdderTest, AddInitialNodesAndPriors) {
  const auto pose = lc::RandomPose();
  const auto time = lc::RandomDouble();
  pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Nodes>()));
  pose_node_adder_->AddInitialNodesAndPriors(pose, params_.start_noise_models, time, factors_);
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_SAME_NODE(time, pose);
  // Check factor
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_SAME_PRIOR_FACTOR(0, pose);
  // Check noise
  EXPECT_SAME_PRIOR_NOISE(0, params_.start_noise_models[0]);
}

TEST_F(PoseNodeAdderTest, AddNode) {
  ZeroInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_EQ(factors_.size(), 1);
  AddMeasurements();
  // Add 1st node
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[0], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  // Add 2nd node
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 3);
  EXPECT_EQ(factors_.size(), 3);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(1);
  // Add 3rd node
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[2], factors_));
  EXPECT_EQ(nodes.size(), 4);
  EXPECT_EQ(factors_.size(), 4);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(1);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(2);
  // Add 4th node in between 3rd and 4th measurement
  const lc::Time timestamp_2_3 = (timestamps_[2] + timestamps_[3]) / 2.0;
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamp_2_3, factors_));
  EXPECT_EQ(nodes.size(), 5);
  EXPECT_EQ(factors_.size(), 5);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(1);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(2);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE_INTERPOLATED(2, 3, 0.5);
}

TEST_F(PoseNodeAdderTest, AddNodeTooSoon) {
  ZeroInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  AddMeasurements();
  // Add 1st node
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[0], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  // Add too soon node
  ASSERT_FALSE(pose_node_adder_->AddNode(params_.starting_time - 1.0, factors_));
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[0], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
}

TEST_F(PoseNodeAdderTest, AddNodeTooLate) {
  ZeroInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  AddMeasurements();
  // Add 1st node
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[0], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  // Add too late node
  ASSERT_FALSE(pose_node_adder_->AddNode(timestamps_.back() + 1.0, factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
}

TEST_F(PoseNodeAdderTest, AddNodeBetweenStartAndFirstMeasurement) {
  ZeroInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_EQ(factors_.size(), 1);
  AddMeasurements();
  // Add 1st node in between start pose and 1st measurement
  const lc::Time timestamp_s_0 = (params_.starting_time + timestamps_[0]) / 2.0;
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamp_s_0, factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE_INTERPOLATED(-1, 0, 0.5);
}

TEST_F(PoseNodeAdderTest, AddSameNode) {
  ZeroInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  AddMeasurements();
  // Add 1st node
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[0], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  // Add same node
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[0], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
}

TEST_F(PoseNodeAdderTest, SplitNode) {
  ZeroInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  AddMeasurements();
  // Add 1st node
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[0], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  // Add 2nd node
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 3);
  EXPECT_EQ(factors_.size(), 3);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(1);
  // Add 3rd node in between 1st and 2nd measurement
  const lc::Time timestamp_0_1 = (timestamps_[0] + timestamps_[1]) / 2.0;
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamp_0_1, factors_));
  EXPECT_EQ(nodes.size(), 4);
  EXPECT_EQ(factors_.size(), 4);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE_INTERPOLATED(0, 1, 0.5);
  EXPECT_SAME_SECOND_NODE_AND_BETWEEN_FACTOR_AND_NOISE_INTERPOLATED(0, 1, 0.5);
}

// Assumes keys start at 1 and increase by 1
TEST_F(PoseNodeAdderTest, OldKeys) {
  ZeroInitialize();
  AddMeasurements();
  EXPECT_TRUE(pose_node_adder_->OldKeys(-1, factors_).empty());
  {
    const auto old_keys = pose_node_adder_->OldKeys(1, factors_);
    ASSERT_EQ(old_keys.size(), 1);
    EXPECT_EQ(old_keys[0], 1);
  }
  // Add 1st node
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[0], factors_));
  {
    const auto old_keys = pose_node_adder_->OldKeys(timestamps_[1], factors_);
    ASSERT_EQ(old_keys.size(), 2);
    EXPECT_EQ(old_keys[0], 1);
    EXPECT_EQ(old_keys[1], 2);
  }
  // Same timestamp shouldn't remove same timestamped node's key
  {
    const auto old_keys = pose_node_adder_->OldKeys(timestamps_[0], factors_);
    ASSERT_EQ(old_keys.size(), 1);
    EXPECT_EQ(old_keys[0], 1);
  }
  // Add 2nd node
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  {
    const auto old_keys = pose_node_adder_->OldKeys(timestamps_[5], factors_);
    ASSERT_EQ(old_keys.size(), 3);
    EXPECT_EQ(old_keys[0], 1);
    EXPECT_EQ(old_keys[1], 2);
    EXPECT_EQ(old_keys[2], 3);
  }
  {
    const auto old_keys = pose_node_adder_->OldKeys((timestamps_[0] + timestamps_[1]) / 2.0, factors_);
    ASSERT_EQ(old_keys.size(), 2);
    EXPECT_EQ(old_keys[0], 1);
    EXPECT_EQ(old_keys[1], 2);
  }
}

TEST_F(PoseNodeAdderTest, NewStartTimeDurationViolation) {
  params_.min_num_states = 0;
  params_.max_num_states = 10;
  params_.starting_time = 0.0;
  params_.ideal_duration = 1.5;
  params_.Initialize();
  pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Nodes>()));
  AddMeasurements();
  // Empty nodes, so expect invalid oldest time
  EXPECT_TRUE(pose_node_adder_->SlideWindowNewStartTime() == boost::none);
  pose_node_adder_->AddInitialNodesAndPriors(factors_);
  // Duration: 0. States: 1
  // Min/Max states not applicable, less than ideal duration (0 < 1.5), so
  // should return 0
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, 0.0);
  }
  // Add node. Duration: 1. States: 2
  // Min/Max states not applicable, less than ideal duration (1 < 1.5), so
  // should return 0
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[0], factors_));
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, 0.0);
  }
  // Add node. Duration: 2, states: 3
  // Duration > ideal duration, should remove start node and return 1st nodes' time
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, timestamps_[0]);
  }
  // Add node. Duration: 3, states: 3
  // Duration > ideal duration, should remove start and 1st node and return 2nd nodes' time
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[2], factors_));
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, timestamps_[1]);
  }
}

TEST_F(PoseNodeAdderTest, NewStartTimeMinMaxStatesViolation) {
  params_.min_num_states = 1;
  params_.max_num_states = 3;
  params_.starting_time = 0.0;
  params_.ideal_duration = 100;
  params_.Initialize();
  pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Nodes>()));
  const auto& nodes = pose_node_adder_->nodes();
  AddMeasurements();
  // Empty nodes, so expect invalid oldest time
  EXPECT_TRUE(pose_node_adder_->SlideWindowNewStartTime() == boost::none);
  EXPECT_EQ(nodes.size(), 0);
  pose_node_adder_->AddInitialNodesAndPriors(factors_);
  // 1 node <= min_states (1), so expect invalid oldest time
  EXPECT_TRUE(pose_node_adder_->SlideWindowNewStartTime() == boost::none);
  EXPECT_EQ(nodes.size(), 1);
  // Add node. States: 2, less than ideal duration so should return 0.
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[0], factors_));
  EXPECT_EQ(nodes.size(), 2);
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, 0.0);
  }
  // Add node. States: 3, less than ideal duration so should return 0.
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 3);
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, 0.0);
  }
  // Add node. States: 4, > max num states, should remove 1st state.
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[2], factors_));
  EXPECT_EQ(nodes.size(), 4);
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, timestamps_[0]);
  }
  // Add node. States: 5, > max num states, should remove 1st and 2nd state.
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[3], factors_));
  EXPECT_EQ(nodes.size(), 5);
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, timestamps_[1]);
  }
}

TEST_F(PoseNodeAdderTest, SlideWindow) {
  params_.start_node = gtsam::Pose3::identity();
  params_.starting_time = 0.0;
  params_.min_num_states = 2;
  params_.max_num_states = 5;
  params_.ideal_duration = 1.5;
  params_.Initialize();
  pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Nodes>()));
  pose_node_adder_->AddInitialNodesAndPriors(factors_);

  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_EQ(*pose_node_adder_->StartTime(), params_.starting_time);
  EXPECT_EQ(*pose_node_adder_->EndTime(), params_.starting_time);
  AddMeasurements();
  // Add 1st node, nodes: 2, duration: 1
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[0], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  // Add 2nd node, nodes: 3, duration 2
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 3);
  EXPECT_EQ(factors_.size(), 3);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(1);
  // Slide window with time older than oldest allowed time, nothing should happen
  ASSERT_TRUE(pose_node_adder_->SlideWindow(-1, boost::none, gtsam::KeyVector(), params_.huber_k, factors_));
  EXPECT_EQ(nodes.size(), 3);
  EXPECT_EQ(factors_.size(), 3);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(1);
  // Add 3rd node, nodes: 4, duration 3
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[2], factors_));
  EXPECT_EQ(nodes.size(), 4);
  EXPECT_EQ(factors_.size(), 4);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(0);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(1);
  EXPECT_SAME_NODE_AND_BETWEEN_FACTOR_AND_NOISE(2);
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[0] - 0.1));
  // Slide window
  // Expect to remove first two nodes, so duration is 1
  const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
  ASSERT_TRUE(new_oldest_time != boost::none);
  EXPECT_EQ(*new_oldest_time, timestamps_[1]);
  const auto old_keys = pose_node_adder_->OldKeys(*new_oldest_time, factors_);
  EXPECT_EQ(old_keys.size(), 2);
  ASSERT_TRUE(pose_node_adder_->SlideWindow(*new_oldest_time, boost::none, old_keys, params_.huber_k, factors_));
  EXPECT_EQ(nodes.size(), 2);
  // 3 nodes should be the 3 poses added after the start node
  EXPECT_SAME_NODE(timestamps_[1], pose(1));
  EXPECT_SAME_NODE(timestamps_[2], pose(2));
  // Between factors aren't removed yet, so all between factors should remain,
  // only prior should be removed/added (removed from start node, added to 1st node)
  EXPECT_EQ(factors_.size(), 4);
  // Indices changed, so factor 0 -> between factor 1 and so on
  EXPECT_SAME_BETWEEN_FACTOR(0, 0);
  EXPECT_SAME_BETWEEN_FACTOR(1, 1);
  EXPECT_SAME_BETWEEN_FACTOR(2, 2);
  // Prior factor added last
  EXPECT_SAME_PRIOR_FACTOR(3, pose(1));
  // Since no marginals available, noise should default to start noise
  EXPECT_SAME_PRIOR_NOISE(3, params_.start_noise_models[0]);
  EXPECT_EQ(*pose_node_adder_->StartTime(), timestamps_[1]);
  EXPECT_EQ(*pose_node_adder_->EndTime(), timestamps_[2]);
  // Old measurements removed when slide window but lower bound kept in case needed for interpolation
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[0]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1]));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(params_.starting_time));
}
// TODO(rsoussan): Test slide window with valid marginals to create valid prior covariances

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
