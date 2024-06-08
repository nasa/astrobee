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
  PoseNodeAdderTest() : time_increment_(1.0), start_time_(0.0), num_measurements_(20) {
    params_ = na::DefaultPoseNodeAdderParams();
    node_adder_model_params_.huber_k = 1.345;
  }

  // Create random pose with covariance measurements
  void SetUp() final {
    for (int i = 0; i < num_measurements_; ++i) {
      const lc::Time time = start_time_ + time_increment_ * i;
      const lm::PoseWithCovarianceMeasurement pose_measurement(lc::RandomPose(), lc::RandomPoseCovariance(), time);
      pose_measurements_.emplace_back(pose_measurement);
      timestamps_.emplace_back(time);
    }
  }

  // Add pose measurements to node adder
  void AddMeasurements() {
    for (const auto& measurement : pose_measurements_) {
      pose_node_adder_->AddMeasurement(measurement);
    }
  }

  // Randomly initialize with random start time
  void RandomlyInitialize() { RandomlyInitialize(lc::RandomDouble()); }

  // Randomly initialize with set start time
  void RandomlyInitialize(const lc::Time starting_time) {
    params_.start_node = lc::RandomPose();
    params_.starting_time = starting_time;
    pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Values>()));
    pose_node_adder_->AddInitialNodesAndPriors(factors_);
  }

  // Initialize with default params
  void DefaultInitialize() {
    pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Values>()));
    pose_node_adder_->AddInitialNodesAndPriors(factors_);
  }

  // Initialize at identity pose and 0 start time
  void ZeroInitialize() {
    params_.start_node = gtsam::Pose3::identity();
    params_.starting_time = 0.0;
    pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Values>()));
    pose_node_adder_->AddInitialNodesAndPriors(factors_);
  }

  // Access the measurement pose at the provided measurement index.
  Eigen::Isometry3d measurement_pose(const int index) { return lc::EigenPose(pose_measurements_[index].pose); }

  // Access the measurement pose covariance at the provided measurement index.
  lc::PoseCovariance measurement_covariance(const int index) { return pose_measurements_[index].covariance; }

  // Access the timestamp at the provided measurement index.
  lc::Time measurement_timestamp(const int index) { return timestamps_[index]; }

  // Check that the pose in the node adder at the provided timestamp matches the provided pose.
  void EXPECT_SAME_NODE_POSE(const lc::Time timestamp, const Eigen::Isometry3d& pose) {
    const auto& nodes = pose_node_adder_->nodes();
    EXPECT_TRUE(nodes.Contains(timestamp));
    const auto node = nodes.Node(timestamp);
    ASSERT_TRUE(node != boost::none);
    EXPECT_MATRIX_NEAR(node->matrix(), pose, 1e-6);
  }

  // Check that the pose in the node adder at the provided timestamp matches the provided pose.
  void EXPECT_SAME_NODE_POSE(const lc::Time timestamp, const gtsam::Pose3& pose) {
    EXPECT_SAME_NODE_POSE(timestamp, lc::EigenPose(pose));
  }

  // Check that the pose in the node adder at the provided measurement index matches the computed pose using the same
  // index. The expect pose uses the starting pose with the relative measurement pose at that timestamp.
  void EXPECT_SAME_NODE_POSE(const int index) {
    const gtsam::Pose3 relative_pose = lc::GtPose(measurement_pose(0).inverse() * measurement_pose(index));
    const gtsam::Pose3 expected_pose = params_.start_node * relative_pose;
    EXPECT_SAME_NODE_POSE(measurement_timestamp(index), expected_pose);
  }

  // Check that the pose in the node adder at the interpolated timestamp using the provided indices and alpha value
  // matches the computed interpolated pose using the same values.
  void EXPECT_SAME_NODE_POSE_INTERPOLATED(const int measurement_index_a, const int measurement_index_b,
                                          const double alpha) {
    const auto interpolated_pose = InterpolatedPose(measurement_index_a, measurement_index_b, alpha);
    const auto relative_pose = lc::GtPose(measurement_pose(0).inverse() * interpolated_pose);
    const auto expected_pose = params_.start_node * relative_pose;
    const lc::Time timestamp_a_b =
      measurement_timestamp(measurement_index_a) * alpha + measurement_timestamp(measurement_index_b) * (1.0 - alpha);
    EXPECT_SAME_NODE_POSE(timestamp_a_b, expected_pose);
  }

  // Check that the prior factor pose at the provided factor index matches the provided pose.
  void EXPECT_SAME_PRIOR_FACTOR_POSE(const int factor_index, const Eigen::Isometry3d& pose) {
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factors_[factor_index].get());
    ASSERT_TRUE(pose_prior_factor);
    EXPECT_MATRIX_NEAR(pose_prior_factor->prior(), pose, 1e-6);
  }

  // Check that the prior factor pose at the provided factor index matches the provided pose.
  void EXPECT_SAME_PRIOR_FACTOR_POSE(const int factor_index, const gtsam::Pose3& pose) {
    return EXPECT_SAME_PRIOR_FACTOR_POSE(factor_index, lc::EigenPose(pose));
  }

  // Check that the relative pose between measurements at index a and b matches the between factor pose at the provided
  // index.
  void EXPECT_SAME_BETWEEN_FACTOR_POSE(const int factor_index, const int measurement_index_a,
                                       const int measurement_index_b) {
    const auto& pose_a = measurement_pose(measurement_index_a);
    const auto& pose_b = measurement_pose(measurement_index_b);
    const auto relative_pose = pose_a.inverse() * pose_b;
    EXPECT_SAME_BETWEEN_FACTOR_POSE(factor_index, relative_pose);
  }

  // Check that the relative pose between measurements at the provided index and the previous index to that matches the
  // between factor pose at the provided index.
  void EXPECT_SAME_BETWEEN_FACTOR_POSE(const int factor_index, const int measurement_index) {
    EXPECT_SAME_BETWEEN_FACTOR_POSE(factor_index, measurement_index - 1, measurement_index);
  }

  // Check that the relative pose between measurements at the provided index and the previous index to that matches the
  // between factor pose at the same index.
  void EXPECT_SAME_BETWEEN_FACTOR_POSE(const int index) {
    const auto& pose_a = measurement_pose(index - 1);
    const auto& pose_b = measurement_pose(index);
    const auto relative_pose = pose_a.inverse() * pose_b;
    EXPECT_SAME_BETWEEN_FACTOR_POSE(index, relative_pose);
  }

  // Check that the provided pose matches the between factor pose at the provided index.
  void EXPECT_SAME_BETWEEN_FACTOR_POSE(const int factor_index, const Eigen::Isometry3d& pose) {
    const auto pose_between_factor = dynamic_cast<gtsam::BetweenFactor<gtsam::Pose3>*>(factors_[factor_index].get());
    ASSERT_TRUE(pose_between_factor);
    EXPECT_MATRIX_NEAR(pose_between_factor->measured(), pose, 1e-6);
  }

  // Check that the provided pose matches the between factor pose at the provided index.
  // Assumes the first factor index is the prior factor and the subsequent indices are between factors.
  void EXPECT_SAME_BETWEEN_FACTOR_POSE(const int factor_index, const gtsam::Pose3& pose) {
    EXPECT_SAME_BETWEEN_FACTOR_POSE(factor_index, lc::EigenPose(pose));
  }

  Eigen::Isometry3d InterpolatedPose(const int index_a, const int index_b, const double alpha) {
    return lc::Interpolate(measurement_pose(index_a), measurement_pose(index_b), alpha);
  }

  // Check that the relative pose in the between factor at the interpolated timestamp using the provided indices and
  // alpha value matches the computed interpolated pose using the same values.
  void EXPECT_SAME_BETWEEN_FACTOR_POSE_INTERPOLATED(const int measurement_index_a, const int measurement_index_b,
                                                    const double alpha) {
    const auto interpolated_pose = InterpolatedPose(measurement_index_a, measurement_index_b, alpha);
    const auto& pose_a = measurement_pose(measurement_index_a);
    const auto relative_pose = pose_a.inverse() * interpolated_pose;
    EXPECT_SAME_BETWEEN_FACTOR_POSE(measurement_index_b, relative_pose);
  }

  // Check that the relative pose in the between factor at the interpolated timestamp using the provided indices and
  // alpha value matches the computed interpolated pose using the same values. Here rather than using the portion from
  // the pose at measurement_index_a to the interpolated timestamp, the portion from the interpolated timestamp to the
  // pose at measurement_index_b is used.
  void EXPECT_SAME_SECOND_BETWEEN_FACTOR_POSE_INTERPOLATED(const int measurement_index_a, const int measurement_index_b,
                                                           const double alpha) {
    const auto pose_interpolated = InterpolatedPose(measurement_index_a, measurement_index_b, alpha);
    const Eigen::Isometry3d relative_pose = pose_interpolated.inverse() * measurement_pose(measurement_index_b);
    EXPECT_SAME_BETWEEN_FACTOR_POSE(measurement_index_b + 1, relative_pose);
  }

  // Check that the factor noise model matches the provided covariance.
  template <typename FactorPtrType>
  void EXPECT_SAME_NOISE(const FactorPtrType factor, const lc::PoseCovariance& covariance) {
    EXPECT_MATRIX_NEAR(na::Covariance(factor->noiseModel()), covariance, 1e-6);
  }

  // Check that the factor noise model matches the provided noise model.
  template <typename FactorPtrType>
  void EXPECT_SAME_NOISE(const FactorPtrType factor, const gtsam::SharedNoiseModel noise) {
    EXPECT_SAME_NOISE(factor, na::Covariance(noise));
  }

  // Check that the between factor noise at the provided index matches the provided covariance.
  void EXPECT_SAME_BETWEEN_FACTOR_NOISE(const int factor_index, const lc::PoseCovariance& covariance) {
    // Handle special case where need prior noise (remove this once covariance computation is fixed)
    /*if (factor_index == -1) {
      EXPECT_SAME_NOISE(dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factors_[0].get()), covariance);
    } else {*/
    const auto pose_between_factor = dynamic_cast<gtsam::BetweenFactor<gtsam::Pose3>*>(factors_[factor_index].get());
    EXPECT_SAME_NOISE(pose_between_factor, covariance);
    //}
  }

  // Check that the between factor noise at the provided index matches the provided covariance.
  void EXPECT_SAME_BETWEEN_FACTOR_NOISE(const int factor_index) {
    // TODO(rsoussan): Update this with interpolation/relative computation for cov.
    // EXPECT_SAME_BETWEEN_FACTOR_NOISE(factor_index, measurement_covariance(factor_index));
  }

  // Check that the between factor noise at factor index_b matches the computed interpolated covariance at measurement
  // indices a and b.
  void EXPECT_SAME_BETWEEN_FACTOR_NOISE_INTERPOLATED(const int measurement_index_a, const int measurement_index_b,
                                                     const double alpha) {
    // TODO(rsoussan): Update this with interpolation/relative computation for cov.
    // EXPECT_SAME_BETWEEN_FACTOR_NOISE(measurement_index_b, measurement_covariance(measurement_index_a));
  }

  // Check that the between factor noise at factor index_b + 1 matches the second half of the computed interpolated
  // covariance at measurement indices a and b.
  void EXPECT_SAME_SECOND_BETWEEN_FACTOR_NOISE_INTERPOLATED(const int index_a, const int index_b, const double alpha) {
    // TODO(rsoussan): Update this with interpolation/relative computation for cov.
    // EXPECT_SAME_BETWEEN_FACTOR_NOISE(index_b + 1, measurement_covariance(index_b));
  }

  // Check that the factor noise at the factor index matches the provided noise.
  void EXPECT_SAME_PRIOR_NOISE(const int factor_index, const gtsam::SharedNoiseModel& noise) {
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factors_[factor_index].get());
    EXPECT_SAME_NOISE(pose_prior_factor, noise);
  }

  // Check that the relative pose, noise, and node pose at the provided factor index
  // matches the expected pose and noise at the same measurement index.
  void EXPECT_SAME_BETWEEN_FACTOR(const int index) {
    EXPECT_SAME_NODE_POSE(index);
    EXPECT_SAME_BETWEEN_FACTOR_POSE(index);
    EXPECT_SAME_BETWEEN_FACTOR_NOISE(index);
  }

  // Check that the relative pose, noise, and node pose at factor index b
  // match computed interpolated relative pose, noise, and node pose using measurement
  // indices a and b.
  void EXPECT_SAME_BETWEEN_FACTOR_INTERPOLATED(const int index_a, const int index_b, const double alpha) {
    EXPECT_SAME_NODE_POSE_INTERPOLATED(index_a, index_b, alpha);
    EXPECT_SAME_BETWEEN_FACTOR_POSE_INTERPOLATED(index_a, index_b, alpha);
    EXPECT_SAME_BETWEEN_FACTOR_NOISE_INTERPOLATED(index_a, index_b, alpha);
  }

  // Check that the relative pose, noise, and node pose at factor index b
  // match the second half of the computed interpolated relative pose, noise, and node pose using measurement
  // indices a and b.
  void EXPECT_SAME_SECOND_BETWEEN_FACTOR_INTERPOLATED(const int index_a, const int index_b, const double alpha) {
    EXPECT_SAME_NODE_POSE(index_b);
    EXPECT_SAME_SECOND_BETWEEN_FACTOR_NOISE_INTERPOLATED(index_a, index_b, alpha);
    EXPECT_SAME_SECOND_BETWEEN_FACTOR_POSE_INTERPOLATED(index_a, index_b, alpha);
  }

  std::unique_ptr<na::PoseNodeAdder> pose_node_adder_;
  na::PoseNodeAdderParams params_;
  na::TimestampedNodeAdderModelParams node_adder_model_params_;
  std::vector<lm::PoseWithCovarianceMeasurement> pose_measurements_;
  std::vector<lc::Time> timestamps_;
  gtsam::NonlinearFactorGraph factors_;

 private:
  const double time_increment_;
  const lc::Time start_time_;
  const int num_measurements_;
};

// Test CanAddNode() with add and removing measurements.
TEST_F(PoseNodeAdderTest, AddRemoveCanAddNode) {
  DefaultInitialize();
  EXPECT_FALSE(pose_node_adder_->CanAddNode(10.1));
  constexpr double epsilon = 0.1;
  // With no measurements added, no nodes should be able to be added.
  EXPECT_FALSE(pose_node_adder_->CanAddNode(params_.starting_time));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(params_.starting_time + epsilon));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(params_.starting_time - epsilon));
  // Add measurement 0, only a node at that timestamp should be able to be added.
  pose_node_adder_->AddMeasurement(pose_measurements_[0]);
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[0]));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[0] + epsilon));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[0] - epsilon));
  // Add measurement 1, nodes a times [t_0, t_1] should be able to be added.
  pose_node_adder_->AddMeasurement(pose_measurements_[1]);
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[0] - epsilon));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[0]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode((timestamps_[0] + timestamps_[1]) / 2.0));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1]));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[1] + epsilon));
  // Add measurement 2, nodes a times [t_0, t_2] should be able to be added.
  pose_node_adder_->AddMeasurement(pose_measurements_[2]);
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[0] - epsilon));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[0]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode((timestamps_[0] + timestamps_[1]) / 2.0));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1] + epsilon));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[2]));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[2] + epsilon));
  // Add measurement 3, nodes a times [t_0, t_3] should be able to be added.
  pose_node_adder_->AddMeasurement(pose_measurements_[3]);
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[0] - epsilon));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[0]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode((timestamps_[0] + timestamps_[1]) / 2.0));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1] + epsilon));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[2]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[2] + epsilon));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[3]));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[3] + epsilon));

  // Remove measurement 0, but keep 1 since it's the lower bound and
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

// Test initialization at a random time and pose.
TEST_F(PoseNodeAdderTest, InitializeWithRandomTime) {
  const auto pose = lc::RandomPose();
  const auto time = lc::RandomDouble();
  pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Values>()));
  pose_node_adder_->AddInitialNodesAndPriors(pose, params_.start_noise_models, time, factors_);
  const auto& nodes = pose_node_adder_->nodes();
  // There should only be one start node and one factor (prior factor on start node)
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_SAME_NODE_POSE(time, pose);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, pose);
  EXPECT_SAME_PRIOR_NOISE(0, params_.start_noise_models[0]);
}

// Test between factor and node creation after adding measurements and using
// random initialization.
TEST_F(PoseNodeAdderTest, AddNodesAfterRandomInitialization) {
  RandomlyInitialize(0.0);
  const auto& nodes = pose_node_adder_->nodes();
  // Starting node and prior factor should exist after initialization.
  // Values should be the same as starting node/time/noise in provided params.
  // Graph structure:
  // P_0 -> N_0
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_SAME_NODE_POSE(params_.starting_time, params_.start_node);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_PRIOR_NOISE(0, params_.start_noise_models[0]);

  AddMeasurements();
  // Add Node at timestamp 1.
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_NODE_POSE(params_.starting_time, params_.start_node);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_PRIOR_NOISE(0, params_.start_noise_models[0]);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  // Add Node at timestamp 2.
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1 <-B_1_2-> N_2
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[2], factors_));
  EXPECT_EQ(nodes.size(), 3);
  EXPECT_EQ(factors_.size(), 3);
  EXPECT_SAME_NODE_POSE(params_.starting_time, params_.start_node);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_PRIOR_NOISE(0, params_.start_noise_models[0]);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  EXPECT_SAME_BETWEEN_FACTOR(2);
  // Add Node at timestamp 3.
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1 <-B_1_2-> N_2 <-B_2_3-> N_3
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[3], factors_));
  EXPECT_EQ(nodes.size(), 4);
  EXPECT_EQ(factors_.size(), 4);
  EXPECT_SAME_NODE_POSE(params_.starting_time, params_.start_node);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_PRIOR_NOISE(0, params_.start_noise_models[0]);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  EXPECT_SAME_BETWEEN_FACTOR(2);
  EXPECT_SAME_BETWEEN_FACTOR(3);
  // Add Node in between timestamps 3 and 4. Node and between factor should be interpolated.
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1 <-B_1_2-> N_2 <-B_2_3-> N_3 <-B_3_3.5-> N_3.5
  const lc::Time timestamp_3_4 = (timestamps_[3] + timestamps_[4]) / 2.0;
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamp_3_4, factors_));
  EXPECT_EQ(nodes.size(), 5);
  EXPECT_EQ(factors_.size(), 5);
  EXPECT_SAME_NODE_POSE(params_.starting_time, params_.start_node);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_PRIOR_NOISE(0, params_.start_noise_models[0]);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  EXPECT_SAME_BETWEEN_FACTOR(2);
  EXPECT_SAME_BETWEEN_FACTOR(3);
  EXPECT_SAME_BETWEEN_FACTOR_INTERPOLATED(3, 4, 0.5);
}

// Test adding node before first measurement, should fail.
TEST_F(PoseNodeAdderTest, AddNodeTooSoon) {
  ZeroInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  AddMeasurements();
  // Add node at timestamp 1.
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  // Add node before starting time.
  ASSERT_FALSE(pose_node_adder_->AddNode(params_.starting_time - 1.0, factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_BETWEEN_FACTOR(1);
}

// Test adding node after last measurement, should fail.
TEST_F(PoseNodeAdderTest, AddNodeTooLate) {
  ZeroInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  AddMeasurements();
  // Add node at timestamp 1.
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  // Try to add node after last measurement time.
  ASSERT_FALSE(pose_node_adder_->AddNode(timestamps_.back() + 1.0, factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_BETWEEN_FACTOR(1);
}

// Test adding interpolated node between start node and first measurement.
TEST_F(PoseNodeAdderTest, AddNodeBetweenStartAndFirstMeasurement) {
  ZeroInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_EQ(factors_.size(), 1);
  AddMeasurements();
  // Add node in between start pose and 1st measurement
  // Graph structure:
  // P_0 -> N_0 <-B_0_0.5-> N_0.5
  const lc::Time timestamp_0_5 = (params_.starting_time + timestamps_[1]) / 2.0;
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamp_0_5, factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_BETWEEN_FACTOR_INTERPOLATED(0, 1, 0.5);
}

// Adding the same node twice should result in no change to the graph size.
TEST_F(PoseNodeAdderTest, AddSameNode) {
  ZeroInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  AddMeasurements();
  // Add node at timestamp 1.
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  // Add same node at timestamp 1, shouldn't change graph structure.
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_BETWEEN_FACTOR(1);
}

// Test adding node in between already added 1st and 2nd nodes.
TEST_F(PoseNodeAdderTest, SplitNode) {
  ZeroInitialize();
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  AddMeasurements();
  // Add node at timestamp 1.
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  // Add node at timestamp 2.
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1 <-B_1_2-> N_2
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[2], factors_));
  EXPECT_EQ(nodes.size(), 3);
  EXPECT_EQ(factors_.size(), 3);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  EXPECT_SAME_BETWEEN_FACTOR(2);
  // Add node at timestamp in between 1st and 2nd measurement
  const lc::Time timestamp_1_2 = (timestamps_[1] + timestamps_[2]) / 2.0;
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamp_1_2, factors_));
  EXPECT_EQ(nodes.size(), 4);
  EXPECT_EQ(factors_.size(), 4);
  EXPECT_SAME_PRIOR_FACTOR_POSE(0, params_.start_node);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  EXPECT_SAME_BETWEEN_FACTOR_INTERPOLATED(1, 2, 0.5);
  EXPECT_SAME_SECOND_BETWEEN_FACTOR_INTERPOLATED(1, 2, 0.5);
}

// Test getting old keys from the pose node adder.
// Assumes keys start at 1 and increase by 1
TEST_F(PoseNodeAdderTest, OldKeys) {
  ZeroInitialize();
  AddMeasurements();
  // No keys should return when using timestamp before the start node.
  EXPECT_TRUE(pose_node_adder_->OldKeys(-1, factors_).empty());
  {
    // Start node key should return when using more recent timestamp than the start node.
    const auto old_keys = pose_node_adder_->OldKeys(1, factors_);
    ASSERT_EQ(old_keys.size(), 1);
    EXPECT_EQ(old_keys[0], 1);
  }
  // Add node at 1st measurement timestamp.
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  {
    // Start node and first node keys should return when given 2nd timestamp.
    const auto old_keys = pose_node_adder_->OldKeys(timestamps_[2], factors_);
    ASSERT_EQ(old_keys.size(), 2);
    EXPECT_EQ(old_keys[0], 1);
    EXPECT_EQ(old_keys[1], 2);
  }
  // Same timestamp shouldn't remove same timestamped node's key -
  // only start node key should be old.
  {
    const auto old_keys = pose_node_adder_->OldKeys(timestamps_[1], factors_);
    ASSERT_EQ(old_keys.size(), 1);
    EXPECT_EQ(old_keys[0], 1);
  }
  // Add node at second timestamp.
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[2], factors_));
  {
    // Start, first, and second node keys should return when given more recent timestamp.
    const auto old_keys = pose_node_adder_->OldKeys(timestamps_[5], factors_);
    ASSERT_EQ(old_keys.size(), 3);
    EXPECT_EQ(old_keys[0], 1);
    EXPECT_EQ(old_keys[1], 2);
    EXPECT_EQ(old_keys[2], 3);
  }
  {
    // Start and first node keys should return when given timestamp more recent than 1st
    // node but older than second node.
    const auto old_keys = pose_node_adder_->OldKeys((timestamps_[1] + timestamps_[2]) / 2.0, factors_);
    ASSERT_EQ(old_keys.size(), 2);
    EXPECT_EQ(old_keys[0], 1);
    EXPECT_EQ(old_keys[1], 2);
  }
}

// Test slide window new start time when max duration violations occur.
TEST_F(PoseNodeAdderTest, NewStartTimeDurationViolation) {
  params_.min_num_states = 0;
  params_.max_num_states = 10;
  params_.starting_time = 0.0;
  params_.ideal_duration = 1.5;
  pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Values>()));
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
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, 0.0);
  }
  // Add node. Duration: 2, states: 3
  // Duration > ideal duration, should remove start node and return 1st nodes' time
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[2], factors_));
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, timestamps_[1]);
  }
  // Add node. Duration: 3, states: 3
  // Duration > ideal duration, should remove start and 1st node and return 2nd nodes' time
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[3], factors_));
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, timestamps_[2]);
  }
}

// Test slide window new start time when max states violations occur.
TEST_F(PoseNodeAdderTest, NewStartTimeMinMaxStatesViolation) {
  params_.min_num_states = 1;
  params_.max_num_states = 3;
  params_.starting_time = 0.0;
  params_.ideal_duration = 100;
  pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Values>()));
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
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 2);
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, 0.0);
  }
  // Add node. States: 3, less than ideal duration so should return 0.
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[2], factors_));
  EXPECT_EQ(nodes.size(), 3);
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, 0.0);
  }
  // Add node. States: 4, > max num states, should remove 1st state.
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[3], factors_));
  EXPECT_EQ(nodes.size(), 4);
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, timestamps_[1]);
  }
  // Add node. States: 5, > max num states, should remove 1st and 2nd state.
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[4], factors_));
  EXPECT_EQ(nodes.size(), 5);
  {
    const auto new_oldest_time = pose_node_adder_->SlideWindowNewStartTime();
    ASSERT_TRUE(new_oldest_time != boost::none);
    EXPECT_EQ(*new_oldest_time, timestamps_[2]);
  }
}

// Test sliding window
TEST_F(PoseNodeAdderTest, SlideWindow) {
  params_.start_node = gtsam::Pose3::identity();
  params_.starting_time = 0.0;
  params_.min_num_states = 2;
  params_.max_num_states = 5;
  params_.ideal_duration = 1.5;
  pose_node_adder_.reset(new na::PoseNodeAdder(params_, node_adder_model_params_, std::make_shared<no::Values>()));
  pose_node_adder_->AddInitialNodesAndPriors(factors_);

  // Only start node exists
  // Graph structure:
  // P_0 -> N_0
  const auto& nodes = pose_node_adder_->nodes();
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_EQ(*pose_node_adder_->StartTime(), params_.starting_time);
  EXPECT_EQ(*pose_node_adder_->EndTime(), params_.starting_time);
  AddMeasurements();
  // Add 1st node, nodes: 2, duration: 1
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[1], factors_));
  EXPECT_EQ(nodes.size(), 2);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  // Add 2nd node, nodes: 3, duration 2
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1 <-B_1_2-> N_2
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[2], factors_));
  EXPECT_EQ(nodes.size(), 3);
  EXPECT_EQ(factors_.size(), 3);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  EXPECT_SAME_BETWEEN_FACTOR(2);
  // Slide window with time older than oldest allowed time, nothing should happen
  ASSERT_TRUE(pose_node_adder_->SlideWindow(-1, boost::none, gtsam::KeyVector(), params_.huber_k, factors_));
  EXPECT_EQ(nodes.size(), 3);
  EXPECT_EQ(factors_.size(), 3);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  EXPECT_SAME_BETWEEN_FACTOR(2);
  // Add 3rd node, nodes: 4, duration 3
  // Graph structure:
  // P_0 -> N_0 <-B_0_1-> N_1 <-B_1_2-> N_2 <-B_2_3-> N_3
  ASSERT_TRUE(pose_node_adder_->AddNode(timestamps_[3], factors_));
  EXPECT_EQ(nodes.size(), 4);
  EXPECT_EQ(factors_.size(), 4);
  EXPECT_SAME_BETWEEN_FACTOR(1);
  EXPECT_SAME_BETWEEN_FACTOR(2);
  EXPECT_SAME_BETWEEN_FACTOR(3);
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1] - 0.1));
  // Slide window
  // Expect to remove first two nodes, so duration is 1
  const auto new_start_time = pose_node_adder_->SlideWindowNewStartTime();
  ASSERT_TRUE(new_start_time != boost::none);
  EXPECT_EQ(*new_start_time, timestamps_[2]);
  const auto old_keys = pose_node_adder_->OldKeys(*new_start_time, factors_);
  EXPECT_EQ(old_keys.size(), 2);
  ASSERT_TRUE(pose_node_adder_->SlideWindow(*new_start_time, boost::none, old_keys, params_.huber_k, factors_));
  // Graph structure:
  // P_2 -> N_2 <-B_2_3-> N_3
  EXPECT_EQ(nodes.size(), 2);
  // 2 nodes should be the 2 poses added after the start node
  EXPECT_SAME_NODE_POSE(2);
  EXPECT_SAME_NODE_POSE(3);
  // Between factors aren't removed yet (removed in sliding window graph optimizer),
  // so all between factors should remain,
  // only prior should be removed/added (removed from start node, added to 1st node)
  EXPECT_EQ(factors_.size(), 4);
  // Indices changed since prior removed then added last, so factor index 0 -> between factor 1 and so on
  EXPECT_SAME_BETWEEN_FACTOR_POSE(0, 1);
  EXPECT_SAME_BETWEEN_FACTOR_POSE(1, 2);
  EXPECT_SAME_BETWEEN_FACTOR_POSE(2, 3);
  // Prior factor added last
  {
    const gtsam::Pose3 relative_pose = lc::GtPose(measurement_pose(0).inverse() * measurement_pose(2));
    const gtsam::Pose3 expected_prior_pose = params_.start_node * relative_pose;
    EXPECT_SAME_PRIOR_FACTOR_POSE(3, expected_prior_pose);
  }
  // Since no marginals available, noise should default to start noise
  EXPECT_SAME_PRIOR_NOISE(3, params_.start_noise_models[0]);
  EXPECT_EQ(*pose_node_adder_->StartTime(), timestamps_[2]);
  EXPECT_EQ(*pose_node_adder_->EndTime(), timestamps_[3]);
  // Old measurements removed when slide window but lower bound kept in case needed for interpolation
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[1]));
  EXPECT_TRUE(pose_node_adder_->CanAddNode(timestamps_[2]));
  EXPECT_FALSE(pose_node_adder_->CanAddNode(timestamps_[0]));
}
// TODO(rsoussan): Test slide window with valid marginals to create valid prior covariances

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
