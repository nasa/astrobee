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

#include <factor_adders/standstill_factor_adder.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/standstill_measurement.h>
#include <node_adders/node_adder.h>
#include <node_adders/utilities.h>

#include <gtest/gtest.h>

namespace fa = factor_adders;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace na = node_adders;

// Test node adder that just returns keys that should be used.
// Key values are calculated using the integer timestamps passed, where the
// first key is a pose key and second is a velocity key.
// Pose keys are 2*timestamp and velocity keys are 2*timestamp + 1.
class SimplePoseVelocityNodeAdder : public na::NodeAdder {
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
};

class StandstillFactorAdderTest : public ::testing::Test {
 public:
  StandstillFactorAdderTest() { node_adder_.reset(new SimplePoseVelocityNodeAdder()); }

  void SetUp() final {}

  void AddMeasurements() {
    constexpr int kNumMeasurements = 10;
    for (int i = 0; i < kNumMeasurements; ++i) {
      const lm::StandstillMeasurement measurement(i + 1, i);
      measurements_.emplace_back(measurement);
      factor_adder_->AddMeasurement(measurement);
    }
  }

  void Initialize(const fa::StandstillFactorAdderParams& params) {
    factor_adder_.reset(new fa::StandstillFactorAdder<SimplePoseVelocityNodeAdder>(params, node_adder_));
    // Create zero velocity noise
    const gtsam::Vector3 velocity_prior_noise_sigmas(
      (gtsam::Vector(3) << params.prior_velocity_stddev, params.prior_velocity_stddev, params.prior_velocity_stddev)
        .finished());
    zero_velocity_noise_ = localization_common::Robust(
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)),
      params.huber_k);

    // Create zero relative pose noise
    const gtsam::Vector6 pose_between_noise_sigmas(
      (gtsam::Vector(6) << params.pose_between_factor_rotation_stddev, params.pose_between_factor_rotation_stddev,
       params.pose_between_factor_rotation_stddev, params.pose_between_factor_translation_stddev,
       params.pose_between_factor_translation_stddev, params.pose_between_factor_translation_stddev)
        .finished());
    zero_relative_pose_noise_ = localization_common::Robust(
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_between_noise_sigmas)),
      params.huber_k);
  }

  fa::StandstillFactorAdderParams DefaultParams() {
    fa::StandstillFactorAdderParams params;
    params.enabled = true;
    params.huber_k = 1.345;
    params.add_velocity_prior = true;
    params.add_pose_between_factor = true;
    params.prior_velocity_stddev = 0.1;
    params.pose_between_factor_translation_stddev = 0.2;
    params.pose_between_factor_rotation_stddev = 0.3;
    return params;
  }

  // TODO(rsoussan): Get from common location, share with pose_node_adder test
  template <typename FactorPtrType>
  void EXPECT_SAME_NOISE(const FactorPtrType factor, const gtsam::Matrix& covariance) {
    EXPECT_MATRIX_NEAR(na::Covariance(factor->noiseModel()), covariance, 1e-6);
  }

  template <typename FactorPtrType>
  void EXPECT_SAME_NOISE(const FactorPtrType factor, const gtsam::SharedNoiseModel noise) {
    EXPECT_SAME_NOISE(factor, na::Covariance(noise));
  }

  void EXPECT_SAME_VELOCITY_PRIOR_FACTOR(const int factor_index, const int key_index) {
    const auto velocity_prior_factor =
      dynamic_cast<gtsam::PriorFactor<gtsam::Velocity3>*>(factors_[factor_index].get());
    ASSERT_TRUE(velocity_prior_factor);
    EXPECT_MATRIX_NEAR(velocity_prior_factor->prior(), Eigen::Vector3d::Zero(), 1e-6);
    EXPECT_EQ(velocity_prior_factor->key(), gtsam::Key(key_index));
    EXPECT_SAME_NOISE(velocity_prior_factor, zero_velocity_noise_);
  }

  void EXPECT_SAME_POSE_BETWEEN_FACTOR(const int factor_index, const int key_index) {
    const auto pose_between_factor = dynamic_cast<gtsam::BetweenFactor<gtsam::Pose3>*>(factors_[factor_index].get());
    ASSERT_TRUE(pose_between_factor);
    EXPECT_MATRIX_NEAR(pose_between_factor->measured(), Eigen::Isometry3d::Identity(), 1e-6);
    EXPECT_EQ(pose_between_factor->key1(), gtsam::Key(key_index));
    EXPECT_EQ(pose_between_factor->key2(), gtsam::Key((key_index + 2)));
    EXPECT_SAME_NOISE(pose_between_factor, zero_relative_pose_noise_);
  }

  lc::Time time(int index) { return measurements_[index].timestamp; }

  std::unique_ptr<fa::StandstillFactorAdder<SimplePoseVelocityNodeAdder>> factor_adder_;
  std::shared_ptr<SimplePoseVelocityNodeAdder> node_adder_;
  gtsam::NonlinearFactorGraph factors_;
  gtsam::SharedNoiseModel zero_velocity_noise_;
  gtsam::SharedNoiseModel zero_relative_pose_noise_;

 private:
  std::vector<lm::StandstillMeasurement> measurements_;
};

TEST_F(StandstillFactorAdderTest, PoseAndVelocityFactors) {
  auto params = DefaultParams();
  Initialize(params);
  AddMeasurements();
  // Add first factors
  EXPECT_EQ(factor_adder_->AddFactors(time(0), time(0), factors_), 2);
  EXPECT_EQ(factors_.size(), 2);
  // Keys and their indices:
  // pose_0: 0, velocity_0: 1
  // pose_1: 2, velocity_1: 3
  // Factors and their indices:
  // pose_between: 0, velocity_prior: 1
  EXPECT_SAME_POSE_BETWEEN_FACTOR(0, 0);
  // Use velocity_1 key since velocity prior is added to most recent timestamp
  // in standstill measurement
  EXPECT_SAME_VELOCITY_PRIOR_FACTOR(1, 3);
  // Add 2nd and 3rd factors
  EXPECT_EQ(factor_adder_->AddFactors((time(0) + time(1)) / 2.0, (time(2) + time(3)) / 2.0, factors_), 4);
  EXPECT_EQ(factors_.size(), 6);
  // Keys and their indices:
  // pose_0: 0, velocity_0: 1
  // pose_1: 2, velocity_1: 3
  // pose_2: 4, velocity_1: 5
  // pose_3: 6, velocity_1: 7
  // Factors and their indices:
  // pose_between: 0, velocity_prior: 1
  // pose_between: 2, velocity_prior: 3
  // pose_between: 4, velocity_prior: 5
  EXPECT_SAME_POSE_BETWEEN_FACTOR(0, 0);
  EXPECT_SAME_VELOCITY_PRIOR_FACTOR(1, 3);
  EXPECT_SAME_POSE_BETWEEN_FACTOR(2, 2);
  EXPECT_SAME_VELOCITY_PRIOR_FACTOR(3, 5);
  EXPECT_SAME_POSE_BETWEEN_FACTOR(4, 4);
  EXPECT_SAME_VELOCITY_PRIOR_FACTOR(5, 7);
}

TEST_F(StandstillFactorAdderTest, PoseOnlyFactors) {
  auto params = DefaultParams();
  params.add_velocity_prior = false;
  Initialize(params);
  AddMeasurements();
  // Add first factors
  EXPECT_EQ(factor_adder_->AddFactors(time(0), time(0), factors_), 1);
  EXPECT_EQ(factors_.size(), 1);
  // Keys and their indices:
  // pose_0: 0, velocity_0: 1
  // pose_1: 2, velocity_1: 3
  // Factors and their indices:
  // pose_between: 0
  EXPECT_SAME_POSE_BETWEEN_FACTOR(0, 0);
  // Add 2nd and 3rd factors
  EXPECT_EQ(factor_adder_->AddFactors((time(0) + time(1)) / 2.0, (time(2) + time(3)) / 2.0, factors_), 2);
  EXPECT_EQ(factors_.size(), 3);
  // Keys and their indices:
  // pose_0: 0, velocity_0: 1
  // pose_1: 2, velocity_1: 3
  // pose_2: 4, velocity_1: 5
  // pose_3: 6, velocity_1: 7
  // Factors and their indices:
  // pose_between: 0
  // pose_between: 1
  // pose_between: 2
  EXPECT_SAME_POSE_BETWEEN_FACTOR(0, 0);
  EXPECT_SAME_POSE_BETWEEN_FACTOR(1, 2);
  EXPECT_SAME_POSE_BETWEEN_FACTOR(2, 4);
}

TEST_F(StandstillFactorAdderTest, VelocityOnlyFactors) {
  auto params = DefaultParams();
  params.add_pose_between_factor = false;
  Initialize(params);
  AddMeasurements();
  // Add first factors
  EXPECT_EQ(factor_adder_->AddFactors(time(0), time(0), factors_), 1);
  EXPECT_EQ(factors_.size(), 1);
  // Keys and their indices:
  // pose_0: 0, velocity_0: 1
  // pose_1: 2, velocity_1: 3
  // Factors and their indices:
  // velocity_prior: 0
  // Use velocity_1 key since velocity prior is added to most recent timestamp
  // in standstill measurement
  EXPECT_SAME_VELOCITY_PRIOR_FACTOR(0, 3);
  // Add 2nd and 3rd factors
  EXPECT_EQ(factor_adder_->AddFactors((time(0) + time(1)) / 2.0, (time(2) + time(3)) / 2.0, factors_), 2);
  EXPECT_EQ(factors_.size(), 3);
  // Keys and their indices:
  // pose_0: 0, velocity_0: 1
  // pose_1: 2, velocity_1: 3
  // pose_2: 4, velocity_1: 5
  // pose_3: 6, velocity_1: 7
  // Factors and their indices:
  // velocity_prior: 0
  // velocity_prior: 1
  // velocity_prior: 2
  EXPECT_SAME_VELOCITY_PRIOR_FACTOR(0, 3);
  EXPECT_SAME_VELOCITY_PRIOR_FACTOR(1, 5);
  EXPECT_SAME_VELOCITY_PRIOR_FACTOR(2, 7);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
