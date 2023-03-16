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
  gtsam::KeyVector Keys(const localization_common::Time timestamp) const final {
    gtsam::KeyVector keys;
    keys.emplace_back(gtsam::Key(static_cast<int>(timestamp) * 2));
    keys.emplace_back(gtsam::Key(static_cast<int>(timestamp) * 2 + 1));
    return keys;
  }

  std::string type() const final { return "simple_pose_velocity_node_adder"; }

 private:
  gtsam::KeyVector pose_keys_;
  gtsam::KeyVector velocity_keys_;
  int key_index_ = 0;
};

class StandstillFactorAdderTest : public ::testing::Test {
 public:
  StandstillFactorAdderTest() { node_adder_.reset(new SimplePoseVelocityNodeAdder()); }

  void SetUp() final {}

  void AddMeasurements() {
    constexpr int kNumMeasurements = 10;
    for (int i = 0; i < kNumMeasurements; ++i) {
      const lm::StandstillMeasurement measurement(i, i - 1);
      measurements_.emplace_back(measurement);
      factor_adder_->AddMeasurement(measurement);
    }
  }

  void Initialize(const fa::StandstillFactorAdderParams& params) {
    factor_adder_.reset(new fa::StandstillFactorAdder<SimplePoseVelocityNodeAdder>(params, node_adder_));
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

  void EXPECT_SAME_VELOCITY_PRIOR_FACTOR(const int factor_index, const int key_index) {
    const auto velocity_prior_factor =
      dynamic_cast<gtsam::PriorFactor<gtsam::Velocity3>*>(factors_[factor_index].get());
    ASSERT_TRUE(velocity_prior_factor);
    EXPECT_MATRIX_NEAR(velocity_prior_factor->prior(), Eigen::Vector3d::Zero(), 1e-6);
    EXPECT_EQ(velocity_prior_factor->key(), velocity_key(key_index));
  }

  void EXPECT_SAME_POSE_BETWEEN_FACTOR(const int factor_index, const int key_index) {
    const auto pose_between_factor = dynamic_cast<gtsam::BetweenFactor<gtsam::Pose3>*>(factors_[factor_index].get());
    ASSERT_TRUE(pose_between_factor);
    EXPECT_MATRIX_NEAR(pose_between_factor->measured(), Eigen::Isometry3d::Identity(), 1e-6);
    EXPECT_EQ(pose_between_factor->key1(), pose_key(key_index));
    EXPECT_EQ(pose_between_factor->key2(), pose_key(key_index + 1));
  }

  lc::Time time(int index) { return measurements_[index].timestamp; }

  // Velocity key is created after pose key
  gtsam::Key velocity_key(int index) { return gtsam::Key(index * 2 + 1); }

  gtsam::Key pose_key(int index) { return gtsam::Key(index * 2); }

  std::unique_ptr<fa::StandstillFactorAdder<SimplePoseVelocityNodeAdder>> factor_adder_;
  std::shared_ptr<SimplePoseVelocityNodeAdder> node_adder_;
  gtsam::NonlinearFactorGraph factors_;

 private:
  std::vector<lm::StandstillMeasurement> measurements_;
};

TEST_F(StandstillFactorAdderTest, PoseAndVelocityFactors) {
  auto params = DefaultParams();
  Initialize(params);
  AddMeasurements();
  // Add first factors
  EXPECT_EQ(factor_adder_->AddFactors(time(0), time(0), factors_), 2);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
