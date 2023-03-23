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
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/matched_projections_measurement.h>
#include <node_adders/node_adder.h>
#include <node_adders/utilities.h>

#include <gtest/gtest.h>

namespace fa = factor_adders;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace na = node_adders;

// Test node adder that just returns keys that should be used.
// Key values are calculated using the integer timestamps passed.
class SimplePoseNodeAdder : public na::NodeAdder {
 public:
  void AddInitialNodesAndPriors(gtsam::NonlinearFactorGraph& graph) final{};

  bool AddNode(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final { return true; }

  bool CanAddNode(const localization_common::Time timestamp) const final { return true; }

  // Assumes integer timestamps that perfectly cast to ints.
  // First key is pose key.
  gtsam::KeyVector Keys(const localization_common::Time timestamp) const final {
    gtsam::KeyVector keys;
    keys.emplace_back(gtsam::Key(static_cast<int>(timestamp)));
    return keys;
  }

  const no::Nodes& nodes() const { return nodes_; }

  no::Nodes& nodes() { return nodes_; }

  std::string type() const final { return "simple_pose_node_adder"; }

 private:
  no::Nodes nodes_;
};

class LocFactorAdderTest : public ::testing::Test {
 public:
  LocFactorAdderTest() { node_adder_.reset(new SimplePoseNodeAdder()); }

  void SetUp() final {}

  void AddMeasurements() {
    constexpr int kNumMeasurements = 10;
    for (int i = 0; i < kNumMeasurements; ++i) {
      const lm::MatchedProjectionsMeasurement measurement(i + 1, i);
      measurements_.emplace_back(measurement);
      factor_adder_->AddMeasurement(measurement);
    }
  }

  void Initialize(const fa::LocFactorAdderParams& params) {
    factor_adder_.reset(new fa::LocFactorAdder<SimplePoseNodeAdder>(params, node_adder_));
  }

  fa::LocFactorAdderParams DefaultParams() {
    fa::LocFactorAdderParams params;
    params.enabled = true;
    params.huber_k = 1.345;
    params.add_velocity_prior = true;
    params.add_pose_between_factor = true;
    params.prior_velocity_stddev = 0.1;
    params.pose_between_factor_translation_stddev = 0.2;
    params.pose_between_factor_rotation_stddev = 0.3;
    return params;
  }

  std::unique_ptr<fa::LocFactorAdder<SimplePoseNodeAdder>> factor_adder_;
  std::shared_ptr<SimplePoseNodeAdder> node_adder_;
  gtsam::NonlinearFactorGraph factors_;

 private:
  std::vector<lm::MatchedProjectionsMeasurement> measurements_;
};

TEST_F(LocFactorAdderTest, PoseAndVelocityFactors) {
  auto params = DefaultParams();
  Initialize(params);
  AddMeasurements();
/*  // Add first factors
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
  EXPECT_SAME_VELOCITY_PRIOR_FACTOR(5, 7);*/
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
