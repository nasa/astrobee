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

#include <factor_adders/single_measurement_based_factor_adder.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/pose_measurement.h>

#include <gtest/gtest.h>

namespace fa = factor_adders;
namespace lc = localization_common;
namespace lm = localization_measurements;

struct SimpleAdderParams : public fa::FactorAdderParams {
  SimpleAdderParams() {
    huber_k = 1.345;
    enabled = true;
  }
};

class SimpleAdder : public fa::SingleMeasurementBasedFactorAdder<lm::PoseMeasurement> {
 public:
  explicit SimpleAdder(const SimpleAdderParams& params = SimpleAdderParams())
      : fa::SingleMeasurementBasedFactorAdder<lm::PoseMeasurement>(params), params_(params) {
    // Create pose noise
    const double translation_stddev = 0.1;
    const double rotation_stddev = 0.2;
    const gtsam::Vector6 pose_noise_sigmas((gtsam::Vector(6) << rotation_stddev, rotation_stddev, rotation_stddev,
                                            translation_stddev, translation_stddev, translation_stddev)
                                             .finished());
    pose_noise_ = lc::Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_noise_sigmas)),
                             params_.huber_k);
  }

  int end_time_ = 10;

 private:
  int AddFactorsForSingleMeasurement(const lm::PoseMeasurement& measurement,
                                     gtsam::NonlinearFactorGraph& factors) final {
    gtsam::PriorFactor<gtsam::Pose3>::shared_ptr pose_prior_factor(
      new gtsam::PriorFactor<gtsam::Pose3>(gtsam::Key(key_value_++), measurement.pose, pose_noise_));
    factors.push_back(pose_prior_factor);
    return 1;
  }

  // Simulate some node adder delay and don't add factors if past end time.
  bool CanAddFactor(const lm::PoseMeasurement& measurement) const final { return measurement.timestamp <= end_time_; }

  int key_value_ = 0;
  gtsam::SharedNoiseModel pose_noise_;
  SimpleAdderParams params_;
};

class SingleMeasurementBasedFactorAdderTest : public ::testing::Test {
 public:
  SingleMeasurementBasedFactorAdderTest() {}

  void SetUp() final {
    constexpr int kNumMeasurements = 10;
    for (int i = 0; i < kNumMeasurements; ++i) {
      const lm::PoseMeasurement measurement(lc::RandomPose(), lc::Time(i));
      measurements_.emplace_back(measurement);
    }
  }

  void AddMeasurements() {
    for (const auto& measurement : measurements_) factor_adder_.AddMeasurement(measurement);
  }

  // TODO(rsoussan): Unify this with pose_node_adder test!
  void EXPECT_SAME_PRIOR_FACTOR(const int index, const Eigen::Isometry3d& pose) {
    const auto pose_prior_factor = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(factors_[index].get());
    ASSERT_TRUE(pose_prior_factor);
    EXPECT_MATRIX_NEAR(pose_prior_factor->prior(), pose, 1e-6);
  }

  void EXPECT_SAME_PRIOR_FACTOR(const int index, const gtsam::Pose3& pose) {
    EXPECT_SAME_PRIOR_FACTOR(index, lc::EigenPose(pose));
  }

  void EXPECT_SAME_PRIOR_FACTOR(const int index) { EXPECT_SAME_PRIOR_FACTOR(index, pose(index)); }

  void EXPECT_SAME_PRIOR_FACTOR(const int factor_index, const int pose_index) {
    EXPECT_SAME_PRIOR_FACTOR(factor_index, pose(pose_index));
  }

  lc::Time time(int index) { return measurements_[index].timestamp; }

  const gtsam::Pose3& pose(const int index) { return measurements_[index].pose; }

  SimpleAdder factor_adder_;
  std::vector<lm::PoseMeasurement> measurements_;
  gtsam::NonlinearFactorGraph factors_;

 private:
};

TEST_F(SingleMeasurementBasedFactorAdderTest, AddMeasurements) {
  AddMeasurements();
  EXPECT_EQ(factor_adder_.AddFactors(-100, -10, factors_), 0);
  EXPECT_EQ(factors_.size(), 0);
  // Add factor 0
  EXPECT_EQ(factor_adder_.AddFactors(-100.1, time(0), factors_), 1);
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_SAME_PRIOR_FACTOR(0);
  // Add factors 1, 2, 3
  EXPECT_EQ(factor_adder_.AddFactors(time(1) - 0.1, time(3) + 0.1, factors_), 3);
  EXPECT_EQ(factors_.size(), 4);
  EXPECT_SAME_PRIOR_FACTOR(0);
  EXPECT_SAME_PRIOR_FACTOR(1);
  EXPECT_SAME_PRIOR_FACTOR(2);
  EXPECT_SAME_PRIOR_FACTOR(3);
  // Add factors 1, 2, 3 again, should have no affect
  EXPECT_EQ(factor_adder_.AddFactors(time(1) - 0.1, time(3) + 0.1, factors_), 0);
  EXPECT_EQ(factors_.size(), 4);
  EXPECT_SAME_PRIOR_FACTOR(0);
  EXPECT_SAME_PRIOR_FACTOR(1);
  EXPECT_SAME_PRIOR_FACTOR(2);
  EXPECT_SAME_PRIOR_FACTOR(3);
  // Add factors 4, 5
  EXPECT_EQ(factor_adder_.AddFactors(time(4) - 0.1, time(5), factors_), 2);
  EXPECT_EQ(factors_.size(), 6);
  EXPECT_SAME_PRIOR_FACTOR(0);
  EXPECT_SAME_PRIOR_FACTOR(1);
  EXPECT_SAME_PRIOR_FACTOR(2);
  EXPECT_SAME_PRIOR_FACTOR(3);
  EXPECT_SAME_PRIOR_FACTOR(4);
  EXPECT_SAME_PRIOR_FACTOR(5);
  // Old and used measurements should be removed up, expect adding factors 0,1,2 to fail
  EXPECT_EQ(factor_adder_.AddFactors(time(0) - 0.1, time(2), factors_), 0);
  EXPECT_EQ(factors_.size(), 6);
  EXPECT_SAME_PRIOR_FACTOR(0);
  EXPECT_SAME_PRIOR_FACTOR(1);
  EXPECT_SAME_PRIOR_FACTOR(2);
  EXPECT_SAME_PRIOR_FACTOR(3);
  EXPECT_SAME_PRIOR_FACTOR(4);
  EXPECT_SAME_PRIOR_FACTOR(5);
  // Add factors 8, 9
  EXPECT_EQ(factor_adder_.AddFactors(time(8) - 0.1, time(9), factors_), 2);
  EXPECT_EQ(factors_.size(), 8);
  EXPECT_SAME_PRIOR_FACTOR(0);
  EXPECT_SAME_PRIOR_FACTOR(1);
  EXPECT_SAME_PRIOR_FACTOR(2);
  EXPECT_SAME_PRIOR_FACTOR(3);
  EXPECT_SAME_PRIOR_FACTOR(4);
  EXPECT_SAME_PRIOR_FACTOR(5);
  EXPECT_SAME_PRIOR_FACTOR(6, 8);
  EXPECT_SAME_PRIOR_FACTOR(7, 9);
  // Add factors 6,7 - should fail since already used an oldest allowed time (8) newer
  // than each of these and therefore measurements were removed
  EXPECT_EQ(factor_adder_.AddFactors(time(6) - 0.1, time(7), factors_), 0);
  EXPECT_EQ(factors_.size(), 8);
  EXPECT_SAME_PRIOR_FACTOR(0);
  EXPECT_SAME_PRIOR_FACTOR(1);
  EXPECT_SAME_PRIOR_FACTOR(2);
  EXPECT_SAME_PRIOR_FACTOR(3);
  EXPECT_SAME_PRIOR_FACTOR(4);
  EXPECT_SAME_PRIOR_FACTOR(5);
  EXPECT_SAME_PRIOR_FACTOR(6, 8);
  EXPECT_SAME_PRIOR_FACTOR(7, 9);
  // Can't add factors newer than measurements
  EXPECT_EQ(factor_adder_.AddFactors(time(9) + 0.1, time(9) + 1000.1, factors_), 0);
  EXPECT_EQ(factors_.size(), 8);
}

TEST_F(SingleMeasurementBasedFactorAdderTest, AddFactorsTooSoon) {
  AddMeasurements();
  // Add factor 0
  EXPECT_EQ(factor_adder_.AddFactors(-100.1, time(0), factors_), 1);
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_SAME_PRIOR_FACTOR(0);
  // Make factors unaddable, make sure none are added
  factor_adder_.end_time_ = 0.5;
  EXPECT_EQ(factor_adder_.AddFactors(time(1), time(3), factors_), 0);
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_SAME_PRIOR_FACTOR(0);
  // Increase end time, measurements shouldn't have been removed and
  // now factors should be added.
  factor_adder_.end_time_ = 10;
  EXPECT_EQ(factor_adder_.AddFactors(time(1), time(3), factors_), 3);
  EXPECT_EQ(factors_.size(), 4);
  EXPECT_SAME_PRIOR_FACTOR(0);
  EXPECT_SAME_PRIOR_FACTOR(1);
  EXPECT_SAME_PRIOR_FACTOR(2);
  EXPECT_SAME_PRIOR_FACTOR(3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
