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
#include <localization_measurements/imu_measurement.h>
#include <node_adders/combined_nav_state_node_adder_model.h>
#include <node_adders/utilities.h>

#include <gtest/gtest.h>

namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace no = nodes;
namespace na = node_adders;

class CombinedNavStateNodeAdderModelTest : public ::testing::Test {
 public:
  CombinedNavStateNodeAdderModelTest()
      : params_(na::DefaultCombinedNavStateNodeAdderModelParams()),
        model_(params_),
        nodes_(std::make_shared<no::Values>()),
        imu_integrator_(params_.imu_integrator) {}
  void SetUp() final {
    const Eigen::Vector3d acceleration(0.01, 0.02, 0.03);
    const Eigen::Vector3d angular_velocity(0.04, 0.05, 0.06);
    const Eigen::Vector3d acceleration_bias = 0.5 * acceleration;
    const Eigen::Vector3d angular_velocity_bias = 0.5 * angular_velocity;
    const Eigen::Vector3d bias_corrected_acceleration = acceleration - acceleration_bias;
    const Eigen::Vector3d bias_corrected_angular_velocity = angular_velocity - angular_velocity_bias;
    constexpr int kNumIterations = 10;
    constexpr double kTimeDiff = 1;
    lc::Time time = 0.0;
    for (int i = 0; i < kNumIterations; ++i) {
      const lm::ImuMeasurement imu_measurement(acceleration, angular_velocity, time);
      measurements_.emplace_back(imu_measurement);
      timestamps_.emplace_back(time);
      time += kTimeDiff;
    }
  }

  void AddMeasurements() {
    for (const auto& measurement : measurements_) {
      model_.AddMeasurement(measurement);
      imu_integrator_.AddImuMeasurement(measurement);
    }
  }

  std::vector<gtsam::SharedNoiseModel> Noise() {
    constexpr double kTranslationStddev = 0.1;
    constexpr double kQuaternionStddev = 0.2;
    const gtsam::Vector6 pose_prior_noise_sigmas((gtsam::Vector(6) << kTranslationStddev, kTranslationStddev,
                                                  kTranslationStddev, kQuaternionStddev, kQuaternionStddev,
                                                  kQuaternionStddev)
                                                   .finished());
    constexpr double kVelocityStddev = 0.3;
    const gtsam::Vector3 velocity_prior_noise_sigmas(
      (gtsam::Vector(3) << kVelocityStddev, kVelocityStddev, kVelocityStddev).finished());
    constexpr double kAccelBiasStddev = 0.4;
    constexpr double kGyroBiasStddev = 0.5;
    const gtsam::Vector6 bias_prior_noise_sigmas((gtsam::Vector(6) << kAccelBiasStddev, kAccelBiasStddev,
                                                  kAccelBiasStddev, kGyroBiasStddev, kGyroBiasStddev, kGyroBiasStddev)
                                                   .finished());
    const auto pose_noise = lc::Robust(
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_prior_noise_sigmas)), params_.huber_k);
    const auto velocity_noise =
      lc::Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)),
                 params_.huber_k);
    const auto bias_noise = lc::Robust(
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(bias_prior_noise_sigmas)), params_.huber_k);
    std::vector<gtsam::SharedNoiseModel> noise;
    noise.emplace_back(pose_noise);
    noise.emplace_back(velocity_noise);
    noise.emplace_back(bias_noise);
    return noise;
  }

  const lm::ImuMeasurement& measurement(const int index) { return measurements_[index]; }
  lc::Time time(const int index) { return timestamps_[index]; }

  std::vector<lm::ImuMeasurement> measurements_;
  std::vector<lc::Time> timestamps_;
  na::CombinedNavStateNodeAdderModelParams params_;
  na::CombinedNavStateNodeAdderModel model_;
  no::CombinedNavStateNodes nodes_;
  gtsam::NonlinearFactorGraph factors_;
  ii::ImuIntegrator imu_integrator_;
};

TEST_F(CombinedNavStateNodeAdderModelTest, AddRemoveCanAddNode) {
  EXPECT_FALSE(model_.CanAddNode(time(0)));
  // Add 1st measurement
  model_.AddMeasurement(measurement(time(0)));
  EXPECT_TRUE(model_.CanAddNode(time(0)));
  EXPECT_FALSE(model_.CanAddNode(time(0) - 0.1));
  EXPECT_FALSE(model_.CanAddNode(time(1)));
  EXPECT_FALSE(model_.CanAddNode(time(5)));
  // Add 2nd measurement
  model_.AddMeasurement(measurement(1));
  EXPECT_TRUE(model_.CanAddNode(time(0)));
  EXPECT_FALSE(model_.CanAddNode(time(0) - 0.1));
  EXPECT_TRUE(model_.CanAddNode((time(0) + time(1)) / 2.0));
  EXPECT_TRUE(model_.CanAddNode(time(1)));
  EXPECT_FALSE(model_.CanAddNode(time(2)));
  // Add 3rd measurement
  model_.AddMeasurement(measurement(2));
  EXPECT_TRUE(model_.CanAddNode(time(0)));
  EXPECT_FALSE(model_.CanAddNode(time(0) - 0.1));
  EXPECT_TRUE(model_.CanAddNode((time(0) + time(1)) / 2.0));
  EXPECT_TRUE(model_.CanAddNode(time(2)));
  // Remove up to 2nd measurement
  model_.RemoveMeasurements(time(1));
  EXPECT_FALSE(model_.CanAddNode(time(0)));
  EXPECT_FALSE(model_.CanAddNode((time(0) + time(1)) / 2.0));
  EXPECT_TRUE(model_.CanAddNode(time(1)));
  EXPECT_TRUE(model_.CanAddNode(time(2)));
}

TEST_F(CombinedNavStateNodeAdderModelTest, AddPriors) {
  const auto node = lc::RandomCombinedNavState();
  const auto keys = nodes_.Add(node.timestamp(), node);
  ASSERT_FALSE(keys.empty());
  const auto noise = Noise();
  model_.AddPriors(node, noise, node.timestamp(), nodes_, factors_);
  ASSERT_EQ(factors_.size(), 3);
  // Check pose prior
  const auto pose_priors = lc::Factors<gtsam::PriorFactor<gtsam::Pose3>>(factors_);
  EXPECT_EQ(pose_priors.size(), 1);
  EXPECT_MATRIX_NEAR(pose_priors[0]->prior(), node.pose(), 1e-6);
  EXPECT_MATRIX_NEAR(na::Covariance(pose_priors[0]->noiseModel()), na::Covariance(noise[0]), 1e-6);
  // Check velocity prior
  const auto velocity_priors = lc::Factors<gtsam::PriorFactor<gtsam::Velocity3>>(factors_);
  EXPECT_EQ(velocity_priors.size(), 1);
  EXPECT_MATRIX_NEAR(velocity_priors[0]->prior(), node.velocity(), 1e-6);
  EXPECT_MATRIX_NEAR(na::Covariance(velocity_priors[0]->noiseModel()), na::Covariance(noise[1]), 1e-6);
  // Check Bias prior
  const auto bias_priors = lc::Factors<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(factors_);
  EXPECT_EQ(bias_priors.size(), 1);
  EXPECT_MATRIX_NEAR(bias_priors[0]->prior().accelerometer(), node.bias().accelerometer(), 1e-6);
  EXPECT_MATRIX_NEAR(bias_priors[0]->prior().gyroscope(), node.bias().gyroscope(), 1e-6);
  EXPECT_MATRIX_NEAR(na::Covariance(bias_priors[0]->noiseModel()), na::Covariance(noise[2]), 1e-6);
}

TEST_F(CombinedNavStateNodeAdderModelTest, AddRelativeFactors) {
  const auto timestamp_a = timestamps_[2];
  const lc::CombinedNavState node_a(
    lc::CombinedNavState(lc::RandomPose(), lc::RandomVelocity(),
                         gtsam::imuBias::ConstantBias(lc::RandomVector3d(), lc::RandomVector3d()), timestamp_a));
  const auto timestamp_b = timestamps_[5];
  const lc::CombinedNavState node_b(
    lc::CombinedNavState(lc::RandomPose(), lc::RandomVelocity(),
                         gtsam::imuBias::ConstantBias(lc::RandomVector3d(), lc::RandomVector3d()), timestamp_b));

  const auto keys_a = nodes_.Add(node_a.timestamp(), node_a);
  ASSERT_FALSE(keys_a.empty());
  const auto keys_b = nodes_.Add(node_b.timestamp(), node_b);
  ASSERT_FALSE(keys_b.empty());

  AddMeasurements();

  // Add relative factor
  ASSERT_FALSE(model_.AddRelativeFactors(timestamps_[0], timestamp_b, nodes_, factors_));
  ASSERT_FALSE(model_.AddRelativeFactors(timestamp_a, timestamps_[3], nodes_, factors_));
  ASSERT_TRUE(model_.AddRelativeFactors(timestamp_a, timestamp_b, nodes_, factors_));
  ASSERT_EQ(factors_.size(), 1);
  const auto imu_factors = lc::Factors<gtsam::CombinedImuFactor>(factors_);
  ASSERT_EQ(imu_factors.size(), 1);
  // Check keys
  // pose_a
  EXPECT_EQ(imu_factors[0]->key1(), keys_a[0]);
  // vel_a
  EXPECT_EQ(imu_factors[0]->key2(), keys_a[1]);
  // pose_b
  EXPECT_EQ(imu_factors[0]->key3(), keys_b[0]);
  // vel_b
  EXPECT_EQ(imu_factors[0]->key4(), keys_b[1]);
  // bias_a
  EXPECT_EQ(imu_factors[0]->key5(), keys_a[2]);
  // bias_b
  EXPECT_EQ(imu_factors[0]->key6(), keys_b[2]);

  // Check PIM
  const auto pim = imu_integrator_.IntegratedPim(node_a.bias(), timestamp_a, timestamp_b);
  ASSERT_TRUE(pim != boost::none);
  EXPECT_TRUE(pim->equals(imu_factors[0]->preintegratedMeasurements()));
}

TEST_F(CombinedNavStateNodeAdderModelTest, AddRelativeFactorsInBetweenTimes) {
  const auto timestamp_a = timestamps_[2];
  const lc::CombinedNavState node_a(
    lc::CombinedNavState(lc::RandomPose(), lc::RandomVelocity(),
                         gtsam::imuBias::ConstantBias(lc::RandomVector3d(), lc::RandomVector3d()), timestamp_a));
  const auto timestamp_b = (timestamps_[5] + timestamps_[6]) / 2.0;
  const lc::CombinedNavState node_b(
    lc::CombinedNavState(lc::RandomPose(), lc::RandomVelocity(),
                         gtsam::imuBias::ConstantBias(lc::RandomVector3d(), lc::RandomVector3d()), timestamp_b));

  const auto keys_a = nodes_.Add(node_a.timestamp(), node_a);
  ASSERT_FALSE(keys_a.empty());
  const auto keys_b = nodes_.Add(node_b.timestamp(), node_b);
  ASSERT_FALSE(keys_b.empty());

  AddMeasurements();

  // Add relative factor
  ASSERT_FALSE(model_.AddRelativeFactors(timestamps_[0], timestamp_b, nodes_, factors_));
  ASSERT_FALSE(model_.AddRelativeFactors(timestamp_a, timestamps_[3], nodes_, factors_));
  ASSERT_TRUE(model_.AddRelativeFactors(timestamp_a, timestamp_b, nodes_, factors_));
  ASSERT_EQ(factors_.size(), 1);
  const auto imu_factors = lc::Factors<gtsam::CombinedImuFactor>(factors_);
  ASSERT_EQ(imu_factors.size(), 1);
  // Check keys
  // pose_a
  EXPECT_EQ(imu_factors[0]->key1(), keys_a[0]);
  // vel_a
  EXPECT_EQ(imu_factors[0]->key2(), keys_a[1]);
  // pose_b
  EXPECT_EQ(imu_factors[0]->key3(), keys_b[0]);
  // vel_b
  EXPECT_EQ(imu_factors[0]->key4(), keys_b[1]);
  // bias_a
  EXPECT_EQ(imu_factors[0]->key5(), keys_a[2]);
  // bias_b
  EXPECT_EQ(imu_factors[0]->key6(), keys_b[2]);

  // Check PIM
  const auto pim = imu_integrator_.IntegratedPim(node_a.bias(), timestamp_a, timestamp_b);
  ASSERT_TRUE(pim != boost::none);
  EXPECT_TRUE(pim->equals(imu_factors[0]->preintegratedMeasurements()));
}

TEST_F(CombinedNavStateNodeAdderModelTest, RemoveRelativeFactors) {
  AddMeasurements();
  const auto timestamp_a = timestamps_[3];
  const auto timestamp_b = timestamps_[7];
  const auto timestamp_c = timestamps_[8];
  // Add first nodes and relative factor
  const lc::CombinedNavState node_a(
    lc::CombinedNavState(lc::RandomPose(), lc::RandomVelocity(),
                         gtsam::imuBias::ConstantBias(lc::RandomVector3d(), lc::RandomVector3d()), timestamp_a));
  const lc::CombinedNavState node_b(
    lc::CombinedNavState(lc::RandomPose(), lc::RandomVelocity(),
                         gtsam::imuBias::ConstantBias(lc::RandomVector3d(), lc::RandomVector3d()), timestamp_b));

  const auto keys_a = nodes_.Add(node_a.timestamp(), node_a);
  ASSERT_FALSE(keys_a.empty());
  const auto keys_b = nodes_.Add(node_b.timestamp(), node_b);
  ASSERT_FALSE(keys_b.empty());
  ASSERT_TRUE(model_.AddRelativeFactors(timestamp_a, timestamp_b, nodes_, factors_));
  // Add second nodes and relative factor
  const lc::CombinedNavState node_c(
    lc::CombinedNavState(lc::RandomPose(), lc::RandomVelocity(),
                         gtsam::imuBias::ConstantBias(lc::RandomVector3d(), lc::RandomVector3d()), timestamp_c));
  const auto keys_c = nodes_.Add(node_c.timestamp(), node_c);
  ASSERT_FALSE(keys_c.empty());
  ASSERT_TRUE(model_.AddRelativeFactors(timestamp_b, timestamp_c, nodes_, factors_));

  ASSERT_EQ(factors_.size(), 2);
  ASSERT_FALSE(model_.RemoveRelativeFactors(timestamp_a - 1.0, timestamp_b, nodes_, factors_));
  ASSERT_FALSE(model_.RemoveRelativeFactors(timestamp_a, timestamp_c, nodes_, factors_));
  ASSERT_TRUE(model_.RemoveRelativeFactors(timestamp_a, timestamp_b, nodes_, factors_));
  // Check the remaining factor is the second relative factor
  const auto imu_factors = lc::Factors<gtsam::CombinedImuFactor>(factors_);
  ASSERT_EQ(imu_factors.size(), 1);
  // Check keys
  // pose_b
  EXPECT_EQ(imu_factors[0]->key1(), keys_b[0]);
  // vel_b
  EXPECT_EQ(imu_factors[0]->key2(), keys_b[1]);
  // pose_c
  EXPECT_EQ(imu_factors[0]->key3(), keys_c[0]);
  // vel_c
  EXPECT_EQ(imu_factors[0]->key4(), keys_c[1]);
  // bias_b
  EXPECT_EQ(imu_factors[0]->key5(), keys_b[2]);
  // bias_c
  EXPECT_EQ(imu_factors[0]->key6(), keys_c[2]);
  // Remove last factor
  ASSERT_FALSE(model_.RemoveRelativeFactors(timestamp_a, timestamp_b, nodes_, factors_));
  ASSERT_TRUE(model_.RemoveRelativeFactors(timestamp_b, timestamp_c, nodes_, factors_));
  ASSERT_TRUE(factors_.empty());
}

TEST_F(CombinedNavStateNodeAdderModelTest, AddNodeAndRelativeFactors) {
  AddMeasurements();
  const auto timestamp_a = timestamps_[1];
  const auto timestamp_b = timestamps_[4];
  // Should fail since no node at timestamp_a exists yet
  EXPECT_FALSE(model_.AddNodesAndRelativeFactors(timestamp_a, timestamp_b, nodes_, factors_));
  const lc::CombinedNavState node_a(
    lc::CombinedNavState(lc::RandomPose(), lc::RandomVelocity(),
                         gtsam::imuBias::ConstantBias(lc::RandomVector3d(), lc::RandomVector3d()), timestamp_a));
  const auto keys_a = nodes_.Add(node_a.timestamp(), node_a);
  ASSERT_FALSE(keys_a.empty());
  EXPECT_TRUE(model_.AddNodesAndRelativeFactors(timestamp_a, timestamp_b, nodes_, factors_));
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_TRUE(nodes_.Contains(timestamp_a));
  EXPECT_TRUE(nodes_.Contains(timestamp_b));
  const auto imu_factors = lc::Factors<gtsam::CombinedImuFactor>(factors_);
  ASSERT_EQ(imu_factors.size(), 1);
  // Check keys
  const auto keys_b = nodes_.Keys(timestamp_b);
  ASSERT_FALSE(keys_b.empty());
  // pose_a
  EXPECT_EQ(imu_factors[0]->key1(), keys_a[0]);
  // vel_a
  EXPECT_EQ(imu_factors[0]->key2(), keys_a[1]);
  // pose_b
  EXPECT_EQ(imu_factors[0]->key3(), keys_b[0]);
  // vel_b
  EXPECT_EQ(imu_factors[0]->key4(), keys_b[1]);
  // bias_a
  EXPECT_EQ(imu_factors[0]->key5(), keys_a[2]);
  // bias_b
  EXPECT_EQ(imu_factors[0]->key6(), keys_b[2]);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
