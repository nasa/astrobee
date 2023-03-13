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
#include <node_updaters/combined_nav_state_node_update_model.h>
#include <node_updaters/utilities.h>

#include <gtest/gtest.h>

namespace go = graph_optimizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace nu = node_updaters;

class CombinedNavStateNodeUpdateModelTest : public ::testing::Test {
 public:
  CombinedNavStateNodeUpdateModelTest()
      : model_(nu::DefaultCombinedNavStateNodeUpdateModelParams()) {}
  void SetUp() final {
  const Eigen::Vector3d acceleration(0.01, 0.02, 0.03);
  const Eigen::Vector3d angular_velocity(0.04, 0.05, 0.06);
  const Eigen::Vector3d acceleration_bias = 0.5 * acceleration;
  const Eigen::Vector3d angular_velocity_bias = 0.5 * angular_velocity;
  const Eigen::Vector3d bias_corrected_acceleration = acceleration - acceleration_bias;
  const Eigen::Vector3d bias_corrected_angular_velocity = angular_velocity - angular_velocity_bias;
//  params.graph_initializer.initial_imu_bias = gtsam::imuBias::ConstantBias(acceleration_bias, angular_velocity_bias);
  // Use depth odometry factor adder since it can add relative pose factors
  constexpr int kNumIterations = 10;
  constexpr double kTimeDiff = 1;
  lc::Time time = 0.0;
  Eigen::Isometry3d current_pose(Eigen::Isometry3d::Identity());
  Eigen::Vector3d velocity(Eigen::Vector3d::Zero());
  // Add initial zero imu value so the imu integrator has more than one measurement when the subsequent
  // measurement is added
  // TODO(rsoussan): is this needed?
  // const lm::ImuMeasurement zero_imu_measurement(acceleration_bias, angular_velocity_bias, time);
  // measurements_.emplace_back(zero_imu_measurement);
  for (int i = 0; i < kNumIterations ; ++i) {
    const lm::ImuMeasurement imu_measurement(acceleration, angular_velocity, time);
    measurements_.emplace_back(imu_measurement);
    timestamps_.emplace_back(time);
    poses_.emplace_back(current_pose);
    velocities_.emplace_back(velocity);
    // Update values for next iteration
    time += kTimeDiff;
    const Eigen::Matrix3d relative_orientation =
      (gtsam::Rot3::Expmap(bias_corrected_angular_velocity * kTimeDiff)).matrix();
    const Eigen::Vector3d relative_translation =
      velocity * kTimeDiff + 0.5 * bias_corrected_acceleration * kTimeDiff * kTimeDiff;
    velocity += bias_corrected_acceleration * kTimeDiff;
    // Put velocity in new body frame after integrating accelerations
    velocity = relative_orientation.transpose() * velocity;
    const Eigen::Isometry3d relative_pose = lc::Isometry3d(relative_translation, relative_orientation);
    current_pose = current_pose * relative_pose;
  }
}

const lm::ImuMeasurement& measurement(const int index) { return measurements_[index]; }
lc::Time time(const int index) { return timestamps_[index]; }
const Eigen::Isometry3d& pose(const int index) { return poses_[index]; }
const Eigen::Vector3d& velocity(const int index) { return velocities_[index]; }

std::vector<lm::ImuMeasurement> measurements_;
std::vector<lc::Time> timestamps_;
std::vector<Eigen::Isometry3d> poses_;
std::vector<Eigen::Vector3d> velocities_;
nu::CombinedNavStateNodeUpdateModel model_;
gtsam::NonlinearFactorGraph factors_;
};

TEST_F(CombinedNavStateNodeUpdateModelTest, AddRemoveCanAddNode) {
  EXPECT_FALSE(model_.CanAddNode(0));
  model_.AddMeasurement(measurement(0));
  EXPECT_TRUE(model_.CanAddNode(0));
  model_.AddMeasurement(measurement(1));
  EXPECT_TRUE(model_.CanAddNode(0.5));
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
