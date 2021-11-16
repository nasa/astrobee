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
#include <imu_augmentor/imu_augmentor.h>
#include <localization_common/logger.h>
#include <localization_common/time.h>

#include <gtest/gtest.h>

namespace ia = imu_augmentor;
namespace lc = localization_common;
namespace lm = localization_measurements;
TEST(IMUAugmentorTester, PimPredictConstantAcceleration) {
  const ia::ImuAugmentorParams params = ia::DefaultImuAugmentorParams();
  ia::ImuAugmentor imu_augmentor(params);
  const double acceleration_i = 0.01;
  const Eigen::Vector3d acceleration(acceleration_i, acceleration_i, acceleration_i);
  constexpr double time_increment = 1 / 125.0;
  // Start at time increment so first IMU measurement is after starting combined nav state time
  const lc::Time start_time = time_increment;
  constexpr int num_measurements = 20;
  const std::vector<lm::ImuMeasurement> imu_measurements =
    ia::ConstantAccelerationMeasurements(acceleration, num_measurements, start_time, time_increment);
  for (const auto& imu_measurement : imu_measurements) {
    imu_augmentor.BufferImuMeasurement(imu_measurement);
  }
  const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(), 0);
  lc::CombinedNavState imu_augmented_state = initial_state;

  imu_augmentor.PimPredict(initial_state, imu_augmented_state);
  EXPECT_NEAR(imu_augmented_state.timestamp(), num_measurements * time_increment, 1e-6);
  const double expected_velocity_i = acceleration_i * num_measurements;
  const gtsam::Vector3 expected_velocity(expected_velocity_i, expected_velocity_i, expected_velocity_i);
  // TODO(rsoussan): Replace this with assert pred2 with eigen comparisson when other pr merged
  EXPECT_TRUE(imu_augmented_state.velocity().matrix().isApprox(expected_velocity.matrix(), 1e-6));
  // make sure position and velocity are correct!! (AC)
}

// Test imu integration accuracy!
// pass accel only data
// ensure resulting position and velocity are correct!

// pass omega only data
// ensure resulting orientation is correct!

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
