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
#include <imu_augmentor/imu_augmentor_wrapper.h>
#include <localization_common/logger.h>
#include <localization_common/time.h>
#include <localization_common/utilities.h>

#include <gtest/gtest.h>

namespace ia = imu_augmentor;
namespace lc = localization_common;
namespace lm = localization_measurements;

class ConstantAccelerationTest : public ::testing::Test {
 public:
  // Start at time increment so first IMU measurement is after starting combined nav state time
  ConstantAccelerationTest()
      : acceleration_i_(0.1),
        acceleration_(acceleration_i_, acceleration_i_, acceleration_i_),
        time_increment_(1.0 / 125.0),
        start_time_(time_increment_),
        num_measurements_(20) {
    const ia::ImuAugmentorParams params = ia::DefaultImuAugmentorParams();
    imu_augmentor_wrapper_.reset(new ia::ImuAugmentorWrapper(params));
  }

  void SetUp() final {
    const std::vector<lm::ImuMeasurement> imu_measurements =
      ia::ConstantAccelerationMeasurements(acceleration_, num_measurements_, start_time_, time_increment_);
    for (const auto& imu_measurement : imu_measurements) {
      const auto imu_msg = ia::ImuMsg(imu_measurement);
      imu_augmentor_wrapper_->ImuCallback(imu_msg);
    }
  }

  ia::ImuAugmentorWrapper& imu_augmentor_wrapper() { return *imu_augmentor_wrapper_; }

  double acceleration_i() { return acceleration_i_; }

  const Eigen::Vector3d& acceleration() { return acceleration_; }

  double time_increment() { return time_increment_; }

  double start_time() { return start_time_; }

  int num_measurements() { return num_measurements_; }

 private:
  std::unique_ptr<ia::ImuAugmentorWrapper> imu_augmentor_wrapper_;
  const double acceleration_i_;
  const Eigen::Vector3d acceleration_;
  const double time_increment_;
  const lc::Time start_time_;
  const int num_measurements_;
};

TEST_F(ConstantAccelerationTest, AddAllMeasurements) {
  const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(), 0);
  ff_msgs::GraphState loc_msg;
  lc::CombinedNavStateToMsg(initial_state, loc_msg);
  imu_augmentor_wrapper().LocalizationStateCallback(loc_msg);
  const auto imu_augmented_state = imu_augmentor_wrapper().LatestImuAugmentedCombinedNavStateAndCovariances();
  ASSERT_TRUE(imu_augmented_state != boost::none);
  EXPECT_NEAR(imu_augmented_state->first.timestamp(), num_measurements() * time_increment(), 1e-6);
  const double expected_velocity_i = acceleration_i() * num_measurements() * time_increment();
  const gtsam::Vector3 expected_velocity(expected_velocity_i, expected_velocity_i, expected_velocity_i);
  // TODO(rsoussan): Replace this with assert pred2 with eigen comparisson when other pr merged
  EXPECT_TRUE(imu_augmented_state->first.velocity().matrix().isApprox(expected_velocity.matrix(), 1e-6));
  // x = 1/2*a*t^2
  const double expected_position_i = acceleration_i() * 0.5 * std::pow(num_measurements() * time_increment(), 2);
  const gtsam::Vector3 expected_position(expected_position_i, expected_position_i, expected_position_i);
  // TODO(rsoussan): Replace this with assert pred2 with eigen comparisson when other pr merged
  EXPECT_TRUE(imu_augmented_state->first.pose().translation().matrix().isApprox(expected_position.matrix(), 1e-6));
}

/*TEST_F(ConstantAccelerationTest, AddHalfOfMeasurements) {
  const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(), 0);
  const lc::Time imu_augmented_state_start_time = num_measurements() / 2 * time_increment();
  lc::CombinedNavState imu_augmented_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(), imu_augmented_state_start_time);
  imu_augmentor().PimPredict(initial_state, imu_augmented_state);

  EXPECT_NEAR(imu_augmented_state.timestamp(), num_measurements() * time_increment(), 1e-6);
  const double expected_velocity_i = acceleration_i() * num_measurements() / 2 * time_increment();
  const gtsam::Vector3 expected_velocity(expected_velocity_i, expected_velocity_i, expected_velocity_i);
  // TODO(rsoussan): Replace this with assert pred2 with eigen comparisson when other pr merged
  EXPECT_TRUE(imu_augmented_state.velocity().matrix().isApprox(expected_velocity.matrix(), 1e-6));
  // x = 1/2*a*t^2
  const double expected_position_i = acceleration_i() * 0.5 * std::pow(num_measurements() / 2 * time_increment(), 2);
  const gtsam::Vector3 expected_position(expected_position_i, expected_position_i, expected_position_i);
  // TODO(rsoussan): Replace this with assert pred2 with eigen comparisson when other pr merged
  EXPECT_TRUE(imu_augmented_state.pose().translation().matrix().isApprox(expected_position.matrix(), 1e-6));
}

TEST_F(ConstantAccelerationTest, AddAllMeasurementsWithAccelBias) {
  const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(acceleration(), gtsam::Vector3::Zero()), 0);
  lc::CombinedNavState imu_augmented_state = initial_state;
  imu_augmentor().PimPredict(initial_state, imu_augmented_state);

  EXPECT_NEAR(imu_augmented_state.timestamp(), num_measurements() * time_increment(), 1e-6);
  // TODO(rsoussan): Replace this with assert pred2 with eigen comparisson when other pr merged
  EXPECT_TRUE(imu_augmented_state.velocity().matrix().isApprox(gtsam::Vector3::Zero().matrix(), 1e-6));
  // TODO(rsoussan): Replace this with assert pred2 with eigen comparisson when other pr merged
  EXPECT_TRUE(imu_augmented_state.pose().translation().matrix().isApprox(gtsam::Vector3::Zero().matrix(), 1e-6));
}*/

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
