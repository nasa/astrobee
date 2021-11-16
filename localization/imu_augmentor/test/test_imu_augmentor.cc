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

#include <gtest/gtest.h>

namespace ia = imu_augmentor;
TEST(IMUAugmentorTester, PimPredictTimestamp) {
  const auto params = ia::DefaultImuAugmentorParams();
  ia::ImuAugmentor augmentor(params);

  // TODO(rsoussan): add function to add linear accel measurements with ascending timestamps, just pass num
  // measurements! (AAA)

  // Test that timestamp is correct! (BB)
  // make a bunch of same imu measurements
  // pass these to imu aug
  // integrate and check resulting timestamp, ensure this is correct!
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
