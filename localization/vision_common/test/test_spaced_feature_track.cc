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

#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <vision_common/feature_tracker.h>
#include <vision_common/test_utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace vc = vision_common;

class SpacedFeatureTrackTest : public ::testing::Test {
 public:
  SpacedFeatureTrackTest() {}

  void SetUp() final {}

  fa::SpacedFeatureTrackParams DefaultParams() {
    fa::SpacedFeatureTrackParams params;
    params.enabled = true;
    params.huber_k = 1.345;
    params.add_velocity_prior = true;
    params.add_pose_between_factor = true;
    params.prior_velocity_stddev = 0.1;
    params.pose_between_factor_translation_stddev = 0.2;
    params.pose_between_factor_rotation_stddev = 0.3;
    return params;
  }

  std::unique_ptr<vc::SpacedFeatureTrack> feature_tracker_;
};

TEST(SpacedFeatureTrackTest, RansacPnP) {
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
