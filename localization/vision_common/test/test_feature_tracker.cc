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

class FeatureTrackerTest : public ::testing::Test {
 public:
  FeatureTrackerTest() {}

  void SetUp() final {
    for (int time = 0; time < num_timestamps_; ++time) {
      // Add fewer older measurements to tracks to help test removing old measurements.
      for (int id = 0; id <= time; ++id) {
        // Make image points different for different measurements
        const vc::FeaturePoint p(id * 10 + time, id * 10 + time + 1, time + 1, id, time);
        points_.emplace_back(p);
      }
    }
  }

  void InitializeWithRemoval() {
    vc::FeatureTrackerParams params;
    params.remove_undetected_feature_tracks = true;
    feature_tracker_.reset(new vc::FeatureTracker(params));
  }

  void InitializeWithoutRemoval() {
    vc::FeatureTrackerParams params;
    params.remove_undetected_feature_tracks = false;
    feature_tracker_.reset(new vc::FeatureTracker(params));
  }

  const int num_tracks_ = 5;
  const int num_timestamps_ = 5;
  vc::FeaturePoints points_;
  std::unique_ptr<vc::FeatureTracker> feature_tracker_;
};

TEST_F(FeatureTrackerTest, SizeEmptyClear) {
  InitializeWithRemoval();
  EXPECT_EQ(feature_tracker_->size(), 0);
  EXPECT_TRUE(feature_tracker_->empty());
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
