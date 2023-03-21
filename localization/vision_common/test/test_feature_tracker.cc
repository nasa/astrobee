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
      std::vector<vc::FeaturePoint> points;
      // Add fewer older measurements to tracks to help test removing old measurements.
      for (int id = 0; id <= time; ++id) {
        // Make image points different for different measurements
        const vc::FeaturePoint p(id * 10 + time, id * 10 + time + 1, time + 1, id, time);
        points_.emplace_back(p);
        points.emplace_back(p);
      }
      timestamped_points_.emplace_back(points);
    }
  }

  void EXPECT_SAME_POINT(const vc::FeaturePoint& p_a, const vc::FeaturePoint& p_b) {
    EXPECT_EQ(p_a.image_id, p_b.image_id);
    EXPECT_EQ(p_a.feature_track_id, p_b.feature_track_id);
    EXPECT_MATRIX_NEAR(p_a.image_point, p_b.image_point, 1e-6);
    EXPECT_NEAR(p_a.timestamp, p_b.timestamp, 1e-6);
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
  std::vector<vc::FeaturePoints> timestamped_points_;
  std::unique_ptr<vc::FeatureTracker> feature_tracker_;
};

TEST_F(FeatureTrackerTest, SizeEmptyClear) {
  InitializeWithoutRemoval();
  EXPECT_EQ(feature_tracker_->size(), 0);
  EXPECT_TRUE(feature_tracker_->empty());

  // Add measurements
  feature_tracker_->Update(points_);
  EXPECT_EQ(feature_tracker_->size(), num_tracks_);
  EXPECT_FALSE(feature_tracker_->empty());

  // Remove measurements
  feature_tracker_->Clear();
  EXPECT_EQ(feature_tracker_->size(), 0);
  EXPECT_TRUE(feature_tracker_->empty());
}

TEST_F(FeatureTrackerTest, AddMeasurentsWithoutRemoval) {
  InitializeWithoutRemoval();
  EXPECT_EQ(feature_tracker_->size(), 0);
  EXPECT_TRUE(feature_tracker_->empty());

  // Add first set of timestamped measurements
  feature_tracker_->Update(timestamped_points_[0]);
  EXPECT_EQ(feature_tracker_->size(), 1);
  {
    // Only one point exists in first measurement
    const auto& added_point = timestamped_points_[0][0];
    const auto id = added_point.feature_track_id;
    const auto time = added_point.timestamp;
    const auto& track = feature_tracker_->feature_tracks().at(id);
    const auto& track_point = track.Get(time);
    ASSERT_TRUE(track_point != boost::none);
    EXPECT_SAME_POINT(added_point, track_point->value);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
