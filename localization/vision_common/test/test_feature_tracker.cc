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

  // Adds increasing measurements then decreasing measurements.
  // track 0: .....
  // track 1:  ...
  // track 2:   .
  // Here the x axis is the timestamp and the dot indicates whether a measurement
  // occured for the track.
  void SetUp() final {
    const int num_timestamps =
      num_timestamps_with_increasing_measurements_ + num_timestamps_with_decreasing_measurements_;
    for (int time = 0; time < num_timestamps; ++time) {
      std::vector<vc::FeaturePoint> points;
      // Add fewer older measurements initially to tracks to help test removing old measurements.
      if (time < num_timestamps_with_increasing_measurements_) {
        for (int id = 0; id <= time; ++id) {
          // Make image points different for different measurements
          const vc::FeaturePoint p(id * 10 + time, id * 10 + time + 1, time + 1, id, time);
          points_.emplace_back(p);
          points.emplace_back(p);
        }
        timestamped_points_.emplace_back(points);
      } else {
        // Add fewer newer measurements later to tracks to help test removing undetected tracks.
        for (int id = 0; id < num_timestamps - time; ++id) {
          // Make image points different for different measurements
          const vc::FeaturePoint p(id * 10 + time, id * 10 + time + 1, time + 1, id, time);
          points_.emplace_back(p);
          points.emplace_back(p);
        }
        timestamped_points_.emplace_back(points);
      }
    }
  }

  void EXPECT_SAME_POINT(const vc::FeaturePoint& p_a, const vc::FeaturePoint& p_b) {
    EXPECT_EQ(p_a.image_id, p_b.image_id);
    EXPECT_EQ(p_a.feature_track_id, p_b.feature_track_id);
    EXPECT_MATRIX_NEAR(p_a.image_point, p_b.image_point, 1e-6);
    EXPECT_NEAR(p_a.timestamp, p_b.timestamp, 1e-6);
  }

  void EXPECT_SAME_POINT(const int timestamp_index, const int measurement_index) {
    const auto& added_point = timestamped_points_[timestamp_index][measurement_index];
    const auto id = added_point.feature_track_id;
    const auto time = added_point.timestamp;
    const auto& track = feature_tracker_->feature_tracks().at(id);
    const auto& track_point = track.Get(time);
    ASSERT_TRUE(track_point != boost::none);
    EXPECT_SAME_POINT(added_point, track_point->value);
  }

  void EXPECT_TRACK_SIZE_EQ(const int track_index, const int size) {
    EXPECT_EQ(feature_tracker_->feature_tracks().at(track_index).size(), size);
  }

  void InitializeWithRemoval() {
    vc::FeatureTrackerParams params;
    params.remove_undetected_feature_tracks = true;
    feature_tracker_.reset(new vc::FeatureTracker<>(params));
  }

  void InitializeWithoutRemoval() {
    vc::FeatureTrackerParams params;
    params.remove_undetected_feature_tracks = false;
    feature_tracker_.reset(new vc::FeatureTracker<>(params));
  }

  const int num_tracks_ = 3;
  const int num_timestamps_with_increasing_measurements_ = 2;
  const int num_timestamps_with_decreasing_measurements_ = 3;
  vc::FeaturePoints points_;
  std::vector<vc::FeaturePoints> timestamped_points_;
  std::unique_ptr<vc::FeatureTracker<>> feature_tracker_;
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

// Without removal, tracks without detections should continue
// to exist.
TEST_F(FeatureTrackerTest, AddMeasurentsWithoutRemoval) {
  InitializeWithoutRemoval();
  EXPECT_EQ(feature_tracker_->size(), 0);
  EXPECT_TRUE(feature_tracker_->empty());

  // Add first set of timestamped measurements
  feature_tracker_->Update(timestamped_points_[0]);
  EXPECT_EQ(feature_tracker_->size(), 1);
  {
    // Only one point exists in first measurement
    EXPECT_TRACK_SIZE_EQ(0, 1);
    EXPECT_SAME_POINT(0, 0);
  }
  // Add second set of timestamped measurements
  feature_tracker_->Update(timestamped_points_[1]);
  EXPECT_EQ(feature_tracker_->size(), 2);
  {
    // Check first track which should have two points
    EXPECT_TRACK_SIZE_EQ(0, 2);
    EXPECT_SAME_POINT(0, 0);
    EXPECT_SAME_POINT(1, 0);
    // Check second track which should have one point at second timestamp
    EXPECT_TRACK_SIZE_EQ(1, 1);
    EXPECT_SAME_POINT(1, 1);
  }
  // Add 3rd set of timestamped measurements
  feature_tracker_->Update(timestamped_points_[2]);
  EXPECT_EQ(feature_tracker_->size(), 3);
  {
    // Check first track which should have 3 points
    EXPECT_TRACK_SIZE_EQ(0, 3);
    EXPECT_SAME_POINT(0, 0);
    EXPECT_SAME_POINT(1, 0);
    EXPECT_SAME_POINT(2, 0);
    // Check second track which should have 2 points
    EXPECT_TRACK_SIZE_EQ(1, 2);
    EXPECT_SAME_POINT(1, 1);
    EXPECT_SAME_POINT(2, 1);
    // Check third track which should have one point
    EXPECT_TRACK_SIZE_EQ(2, 1);
    EXPECT_SAME_POINT(2, 2);
  }
  // Add 4th set of timestamped measurements
  feature_tracker_->Update(timestamped_points_[3]);
  EXPECT_EQ(feature_tracker_->size(), 3);
  {
    // Check first track which should have 4 points
    EXPECT_TRACK_SIZE_EQ(0, 4);
    EXPECT_SAME_POINT(0, 0);
    EXPECT_SAME_POINT(1, 0);
    EXPECT_SAME_POINT(2, 0);
    EXPECT_SAME_POINT(3, 0);
    // Check second track which should have 3 points
    EXPECT_TRACK_SIZE_EQ(1, 3);
    EXPECT_SAME_POINT(1, 1);
    EXPECT_SAME_POINT(2, 1);
    EXPECT_SAME_POINT(3, 1);
    // Check third track which should have 1 point
    EXPECT_TRACK_SIZE_EQ(2, 1);
    EXPECT_SAME_POINT(2, 2);
  }
  // Add 5th set of timestamped measurements
  feature_tracker_->Update(timestamped_points_[4]);
  EXPECT_EQ(feature_tracker_->size(), 3);
  {
    // Check first track which should have 5 points
    EXPECT_TRACK_SIZE_EQ(0, 5);
    EXPECT_SAME_POINT(0, 0);
    EXPECT_SAME_POINT(1, 0);
    EXPECT_SAME_POINT(2, 0);
    EXPECT_SAME_POINT(3, 0);
    EXPECT_SAME_POINT(4, 0);
    // Check second track which should have 3 points
    EXPECT_TRACK_SIZE_EQ(1, 3);
    EXPECT_SAME_POINT(1, 1);
    EXPECT_SAME_POINT(2, 1);
    EXPECT_SAME_POINT(3, 1);
    // Check third track which should have 1 points
    EXPECT_TRACK_SIZE_EQ(2, 1);
    EXPECT_SAME_POINT(2, 2);
  }
  // Add empty measurement
  // Tracks should be unaffected
  feature_tracker_->Update(vc::FeaturePoints());
  EXPECT_EQ(feature_tracker_->size(), 3);
  {
    // Check first track which should have 5 points
    EXPECT_TRACK_SIZE_EQ(0, 5);
    EXPECT_SAME_POINT(0, 0);
    EXPECT_SAME_POINT(1, 0);
    EXPECT_SAME_POINT(2, 0);
    EXPECT_SAME_POINT(3, 0);
    EXPECT_SAME_POINT(4, 0);
    // Check second track which should have 3 points
    EXPECT_TRACK_SIZE_EQ(1, 3);
    EXPECT_SAME_POINT(1, 1);
    EXPECT_SAME_POINT(2, 1);
    EXPECT_SAME_POINT(3, 1);
    // Check third track which should have 1 points
    EXPECT_TRACK_SIZE_EQ(2, 1);
    EXPECT_SAME_POINT(2, 2);
  }
}

// With removal tracks without detections should gradually be
// removed.
TEST_F(FeatureTrackerTest, AddMeasurentsWithRemoval) {
  InitializeWithRemoval();
  EXPECT_EQ(feature_tracker_->size(), 0);
  EXPECT_TRUE(feature_tracker_->empty());

  // Add first set of timestamped measurements
  feature_tracker_->Update(timestamped_points_[0]);
  EXPECT_EQ(feature_tracker_->size(), 1);
  {
    // Only one point exists in first measurement
    EXPECT_TRACK_SIZE_EQ(0, 1);
    EXPECT_SAME_POINT(0, 0);
  }
  // Add second set of timestamped measurements
  feature_tracker_->Update(timestamped_points_[1]);
  EXPECT_EQ(feature_tracker_->size(), 2);
  {
    // Check first track which should have two points
    EXPECT_TRACK_SIZE_EQ(0, 2);
    EXPECT_SAME_POINT(0, 0);
    EXPECT_SAME_POINT(1, 0);
    // Check second track which should have one point at second timestamp
    EXPECT_TRACK_SIZE_EQ(1, 1);
    EXPECT_SAME_POINT(1, 1);
  }
  // Add 3rd set of timestamped measurements
  feature_tracker_->Update(timestamped_points_[2]);
  EXPECT_EQ(feature_tracker_->size(), 3);
  {
    // Check first track which should have 3 points
    EXPECT_TRACK_SIZE_EQ(0, 3);
    EXPECT_SAME_POINT(0, 0);
    EXPECT_SAME_POINT(1, 0);
    EXPECT_SAME_POINT(2, 0);
    // Check second track which should have 2 points
    EXPECT_TRACK_SIZE_EQ(1, 2);
    EXPECT_SAME_POINT(1, 1);
    EXPECT_SAME_POINT(2, 1);
    // Check third track which should have one point
    EXPECT_TRACK_SIZE_EQ(2, 1);
    EXPECT_SAME_POINT(2, 2);
  }

  // Now since measurements taper off, tracks should start to be removed!
  // Add 4th set of timestamped measurements
  // 3rd track has no measurements so it should be removed
  feature_tracker_->Update(timestamped_points_[3]);
  EXPECT_EQ(feature_tracker_->size(), 2);
  {
    // Check first track which should have 4 points
    EXPECT_TRACK_SIZE_EQ(0, 4);
    EXPECT_SAME_POINT(0, 0);
    EXPECT_SAME_POINT(1, 0);
    EXPECT_SAME_POINT(2, 0);
    EXPECT_SAME_POINT(3, 0);
    // Check second track which should have 3 points
    EXPECT_TRACK_SIZE_EQ(1, 3);
    EXPECT_SAME_POINT(1, 1);
    EXPECT_SAME_POINT(2, 1);
    EXPECT_SAME_POINT(3, 1);
  }
  // Add 5th set of timestamped measurements
  // 2nd track has no measurements so it should be removed
  feature_tracker_->Update(timestamped_points_[4]);
  EXPECT_EQ(feature_tracker_->size(), 1);
  {
    // Check first track which should have 5 points
    EXPECT_TRACK_SIZE_EQ(0, 5);
    EXPECT_SAME_POINT(0, 0);
    EXPECT_SAME_POINT(1, 0);
    EXPECT_SAME_POINT(2, 0);
    EXPECT_SAME_POINT(3, 0);
    EXPECT_SAME_POINT(4, 0);
  }

  // Add empty measurement
  // Should remove all tracks
  feature_tracker_->Update(vc::FeaturePoints());
  EXPECT_TRUE(feature_tracker_->empty());
}

TEST_F(FeatureTrackerTest, RemoveOldPoints) {
  InitializeWithoutRemoval();
  EXPECT_EQ(feature_tracker_->size(), 0);
  EXPECT_TRUE(feature_tracker_->empty());

  // Add first 3 measurements
  feature_tracker_->Update(timestamped_points_[0]);
  feature_tracker_->Update(timestamped_points_[1]);
  feature_tracker_->Update(timestamped_points_[2]);
  EXPECT_EQ(feature_tracker_->size(), 3);
  {
    // Check first track which should have 3 points
    EXPECT_TRACK_SIZE_EQ(0, 3);
    EXPECT_SAME_POINT(0, 0);
    EXPECT_SAME_POINT(1, 0);
    EXPECT_SAME_POINT(2, 0);
    // Check second track which should have 2 points
    EXPECT_TRACK_SIZE_EQ(1, 2);
    EXPECT_SAME_POINT(1, 1);
    EXPECT_SAME_POINT(2, 1);
    // Check third track which should have one point
    EXPECT_TRACK_SIZE_EQ(2, 1);
    EXPECT_SAME_POINT(2, 2);
  }
  // Remove first measurement occuring at t = 0
  {
    const lc::Time oldest_allowed_time = 1;
    feature_tracker_->RemoveOldPoints(oldest_allowed_time);
    EXPECT_EQ(feature_tracker_->size(), 3);
    // Check first track which should have 2 points
    EXPECT_TRACK_SIZE_EQ(0, 2);
    EXPECT_SAME_POINT(1, 0);
    EXPECT_SAME_POINT(2, 0);
    // Check second track which should have 2 points
    EXPECT_TRACK_SIZE_EQ(1, 2);
    EXPECT_SAME_POINT(1, 1);
    EXPECT_SAME_POINT(2, 1);
    // Check third track which should have one point
    EXPECT_TRACK_SIZE_EQ(2, 1);
    EXPECT_SAME_POINT(2, 2);
  }
  // Remove second measurement occuring at t = 1
  {
    const lc::Time oldest_allowed_time = 1.5;
    feature_tracker_->RemoveOldPoints(oldest_allowed_time);
    EXPECT_EQ(feature_tracker_->size(), 3);
    // Check first track which should have 1 point
    EXPECT_TRACK_SIZE_EQ(0, 1);
    EXPECT_SAME_POINT(2, 0);
    // Check second track which should have 1 point
    EXPECT_TRACK_SIZE_EQ(1, 1);
    EXPECT_SAME_POINT(2, 1);
    // Check third track which should have one point
    EXPECT_TRACK_SIZE_EQ(2, 1);
    EXPECT_SAME_POINT(2, 2);
  }
  // Remove third measurement occuring at t = 2
  {
    const lc::Time oldest_allowed_time = 2.1;
    feature_tracker_->RemoveOldPoints(oldest_allowed_time);
    EXPECT_EQ(feature_tracker_->size(), 0);
    EXPECT_TRUE(feature_tracker_->empty());
  }
}

TEST_F(FeatureTrackerTest, LengthOrdered) {
  InitializeWithoutRemoval();
  EXPECT_EQ(feature_tracker_->size(), 0);
  EXPECT_TRUE(feature_tracker_->empty());

  // Create points with increasingly more points for increasing id
  std::vector<vc::FeaturePoints> timestamped_points(3);
  for (int id = 0; id < 3; ++id) {
    std::vector<vc::FeaturePoint> points;
    for (int time = 0; time <= id; ++time) {
      // Make image points different for different measurements
      const vc::FeaturePoint p(id * 10 + time, id * 10 + time + 1, time + 1, id, time);
      timestamped_points[time].emplace_back(p);
    }
  }
  EXPECT_EQ(timestamped_points[0].size(), 3);
  EXPECT_EQ(timestamped_points[1].size(), 2);
  EXPECT_EQ(timestamped_points[2].size(), 1);
  feature_tracker_->Update(timestamped_points[0]);
  feature_tracker_->Update(timestamped_points[1]);
  feature_tracker_->Update(timestamped_points[2]);

  EXPECT_EQ(feature_tracker_->size(), 3);

  // Check track lengths
  EXPECT_TRACK_SIZE_EQ(0, 1);
  EXPECT_TRACK_SIZE_EQ(1, 2);
  EXPECT_TRACK_SIZE_EQ(2, 3);

  const auto length_ordered_tracks = feature_tracker_->FeatureTracksLengthOrdered();
  EXPECT_EQ(length_ordered_tracks[0].get().size(), 3);
  EXPECT_EQ(length_ordered_tracks[0].get().id(), 2);
  EXPECT_EQ(length_ordered_tracks[1].get().size(), 2);
  EXPECT_EQ(length_ordered_tracks[1].get().id(), 1);
  EXPECT_EQ(length_ordered_tracks[2].get().size(), 1);
  EXPECT_EQ(length_ordered_tracks[2].get().id(), 0);
}
// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
