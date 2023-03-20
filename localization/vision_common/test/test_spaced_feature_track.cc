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
#include <vision_common/spaced_feature_track.h>
#include <vision_common/test_utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace vc = vision_common;

class SpacedFeatureTrackTest : public ::testing::Test {
 public:
  SpacedFeatureTrackTest() : track_(feature_track_id_) {}

  void SetUp() final {
    for (int i = 0; i < num_points_; ++i) {
      const lc::Time time = i;
      const vc::FeaturePoint p(i, i + 1, i + 2, feature_track_id_, time);
      track_.Add(time, p);
      points_.emplace_back(p);
    }
  }

  void EXPECT_SAME_POINT(const vc::FeaturePoint& p_a, const vc::FeaturePoint& p_b) {
    EXPECT_EQ(p_a.image_id, p_b.image_id);
    EXPECT_EQ(p_a.feature_track_id, p_b.feature_track_id);
    EXPECT_MATRIX_NEAR(p_a.image_point, p_b.image_point, 1e-6);
    EXPECT_NEAR(p_a.timestamp, p_b.timestamp, 1e-6);
  }

  void EXPECT_SAME_POINT(const vc::FeaturePoint& p_a, const int index) { EXPECT_SAME_POINT(p_a, points_[index]); }
  const vc::FeatureId feature_track_id_ = 4;
  const int num_points_ = 10;
  vc::SpacedFeatureTrack track_;
  std::vector<vc::FeaturePoint> points_;
};

TEST_F(SpacedFeatureTrackTest, LatestSpacedPoints) {
  vc::SpacedFeatureTrack track(1);
  // Test empty track
  {
    const auto points = track.LatestSpacedPoints(0);
    EXPECT_EQ(points.size(), 0);
  }
  {
    const auto points = track.LatestSpacedPoints(10);
    EXPECT_EQ(points.size(), 0);
  }

  // Test single point track
  track.Add(points_[0].timestamp, points_[0]);
  {
    const auto points = track.LatestSpacedPoints(0);
    ASSERT_EQ(points.size(), 1);
    EXPECT_SAME_POINT(points[0], points_[0]);
  }
  {
    const auto points = track.LatestSpacedPoints(5);
    ASSERT_EQ(points.size(), 1);
    EXPECT_SAME_POINT(points[0], points_[0]);
  }

  // Test two point track
  track.Add(points_[1].timestamp, points_[1]);
  {
    const auto points = track.LatestSpacedPoints(0);
    ASSERT_EQ(points.size(), 2);
    EXPECT_SAME_POINT(points[0], points_[0]);
    EXPECT_SAME_POINT(points[1], points_[1]);
  }
  {
    const auto points = track.LatestSpacedPoints(1);
    ASSERT_EQ(points.size(), 1);
    EXPECT_SAME_POINT(points[0], points_[1]);
  }
  {
    const auto points = track.LatestSpacedPoints(4);
    ASSERT_EQ(points.size(), 1);
    EXPECT_SAME_POINT(points[0], points_[1]);
  }

  // Test multi point track
  {
    const auto points = track_.LatestSpacedPoints(0);
    ASSERT_EQ(points.size(), 10);
    for (int i = 0; i < num_points_; ++i) EXPECT_SAME_POINT(points[i], points_[i]);
  }
  {
    const auto points = track_.LatestSpacedPoints(1);
    ASSERT_EQ(points.size(), 5);
    // Expect p1, p3, p5, p7, p9
    for (int i = 0; i < 5; ++i) EXPECT_SAME_POINT(points[i], points_[2 * i + 1]);
  }
  {
    const auto points = track_.LatestSpacedPoints(2);
    ASSERT_EQ(points.size(), 4);
    // Expect p0, p3, p6, p9
    for (int i = 0; i < 4; ++i) EXPECT_SAME_POINT(points[i], points_[3 * i]);
  }
  {
    const auto points = track_.LatestSpacedPoints(3);
    ASSERT_EQ(points.size(), 3);
    // Expect p1, p5, p9
    EXPECT_SAME_POINT(points[0], points_[1]);
    EXPECT_SAME_POINT(points[1], points_[5]);
    EXPECT_SAME_POINT(points[2], points_[9]);
  }
}

TEST_F(SpacedFeatureTrackTest, MaxSpacing) {
  vc::SpacedFeatureTrack track(1);
  // Test empty track
  {
    EXPECT_EQ(track.MaxSpacing(10), 0);
    EXPECT_EQ(track.MaxSpacing(0), 0);
  }
  // Test single point track
  {
    track.Add(points_[0].timestamp, points_[0]);
    EXPECT_EQ(track.MaxSpacing(1), 0);
    EXPECT_EQ(track.MaxSpacing(0), 0);
  }
  // Test two point track
  {
    track.Add(points_[1].timestamp, points_[1]);
    EXPECT_EQ(track.MaxSpacing(0), 0);
    EXPECT_EQ(track.MaxSpacing(1), 0);
    EXPECT_EQ(track.MaxSpacing(2), 0);
    EXPECT_EQ(track.MaxSpacing(10), 0);
  }

  // Test multi point track
  EXPECT_EQ(track_.MaxSpacing(0), 0);
  EXPECT_EQ(track_.MaxSpacing(1), 0);
  // Should result in p0, p9. Spacing is 8.
  EXPECT_EQ(track_.MaxSpacing(2), 8);
  // For 10 points with max 3 points, max spacing of 3 results in:
  // p0, p4, p8
  // A spacing of 4 would result in:
  // p0, p5, p10
  // but p10 doesn't exist, so this is invalid.
  EXPECT_EQ(track_.MaxSpacing(3), 3);
  // For 10 points with max 4 points, max spacing of 2 results in:
  // p0, p3, p6, p9
  EXPECT_EQ(track_.MaxSpacing(4), 2);
  // For 10 points with max 5 points, max spacing of 1 results in:
  // p0, p2, p4, p6, p8
  EXPECT_EQ(track_.MaxSpacing(5), 1);
  // For 10 points with max 6 points, max spacing of 0 results in:
  // p0, p1, p2, p3, p4, p5, p6
  EXPECT_EQ(track_.MaxSpacing(6), 0);
  EXPECT_EQ(track_.MaxSpacing(7), 0);
  EXPECT_EQ(track_.MaxSpacing(8), 0);
  EXPECT_EQ(track_.MaxSpacing(9), 0);
  EXPECT_EQ(track_.MaxSpacing(10000), 0);
}

TEST_F(SpacedFeatureTrackTest, SecondLatestTimestamp) {
  vc::SpacedFeatureTrack track(1);
  // Test empty track
  {
    const auto t = track.SecondLatestTimestamp();
    EXPECT_TRUE(t == boost::none);
  }
  // Test single point track
  {
    track.Add(points_[0].timestamp, points_[0]);
    const auto t = track.SecondLatestTimestamp();
    EXPECT_TRUE(t == boost::none);
  }
  // Test two point track
  {
    track.Add(points_[1].timestamp, points_[1]);
    const auto t = track.SecondLatestTimestamp();
    ASSERT_TRUE(t != boost::none);
    EXPECT_NEAR(*t, points_[0].timestamp, 1e-6);
  }

  // Test multi point track
  const auto t = track_.SecondLatestTimestamp();
  ASSERT_TRUE(t != boost::none);
  EXPECT_NEAR(*t, points_[num_points_ - 2].timestamp, 1e-6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
