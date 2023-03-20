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

TEST_F(SpacedFeatureTrackTest, SecondLatestTimestamp) {
  const auto t = track_.SecondLatestTimestamp();
  ASSERT_TRUE(t != boost::none);
  EXPECT_NEAR(*t, points_[num_points_ - 2].timestamp, 1e-6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
