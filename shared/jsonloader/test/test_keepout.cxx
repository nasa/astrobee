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

// Test keepout
// This test tests the creation of keepout zones using bounding boxes, the
// isSafe condition and that merging bouding boxes gives expected result.

#include <jsonloader/keepout.h>

#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <algorithm>

TEST(Keepout, Construction) {
  // Construct an empty one, make sure it's empty
  jsonloader::Keepout empty(false);
  EXPECT_FALSE(empty.IsSafe());
  EXPECT_EQ(empty.GetBoxes().size(), 0);

  // Construct a few bounding boxes
  Eigen::Vector3f min(0, 1, 2);
  Eigen::Vector3f max(3, 4, 5);
  jsonloader::Keepout::Sequence v;
  v.push_back(jsonloader::Keepout::BoundingBox(min, max));

  // Make a keepout out of this area
  jsonloader::Keepout k(v, true);

  // Make sure it is safe and we have a bounding box
  EXPECT_TRUE(k.IsSafe());
  EXPECT_EQ(k.GetBoxes().size(), 1);
}

TEST(Keepout, MergeCorrect) {
  // Construct a few bounding boxes
  Eigen::Vector3f min(0, 1, 2);
  Eigen::Vector3f max(3, 4, 5);
  jsonloader::Keepout::Sequence v;
  v.push_back(jsonloader::Keepout::BoundingBox(min, max));

  // Make a couple keepout zones
  jsonloader::Keepout a(v, true);
  jsonloader::Keepout b(v, true);

  EXPECT_EQ(a.IsSafe(), b.IsSafe());
  a.Merge(b);
  EXPECT_EQ(a.GetBoxes().size(), 2);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
