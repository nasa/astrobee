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

#include <marker_tracking/arxmlio.h>

#include <gtest/gtest.h>
#include <glog/logging.h>

#include <string>

TEST(AR_XML_IO, Reading) {
  std::string data_dir = std::string(std::getenv("DATA_DIR"));

  marker_tracking::ARTagMap ar_tags;
  marker_tracking::LoadARTagLocation(data_dir + "ar_lab_tags.xml",
                                     &ar_tags);

  // There are 2 traditional AR tags and then one multiscale AR tag. A
  // multscale AR tag is 9 trad AR tags.
  EXPECT_EQ(9u + 1u + 1u, ar_tags.size());
  EXPECT_EQ(1u, ar_tags.count(1));
  EXPECT_EQ(1u, ar_tags.count(2));
  EXPECT_EQ(1u, ar_tags.count(21));
  EXPECT_EQ(1u, ar_tags.count(29));

  // Test that one of the AR tags has the correct top left location.
  static const int TL = 3, BL = 0;
  EXPECT_NEAR(1.907, ar_tags[1].row(TL)[0], 1e-3);
  EXPECT_NEAR(0.257, ar_tags[1].row(TL)[2], 1e-3);

  // Check the midsize has the correct top left and bottom right
  EXPECT_NEAR(-0.993, ar_tags[21].row(TL)[0], 1e-3);
  EXPECT_NEAR(-0.614, ar_tags[21].row(TL)[2], 1e-3);
  EXPECT_NEAR(-0.993, ar_tags[29].row(BL)[0], 1e-3);
  EXPECT_NEAR(-0.824, ar_tags[29].row(BL)[2], 1e-3);
}
