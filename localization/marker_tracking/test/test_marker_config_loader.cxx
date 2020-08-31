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

#include <marker_tracking/arconfigio.h>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <string>

TEST(MarkerDetector, ConfigLoader) {
  std::string                 data_dir = std::string(TEST_DIR) + std::string("/data/");
  std::string                 filename = std::string("markers_sample.config");
  config_reader::ConfigReader config;
  config.AddFile((data_dir + filename).c_str());

  marker_tracking::ARTagMap ar_tags;

  marker_tracking::LoadARTagsConfig(&config, &ar_tags);

  // We have 3 tags in the file
  EXPECT_EQ(3, ar_tags.size());

  // check coordinates of first tag
  static const int BL = 0, BR = 1, TR = 2, TL = 3;
  EXPECT_NEAR(10.0, ar_tags[16].row(TL)[0], 1E-6);
  EXPECT_NEAR(10.0, ar_tags[16].row(TL)[1], 1E-6);
  EXPECT_NEAR(-1.0, ar_tags[16].row(TL)[2], 1E-6);
  EXPECT_NEAR(60.0, ar_tags[16].row(TR)[0], 1E-6);
  EXPECT_NEAR(10.0, ar_tags[16].row(TR)[1], 1E-6);
  EXPECT_NEAR(-1.0, ar_tags[16].row(TR)[2], 1E-6);
  EXPECT_NEAR(10.0, ar_tags[16].row(BL)[0], 1E-6);
  EXPECT_NEAR(60.0, ar_tags[16].row(BL)[1], 1E-6);
  EXPECT_NEAR(-1.0, ar_tags[16].row(BL)[2], 1E-6);

  // Check computed coordinates of second tag
  Eigen::Vector3f xvec  = ar_tags[17].row(TR) - ar_tags[17].row(TL);
  Eigen::Vector3f yvec  = ar_tags[17].row(BL) - ar_tags[17].row(TL);
  Eigen::Vector3f diag1 = xvec + yvec;
  Eigen::Vector3f diag2 = ar_tags[17].row(BR) - ar_tags[17].row(TL);
  EXPECT_NEAR(diag1(0), diag2(0), 1E-2);
  EXPECT_NEAR(diag1(1), diag2(1), 1E-2);
  EXPECT_NEAR(diag1(2), diag2(2), 1E-2);
}
