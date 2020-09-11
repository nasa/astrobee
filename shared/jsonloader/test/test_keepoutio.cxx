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

#include <jsonloader/keepoutio.h>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <string>

// TEST_DIR is defined by cmake as the test directory of our module
const std::string kDataDir = std::string(TEST_DIR) + "/data/";

TEST(KeepoutIO, LoadGoodKeepout) {
  jsonloader::Keepout k(true);  // opposite of what it should be
  ASSERT_TRUE(jsonloader::ReadKeepoutFile(kDataDir + "keepout.json", &k));
  EXPECT_FALSE(k.IsSafe());
  EXPECT_EQ(k.GetBoxes().size(), 1);
}

TEST(KeepoutIO, LoadBadKeepout) {
  jsonloader::Keepout k(true);

  LOG(INFO) << "expect error about 'safe' field";
  EXPECT_FALSE(jsonloader::ReadKeepoutFile(
               kDataDir + "keepout_missing_safe.json", &k));

  LOG(INFO) << "expect error about 'sequence' field";
  EXPECT_FALSE(jsonloader::ReadKeepoutFile(
               kDataDir + "keepout_missing_sequence.json", &k));

  LOG(INFO) << "expect error about 'invalid bounding box'";
  EXPECT_FALSE(jsonloader::ReadKeepoutFile(
               kDataDir + "keepout_wrong_sequence.json", &k));
}

TEST(KeepoutIO, LoadDir) {
  // Required watching everytime you read this line:
  // https://youtu.be/siwpn14IE7E
  jsonloader::Keepout safeZone(true), dangerZone(false);

  jsonloader::ReadKeepoutDirectory(kDataDir + "group/",
      &safeZone, &dangerZone);

  EXPECT_EQ(safeZone.GetBoxes().size(), 1);
  EXPECT_EQ(dangerZone.GetBoxes().size(), 2);
}
