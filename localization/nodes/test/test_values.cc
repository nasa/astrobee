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

#include <nodes/values.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace no = nodes;

TEST(ValuesTester, AddRemove) {
  no::Values values;
  EXPECT_EQ(values.size(), 0);

  // Add element
  const double element_1 = 100.3;
  const auto key_1 = values.Add(element_1);
  EXPECT_TRUE(values.Contains(key_1));
  EXPECT_FALSE(values.Contains(2));
  EXPECT_EQ(values.size(), 1);
  {
    const auto bad_key_val = values.Value<double>(2);
    EXPECT_TRUE(bad_key_val == boost::none);
    const auto bad_type_val = values.Value<int>(2);
    EXPECT_TRUE(bad_type_val == boost::none);
    const auto good_val = values.Value<double>(key_1);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_1);
  }

  // Add element
  const double element_2 = 37.1;
  const auto key_2 = values.Add(element_2);
  EXPECT_TRUE(values.Contains(key_1));
  EXPECT_TRUE(values.Contains(key_2));
  EXPECT_FALSE(values.Contains(300));
  EXPECT_EQ(values.size(), 2);
  {
    const auto bad_key_val = values.Value<double>(3);
    EXPECT_TRUE(bad_key_val == boost::none);
    const auto bad_type_val = values.Value<int>(key_2);
    EXPECT_TRUE(bad_type_val == boost::none);
    const auto good_val = values.Value<double>(key_1);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_1);
  }
  {
    const auto good_val = values.Value<double>(key_2);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_2);
  }

  // Remove
  EXPECT_TRUE(values.Remove(key_1));
  EXPECT_FALSE(values.Contains(key_1));
  EXPECT_TRUE(values.Contains(key_2));
  EXPECT_EQ(values.size(), 1);
  {
    const auto good_val = values.Value<double>(key_2);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_2);
  }

  // Bad Remove
  EXPECT_FALSE(values.Remove(key_1));
  EXPECT_FALSE(values.Remove(100));

  // Remove
  EXPECT_TRUE(values.Remove(key_2));
  EXPECT_FALSE(values.Contains(key_1));
  EXPECT_FALSE(values.Contains(key_2));
  EXPECT_EQ(values.size(), 0);
  {
    const auto bad_val = values.Value<double>(key_2);
    EXPECT_TRUE(bad_val == boost::none);
  }
}

TEST(ValuesTester, Serialization) {
  const no::Values values;
  const auto serialized_values = gtsam::serializeBinary(values);
  no::Values deserialized_values;
  gtsam::deserializeBinary(serialized_values, deserialized_values);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
