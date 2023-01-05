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

#include <graph_optimizer/nodes.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>

#include <gtest/gtest.h>

namespace go = graph_optimizer;
namespace lc = localization_common;

TEST(NodesTester, AddRemove) {
  go::Nodes nodes;
  EXPECT_EQ(nodes.size(), 0);

  // Add element
  const double element_1 = 100.3;
  const auto key_1 = nodes.Add(element_1);
  EXPECT_TRUE(nodes.Contains(key_1));
  EXPECT_FALSE(nodes.Contains(2));
  EXPECT_EQ(nodes.size(), 1);
  {
    const auto bad_key_val = nodes.Node<double>(2);
    EXPECT_TRUE(bad_key_val == boost::none);
    const auto bad_type_val = nodes.Node<int>(2);
    EXPECT_TRUE(bad_type_val == boost::none);
    const auto good_val = nodes.Node<double>(key_1);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_1);
  }

  // Add element
  const double element_2 = 37.1;
  const auto key_2 = nodes.Add(element_2);
  EXPECT_TRUE(nodes.Contains(key_1));
  EXPECT_TRUE(nodes.Contains(key_2));
  EXPECT_FALSE(nodes.Contains(300));
  EXPECT_EQ(nodes.size(), 2);
  {
    const auto bad_key_val = nodes.Node<double>(3);
    EXPECT_TRUE(bad_key_val == boost::none);
    const auto bad_type_val = nodes.Node<int>(key_2);
    EXPECT_TRUE(bad_type_val == boost::none);
    const auto good_val = nodes.Node<double>(key_1);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_1);
  }
  {
    const auto good_val = nodes.Node<double>(key_2);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_2);
  }

  // Remove
  EXPECT_TRUE(nodes.Remove(key_1));
  EXPECT_FALSE(nodes.Contains(key_1));
  EXPECT_TRUE(nodes.Contains(key_2));
  EXPECT_EQ(nodes.size(), 1);
  {
    const auto good_val = nodes.Node<double>(key_2);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_2);
  }

  // Bad Remove
  EXPECT_FALSE(nodes.Remove(key_1));
  EXPECT_FALSE(nodes.Remove(100));

  // Remove
  EXPECT_TRUE(nodes.Remove(key_2));
  EXPECT_FALSE(nodes.Contains(key_1));
  EXPECT_FALSE(nodes.Contains(key_2));
  EXPECT_EQ(nodes.size(), 0);
  {
    const auto bad_val = nodes.Node<double>(key_2);
    EXPECT_TRUE(bad_val == boost::none);
  }
}

TEST(NodesTester, Serialization) {
  const go::Nodes nodes;
  const auto serialized_nodes = gtsam::serializeBinary(nodes);
  go::Nodes deserialized_nodes;
  gtsam::deserializeBinary(serialized_nodes, deserialized_nodes);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
