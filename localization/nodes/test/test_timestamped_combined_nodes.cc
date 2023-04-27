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

#include <nodes/timestamped_combined_nodes.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace no = nodes;

struct CombinedObject {
  CombinedObject(const double val_a, const double val_b) : val_a(val_a), val_b(val_b) {}
  bool operator==(const CombinedObject& rhs) const { return val_a == rhs.val_a && val_b == rhs.val_b; }
  static CombinedObject Random() { return CombinedObject(lc::RandomDouble(), lc::RandomDouble()); }
  double val_a;
  double val_b;
};

class TestCombinedNodes : public no::TimestampedCombinedNodes<CombinedObject> {
  using Base = no::TimestampedCombinedNodes<CombinedObject>;

 public:
  explicit TestCombinedNodes(std::shared_ptr<no::Values> values) : Base(values) {}

  // For serialization only
  TestCombinedNodes() = default;

 private:
  gtsam::KeyVector AddNode(const CombinedObject& node) final {
    const auto key_a = values_->Add(node.val_a);
    const auto key_b = values_->Add(node.val_b);
    return {key_a, key_b};
  }

  boost::optional<CombinedObject> GetNode(const gtsam::KeyVector& keys,
                                          const localization_common::Time timestamp) const final {
    const auto val_a = values_->Value<double>(keys[0]);
    const auto val_b = values_->Value<double>(keys[1]);
    if (!val_a || !val_b) return boost::none;
    return CombinedObject(*val_a, *val_b);
  }

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};

TEST(TimestampedNodesTester, AddRemoveContainsEmptySize) {
  std::shared_ptr<no::Values> values(new no::Values());
  TestCombinedNodes timestamped_nodes(values);
  EXPECT_EQ(timestamped_nodes.size(), 0);
  EXPECT_TRUE(timestamped_nodes.empty());

  // Add element 1
  const CombinedObject node_1(-2, 23.8);
  const localization_common::Time timestamp_1 = 1.0;
  EXPECT_EQ(timestamped_nodes.Add(timestamp_1, node_1).size(), 2);
  EXPECT_EQ(timestamped_nodes.size(), 1);
  EXPECT_FALSE(timestamped_nodes.empty());
  {
    EXPECT_TRUE(timestamped_nodes.Node(2.0) == boost::none);
    const auto accessed_node = timestamped_nodes.Node(timestamp_1);
    ASSERT_TRUE(accessed_node != boost::none);
    EXPECT_EQ(*accessed_node, node_1);
    EXPECT_TRUE(timestamped_nodes.Contains(timestamp_1));
  }
  // Add element 2
  const CombinedObject node_2(123.2, 22.7);
  const localization_common::Time timestamp_2 = 3.3;
  EXPECT_EQ(timestamped_nodes.Add(timestamp_2, node_2).size(), 2);
  EXPECT_EQ(timestamped_nodes.size(), 2);
  EXPECT_FALSE(timestamped_nodes.empty());
  {
    EXPECT_TRUE(timestamped_nodes.Node(7.0) == boost::none);
    const auto accessed_node_1 = timestamped_nodes.Node(timestamp_1);
    ASSERT_TRUE(accessed_node_1 != boost::none);
    EXPECT_EQ(*accessed_node_1, node_1);
    EXPECT_TRUE(timestamped_nodes.Contains(timestamp_1));
    const auto accessed_node_2 = timestamped_nodes.Node(timestamp_2);
    ASSERT_TRUE(accessed_node_2 != boost::none);
    EXPECT_EQ(*accessed_node_2, node_2);
    EXPECT_TRUE(timestamped_nodes.Contains(timestamp_2));
  }

  // Remove element 1
  EXPECT_TRUE(timestamped_nodes.Remove(timestamp_1));
  EXPECT_TRUE(timestamped_nodes.Node(timestamp_1) == boost::none);
  EXPECT_FALSE(timestamped_nodes.Contains(timestamp_1));
  EXPECT_TRUE(timestamped_nodes.Node(timestamp_2) != boost::none);
  EXPECT_TRUE(timestamped_nodes.Contains(timestamp_2));
  EXPECT_TRUE(timestamped_nodes.Node(timestamp_2) != boost::none);
  EXPECT_EQ(timestamped_nodes.size(), 1);
  EXPECT_FALSE(timestamped_nodes.empty());
  {
    const auto good_val = timestamped_nodes.Node(timestamp_2);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(*good_val, node_2);
  }

  // Bad Remove
  EXPECT_FALSE(timestamped_nodes.Remove(timestamp_1));
  EXPECT_FALSE(timestamped_nodes.Remove(100));

  // Remove element 2
  EXPECT_TRUE(timestamped_nodes.Remove(timestamp_2));
  EXPECT_TRUE(timestamped_nodes.Node(timestamp_1) == boost::none);
  EXPECT_FALSE(timestamped_nodes.Contains(timestamp_1));
  EXPECT_TRUE(timestamped_nodes.Node(timestamp_2) == boost::none);
  EXPECT_FALSE(timestamped_nodes.Contains(timestamp_2));
  EXPECT_EQ(timestamped_nodes.size(), 0);
  EXPECT_TRUE(timestamped_nodes.empty());
  {
    const auto bad_val = timestamped_nodes.Node(timestamp_2);
    EXPECT_TRUE(bad_val == boost::none);
  }
}
TEST(TimestampedNodesTester, OldestLatest) {
  std::shared_ptr<no::Values> values(new no::Values());
  TestCombinedNodes timestamped_nodes(values);
  // No elements
  {
    EXPECT_TRUE(timestamped_nodes.OldestTimestamp() == boost::none);
    EXPECT_TRUE(timestamped_nodes.OldestNode() == boost::none);
    EXPECT_TRUE(timestamped_nodes.LatestTimestamp() == boost::none);
    EXPECT_TRUE(timestamped_nodes.LatestNode() == boost::none);
  }
  const CombinedObject node_1 = CombinedObject::Random();
  const localization_common::Time timestamp_1 = 1.0;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_1, node_1).size(), 2);
  // 1 element
  {
    const auto oldest_timestamp = timestamped_nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_1);
    const auto latest_timestamp = timestamped_nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_1);

    const auto oldest_node = timestamped_nodes.OldestNode();
    ASSERT_TRUE(oldest_node != boost::none);
    EXPECT_EQ(*oldest_node, node_1);
    const auto latest_node = timestamped_nodes.LatestNode();
    ASSERT_TRUE(latest_node != boost::none);
    EXPECT_EQ(*latest_node, node_1);
  }
  // 2 elements
  const CombinedObject node_2 = CombinedObject::Random();
  const localization_common::Time timestamp_2 = 3.3;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_2, node_2).size(), 2);
  {
    const auto oldest_timestamp = timestamped_nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_1);
    const auto latest_timestamp = timestamped_nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_2);

    const auto oldest_node = timestamped_nodes.OldestNode();
    ASSERT_TRUE(oldest_node != boost::none);
    EXPECT_EQ(*oldest_node, node_1);
    const auto latest_node = timestamped_nodes.LatestNode();
    ASSERT_TRUE(latest_node != boost::none);
    EXPECT_EQ(*latest_node, node_2);
  }

  // 3 elements
  const CombinedObject node_3 = CombinedObject::Random();
  const localization_common::Time timestamp_3 = 19.3;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_3, node_3).size(), 2);
  {
    const auto oldest_timestamp = timestamped_nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_1);
    const auto latest_timestamp = timestamped_nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_3);

    const auto oldest_node = timestamped_nodes.OldestNode();
    ASSERT_TRUE(oldest_node != boost::none);
    EXPECT_EQ(*oldest_node, node_1);
    const auto latest_node = timestamped_nodes.LatestNode();
    ASSERT_TRUE(latest_node != boost::none);
    EXPECT_EQ(*latest_node, node_3);
  }

  ASSERT_TRUE(timestamped_nodes.Remove(timestamp_1));
  {
    const auto oldest_timestamp = timestamped_nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_2);
    const auto latest_timestamp = timestamped_nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_3);

    const auto oldest_node = timestamped_nodes.OldestNode();
    ASSERT_TRUE(oldest_node != boost::none);
    EXPECT_EQ(*oldest_node, node_2);
    const auto latest_node = timestamped_nodes.LatestNode();
    ASSERT_TRUE(latest_node != boost::none);
    EXPECT_EQ(*latest_node, node_3);
  }

  ASSERT_TRUE(timestamped_nodes.Remove(timestamp_3));
  {
    const auto oldest_timestamp = timestamped_nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_2);
    const auto latest_timestamp = timestamped_nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_2);

    const auto oldest_node = timestamped_nodes.OldestNode();
    ASSERT_TRUE(oldest_node != boost::none);
    EXPECT_EQ(*oldest_node, node_2);
    const auto latest_node = timestamped_nodes.LatestNode();
    ASSERT_TRUE(latest_node != boost::none);
    EXPECT_EQ(*latest_node, node_2);
  }

  ASSERT_TRUE(timestamped_nodes.Remove(timestamp_2));
  {
    EXPECT_TRUE(timestamped_nodes.OldestTimestamp() == boost::none);
    EXPECT_TRUE(timestamped_nodes.OldestNode() == boost::none);
    EXPECT_TRUE(timestamped_nodes.LatestTimestamp() == boost::none);
    EXPECT_TRUE(timestamped_nodes.LatestNode() == boost::none);
  }
}

TEST(TimestampedNodesTester, LowerAndUpperBounds) {
  std::shared_ptr<no::Values> values(new no::Values());
  TestCombinedNodes timestamped_nodes(values);
  // No elements
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(1.0);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.first == boost::none);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.second == boost::none);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(1.0);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first == boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second == boost::none);
  }

  // 1 element
  const CombinedObject node_1 = CombinedObject::Random();
  const localization_common::Time timestamp_1 = 37.0;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_1, node_1).size(), 2);
  // 1 element below
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(10.0);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(10.0);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.second), node_1);
  }
  // 1 element above
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(57.3);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_1);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.second == boost::none);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(57.3);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.first), node_1);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second == boost::none);
  }
  // 1 element equal
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(timestamp_1);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_1);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(timestamp_1);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.first), node_1);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.second), node_1);
  }

  // 2 elements
  const CombinedObject node_2 = CombinedObject::Random();
  const localization_common::Time timestamp_2 = 2.33;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_2, node_2).size(), 2);
  // 2 elements below
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(1.1);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_2);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(1.1);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.second), node_2);
  }
  // 2 elements above
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(111.3);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_1);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.second == boost::none);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(111.3);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.first), node_1);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second == boost::none);
  }
  // 2 elements equal lower
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(timestamp_2);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_2);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_2);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(timestamp_2);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.first), node_2);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.second), node_2);
  }
  // 2 elements equal upper
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(timestamp_1);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_1);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(timestamp_1);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.first), node_1);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.second), node_1);
  }
  // 2 elements between
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(15.1);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_2);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(15.1);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.first), node_2);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.second), node_1);
  }

  // 3 elements
  const CombinedObject node_3 = CombinedObject::Random();
  const localization_common::Time timestamp_3 = 14.1;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_3, node_3).size(), 2);
  // 3 elements lower between
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(7.11);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_2);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_3);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(7.11);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.first), node_2);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.second), node_3);
  }
  // 3 elements upper between
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(22.22);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_3);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(22.22);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.first), node_3);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.second), node_1);
  }
}

TEST(TimestampedNodesTester, LowerBoundOrEqual) {
  std::shared_ptr<no::Values> values(new no::Values());
  TestCombinedNodes timestamped_nodes(values);
  const CombinedObject node_1 = CombinedObject::Random();
  const localization_common::Time timestamp_1 = 3.1;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_1, node_1).size(), 2);
  const CombinedObject node_2 = CombinedObject::Random();
  const localization_common::Time timestamp_2 = 5.78;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_2, node_2).size(), 2);
  const CombinedObject node_3 = CombinedObject::Random();
  const localization_common::Time timestamp_3 = 7.88;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_3, node_3).size(), 2);

  const auto too_low_node = timestamped_nodes.LowerBoundOrEqual(1.23);
  EXPECT_TRUE(too_low_node == boost::none);
  const auto lowest_node = timestamped_nodes.LowerBoundOrEqual(4.11);
  ASSERT_TRUE(lowest_node != boost::none);
  EXPECT_EQ(lowest_node->value, node_1);
  EXPECT_EQ(lowest_node->timestamp, timestamp_1);
  const auto middle_node = timestamped_nodes.LowerBoundOrEqual(6.61);
  ASSERT_TRUE(middle_node != boost::none);
  EXPECT_EQ(middle_node->value, node_2);
  EXPECT_EQ(middle_node->timestamp, timestamp_2);
  const auto upper_node = timestamped_nodes.LowerBoundOrEqual(900);
  ASSERT_TRUE(upper_node != boost::none);
  EXPECT_EQ(upper_node->value, node_3);
  EXPECT_EQ(upper_node->timestamp, timestamp_3);
  const auto equal_node = timestamped_nodes.LowerBoundOrEqual(timestamp_2);
  ASSERT_TRUE(equal_node != boost::none);
  EXPECT_EQ(equal_node->value, node_2);
  EXPECT_EQ(equal_node->timestamp, timestamp_2);
}

TEST(TimestampedNodesTester, Closest) {
  std::shared_ptr<no::Values> values(new no::Values());
  TestCombinedNodes timestamped_nodes(values);
  const CombinedObject node_1 = CombinedObject::Random();
  const localization_common::Time timestamp_1 = 3.1;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_1, node_1).size(), 2);
  const CombinedObject node_2 = CombinedObject::Random();
  const localization_common::Time timestamp_2 = 5.78;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_2, node_2).size(), 2);
  const CombinedObject node_3 = CombinedObject::Random();
  const localization_common::Time timestamp_3 = 7.88;
  ASSERT_EQ(timestamped_nodes.Add(timestamp_3, node_3).size(), 2);

  const auto below_lowest_node = timestamped_nodes.ClosestNode(1.23);
  EXPECT_TRUE(below_lowest_node != boost::none);
  EXPECT_EQ(*below_lowest_node, node_1);
  const auto above_lowest_node = timestamped_nodes.ClosestNode(4.11);
  ASSERT_TRUE(above_lowest_node != boost::none);
  EXPECT_EQ(*above_lowest_node, node_1);
  const auto below_middle_node = timestamped_nodes.ClosestNode(5.61);
  ASSERT_TRUE(below_middle_node != boost::none);
  EXPECT_EQ(*below_middle_node, node_2);
  const auto above_middle_node = timestamped_nodes.ClosestNode(6.61);
  ASSERT_TRUE(above_middle_node != boost::none);
  EXPECT_EQ(*above_middle_node, node_2);
  const auto below_upper_node = timestamped_nodes.ClosestNode(7.61);
  ASSERT_TRUE(below_upper_node != boost::none);
  EXPECT_EQ(*below_upper_node, node_3);
  const auto above_upper_node = timestamped_nodes.ClosestNode(8.61);
  ASSERT_TRUE(above_upper_node != boost::none);
  EXPECT_EQ(*above_upper_node, node_3);
  const auto equal_node = timestamped_nodes.ClosestNode(timestamp_2);
  ASSERT_TRUE(equal_node != boost::none);
  EXPECT_EQ(*equal_node, node_2);
}

TEST(TimestampedNodesTester, OldKeysTimestampsAndNodes) {
  std::shared_ptr<no::Values> values(new no::Values());
  TestCombinedNodes timestamped_nodes(values);
  const double t0 = 0;
  const CombinedObject node_0 = CombinedObject::Random();
  const double t1 = 1.001;
  const CombinedObject node_1 = CombinedObject::Random();
  const double t2 = 2.100;
  const CombinedObject node_2 = CombinedObject::Random();
  const double t3 = 3.0222;
  const CombinedObject node_3 = CombinedObject::Random();
  const int k0 = 1;
  const int k1 = 2;
  const int k2 = 3;
  const int k3 = 4;
  const int k4 = 5;
  const int k5 = 6;
  const int k6 = 7;
  const int k7 = 8;
  ASSERT_EQ(timestamped_nodes.Add(t0, node_0).size(), 2);
  ASSERT_EQ(timestamped_nodes.Add(t1, node_1).size(), 2);
  ASSERT_EQ(timestamped_nodes.Add(t2, node_2).size(), 2);
  ASSERT_EQ(timestamped_nodes.Add(t3, node_3).size(), 2);
  {
    const auto old_keys = timestamped_nodes.OldKeys(0);
    EXPECT_EQ(old_keys.size(), 0);
    const auto old_nodes = timestamped_nodes.OldNodes(0);
    EXPECT_EQ(old_nodes.size(), 0);
  }
  {
    const auto old_keys = timestamped_nodes.OldKeys(0.1);
    EXPECT_EQ(old_keys.size(), 2);
    EXPECT_EQ(old_keys[0], k0);
    EXPECT_EQ(old_keys[1], k1);
    const auto old_nodes = timestamped_nodes.OldNodes(0.1);
    ASSERT_EQ(old_nodes.size(), 1);
    EXPECT_EQ(old_nodes[0], node_0);
  }
  {
    const auto old_keys = timestamped_nodes.OldKeys(1.7);
    EXPECT_EQ(old_keys.size(), 4);
    EXPECT_EQ(old_keys[0], k0);
    EXPECT_EQ(old_keys[1], k1);
    EXPECT_EQ(old_keys[2], k2);
    EXPECT_EQ(old_keys[3], k3);
    const auto old_nodes = timestamped_nodes.OldNodes(1.7);
    ASSERT_EQ(old_nodes.size(), 2);
    EXPECT_EQ(old_nodes[0], node_0);
    EXPECT_EQ(old_nodes[1], node_1);
  }
  {
    const auto old_keys = timestamped_nodes.OldKeys(2.333);
    EXPECT_EQ(old_keys.size(), 6);
    EXPECT_EQ(old_keys[0], k0);
    EXPECT_EQ(old_keys[1], k1);
    EXPECT_EQ(old_keys[2], k2);
    EXPECT_EQ(old_keys[3], k3);
    EXPECT_EQ(old_keys[4], k4);
    EXPECT_EQ(old_keys[5], k5);
    const auto old_nodes = timestamped_nodes.OldNodes(2.333);
    ASSERT_EQ(old_nodes.size(), 3);
    EXPECT_EQ(old_nodes[0], node_0);
    EXPECT_EQ(old_nodes[1], node_1);
    EXPECT_EQ(old_nodes[2], node_2);
  }
  {
    const auto old_keys = timestamped_nodes.OldKeys(1999);
    EXPECT_EQ(old_keys.size(), 8);
    EXPECT_EQ(old_keys[0], k0);
    EXPECT_EQ(old_keys[1], k1);
    EXPECT_EQ(old_keys[2], k2);
    EXPECT_EQ(old_keys[3], k3);
    EXPECT_EQ(old_keys[4], k4);
    EXPECT_EQ(old_keys[5], k5);
    EXPECT_EQ(old_keys[6], k6);
    EXPECT_EQ(old_keys[7], k7);
    const auto old_nodes = timestamped_nodes.OldNodes(1999);
    ASSERT_EQ(old_nodes.size(), 4);
    EXPECT_EQ(old_nodes[0], node_0);
    EXPECT_EQ(old_nodes[1], node_1);
    EXPECT_EQ(old_nodes[2], node_2);
    EXPECT_EQ(old_nodes[3], node_3);
  }
}

TEST(TimestampedNodesTester, RemoveOldNodes) {
  {
    std::shared_ptr<no::Values> values(new no::Values());
    TestCombinedNodes timestamped_nodes(values);
    const double t0 = 0;
    const CombinedObject node_0 = CombinedObject::Random();
    const double t1 = 1.001;
    const CombinedObject node_1 = CombinedObject::Random();
    const double t2 = 2.100;
    const CombinedObject node_2 = CombinedObject::Random();
    const double t3 = 3.0222;
    const CombinedObject node_3 = CombinedObject::Random();
    ASSERT_EQ(timestamped_nodes.Add(t0, node_0).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t1, node_1).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t2, node_2).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t3, node_3).size(), 2);
    const int num_nodes_removed = timestamped_nodes.RemoveOldNodes(0);
    EXPECT_EQ(num_nodes_removed, 0);
    EXPECT_EQ(timestamped_nodes.size(), 4);
  }

  {
    std::shared_ptr<no::Values> values(new no::Values());
    TestCombinedNodes timestamped_nodes(values);
    const double t0 = 0;
    const CombinedObject node_0 = CombinedObject::Random();
    const double t1 = 1.001;
    const CombinedObject node_1 = CombinedObject::Random();
    const double t2 = 2.100;
    const CombinedObject node_2 = CombinedObject::Random();
    const double t3 = 3.0222;
    const CombinedObject node_3 = CombinedObject::Random();
    ASSERT_EQ(timestamped_nodes.Add(t0, node_0).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t1, node_1).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t2, node_2).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t3, node_3).size(), 2);
    const int num_nodes_removed = timestamped_nodes.RemoveOldNodes(0.1);
    EXPECT_EQ(num_nodes_removed, 1);
    EXPECT_EQ(timestamped_nodes.size(), 3);
    const auto timestamps = timestamped_nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t1);
    EXPECT_EQ(timestamps[1], t2);
    EXPECT_EQ(timestamps[2], t3);
  }
  {
    std::shared_ptr<no::Values> values(new no::Values());
    TestCombinedNodes timestamped_nodes(values);
    const double t0 = 0;
    const CombinedObject node_0 = CombinedObject::Random();
    const double t1 = 1.001;
    const CombinedObject node_1 = CombinedObject::Random();
    const double t2 = 2.100;
    const CombinedObject node_2 = CombinedObject::Random();
    const double t3 = 3.0222;
    const CombinedObject node_3 = CombinedObject::Random();
    ASSERT_EQ(timestamped_nodes.Add(t0, node_0).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t1, node_1).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t2, node_2).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t3, node_3).size(), 2);
    const int num_nodes_removed = timestamped_nodes.RemoveOldNodes(1.334);
    EXPECT_EQ(num_nodes_removed, 2);
    EXPECT_EQ(timestamped_nodes.size(), 2);
    const auto timestamps = timestamped_nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t2);
    EXPECT_EQ(timestamps[1], t3);
  }

  {
    std::shared_ptr<no::Values> values(new no::Values());
    TestCombinedNodes timestamped_nodes(values);
    const double t0 = 0;
    const CombinedObject node_0 = CombinedObject::Random();
    const double t1 = 1.001;
    const CombinedObject node_1 = CombinedObject::Random();
    const double t2 = 2.100;
    const CombinedObject node_2 = CombinedObject::Random();
    const double t3 = 3.0222;
    const CombinedObject node_3 = CombinedObject::Random();
    ASSERT_EQ(timestamped_nodes.Add(t0, node_0).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t1, node_1).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t2, node_2).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t3, node_3).size(), 2);
    const int num_nodes_removed = timestamped_nodes.RemoveOldNodes(2.78);
    EXPECT_EQ(num_nodes_removed, 3);
    EXPECT_EQ(timestamped_nodes.size(), 1);
    const auto timestamps = timestamped_nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t3);
  }

  {
    std::shared_ptr<no::Values> values(new no::Values());
    TestCombinedNodes timestamped_nodes(values);
    const double t0 = 0;
    const CombinedObject node_0 = CombinedObject::Random();
    const double t1 = 1.001;
    const CombinedObject node_1 = CombinedObject::Random();
    const double t2 = 2.100;
    const CombinedObject node_2 = CombinedObject::Random();
    const double t3 = 3.0222;
    const CombinedObject node_3 = CombinedObject::Random();
    ASSERT_EQ(timestamped_nodes.Add(t0, node_0).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t1, node_1).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t2, node_2).size(), 2);
    ASSERT_EQ(timestamped_nodes.Add(t3, node_3).size(), 2);
    const int num_nodes_removed = timestamped_nodes.RemoveOldNodes(1923.78);
    EXPECT_EQ(num_nodes_removed, 4);
    EXPECT_EQ(timestamped_nodes.size(), 0);
  }
}

TEST(TimestampedNodesTester, Duration) {
  std::shared_ptr<no::Values> values(new no::Values());
  TestCombinedNodes timestamped_nodes(values);
  const CombinedObject node_1 = CombinedObject::Random();
  const CombinedObject node_2 = CombinedObject::Random();
  const CombinedObject node_3 = CombinedObject::Random();
  EXPECT_EQ(timestamped_nodes.Duration(), 0);
  ASSERT_EQ(timestamped_nodes.Add(1.0, node_1).size(), 2);
  EXPECT_EQ(timestamped_nodes.Duration(), 0);
  ASSERT_EQ(timestamped_nodes.Add(2.0, node_2).size(), 2);
  EXPECT_NEAR(timestamped_nodes.Duration(), 1, 1e-6);
  ASSERT_EQ(timestamped_nodes.Add(3.0, node_3).size(), 2);
  EXPECT_NEAR(timestamped_nodes.Duration(), 2, 1e-6);
}

TEST(TimestampedNodesTester, Timestamps) {
  std::shared_ptr<no::Values> values(new no::Values());
  TestCombinedNodes timestamped_nodes(values);
  {
    const auto timestamps = timestamped_nodes.Timestamps();
    EXPECT_EQ(timestamps.size(), 0);
  }
  const double t0 = 0;
  const double t1 = 1;
  const double t2 = 2;
  const double t3 = 3;
  const CombinedObject node_0 = CombinedObject::Random();
  const CombinedObject node_1 = CombinedObject::Random();
  const CombinedObject node_2 = CombinedObject::Random();
  const CombinedObject node_3 = CombinedObject::Random();
  ASSERT_EQ(timestamped_nodes.Add(t0, node_0).size(), 2);
  ASSERT_EQ(timestamped_nodes.Add(t1, node_1).size(), 2);
  ASSERT_EQ(timestamped_nodes.Add(t2, node_2).size(), 2);
  ASSERT_EQ(timestamped_nodes.Add(t3, node_3).size(), 2);
  {
    const auto timestamps = timestamped_nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t0);
    EXPECT_EQ(timestamps[1], t1);
    EXPECT_EQ(timestamps[2], t2);
    EXPECT_EQ(timestamps[3], t3);
  }
}

TEST(TimestampedNodesTester, Serialization) {
  const TestCombinedNodes nodes;
  const auto serialized_nodes = gtsam::serializeBinary(nodes);
  TestCombinedNodes deserialized_nodes;
  gtsam::deserializeBinary(serialized_nodes, deserialized_nodes);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
