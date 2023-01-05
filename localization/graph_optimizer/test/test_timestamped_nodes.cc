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

#include <graph_optimizer/timestamped_nodes.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>

#include <gtest/gtest.h>

namespace go = graph_optimizer;
namespace lc = localization_common;

TEST(TimestampedNodesTester, AddRemoveContainsEmptySize) {
  std::shared_ptr<go::Nodes> nodes(new go::Nodes());
  go::TimestampedNodes<double> timestamped_nodes(nodes);
  EXPECT_EQ(timestamped_nodes.size(), 0);
  EXPECT_TRUE(timestamped_nodes.empty());

  // Add element 1
  const double node_1 = 100.3;
  const localization_common::Time timestamp_1 = 1.0;
  EXPECT_TRUE(timestamped_nodes.Add(timestamp_1, node_1) != boost::none);
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
  const double node_2 = 100.3;
  const localization_common::Time timestamp_2 = 3.3;
  EXPECT_TRUE(timestamped_nodes.Add(timestamp_2, node_2) != boost::none);
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
  std::shared_ptr<go::Nodes> nodes(new go::Nodes());
  go::TimestampedNodes<double> timestamped_nodes(nodes);
  // No elements
  {
    EXPECT_TRUE(timestamped_nodes.OldestTimestamp() == boost::none);
    EXPECT_TRUE(timestamped_nodes.OldestNode() == boost::none);
    EXPECT_TRUE(timestamped_nodes.LatestTimestamp() == boost::none);
    EXPECT_TRUE(timestamped_nodes.LatestNode() == boost::none);
  }
  const double node_1 = 101.0;
  const localization_common::Time timestamp_1 = 1.0;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_1, node_1) != boost::none);
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
  const double node_2 = 100.3;
  const localization_common::Time timestamp_2 = 3.3;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_2, node_2) != boost::none);
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
  const double node_3 = 2100.3;
  const localization_common::Time timestamp_3 = 19.3;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_3, node_3) != boost::none);
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
  std::shared_ptr<go::Nodes> nodes(new go::Nodes());
  go::TimestampedNodes<double> timestamped_nodes(nodes);
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
  const double node_1 = -77.0;
  const localization_common::Time timestamp_1 = 37.0;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_1, node_1) != boost::none);
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
    EXPECT_TRUE(lower_and_upper_bound_timestamps.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(timestamp_1);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.second), node_1);
  }

  // 2 elements
  const double node_2 = 512.0;
  const localization_common::Time timestamp_2 = 2.33;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_2, node_2) != boost::none);
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
    EXPECT_TRUE(lower_and_upper_bound_timestamps.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_2);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(timestamp_2);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.second), node_2);
  }
  // 2 elements equal upper
  {
    const auto lower_and_upper_bound_timestamps = timestamped_nodes.LowerAndUpperBoundTimestamps(timestamp_1);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_2);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = timestamped_nodes.LowerAndUpperBoundNodes(timestamp_1);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_nodes.first), node_2);
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
  const double node_3 = 291.1;
  const localization_common::Time timestamp_3 = 14.1;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_3, node_3) != boost::none);
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
  std::shared_ptr<go::Nodes> nodes(new go::Nodes());
  go::TimestampedNodes<double> timestamped_nodes(nodes);
  const double node_1 = 1.23;
  const localization_common::Time timestamp_1 = 3.1;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_1, node_1) != boost::none);
  const double node_2 = 2.22;
  const localization_common::Time timestamp_2 = 5.78;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_2, node_2) != boost::none);
  const double node_3 = 3.98;
  const localization_common::Time timestamp_3 = 7.88;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_3, node_3) != boost::none);
  const auto too_low_timestamp = timestamped_nodes.LowerBoundOrEqualTimestamp(1.23);
  EXPECT_TRUE(too_low_timestamp == boost::none);
  const auto lowest_timestamp = timestamped_nodes.LowerBoundOrEqualTimestamp(4.11);
  ASSERT_TRUE(lowest_timestamp != boost::none);
  EXPECT_EQ(*lowest_timestamp, timestamp_1);
  const auto middle_timestamp = timestamped_nodes.LowerBoundOrEqualTimestamp(6.61);
  ASSERT_TRUE(middle_timestamp != boost::none);
  EXPECT_EQ(*middle_timestamp, timestamp_2);
  const auto upper_timestamp = timestamped_nodes.LowerBoundOrEqualTimestamp(900);
  ASSERT_TRUE(upper_timestamp != boost::none);
  EXPECT_EQ(*upper_timestamp, timestamp_3);
  const auto equal_timestamp = timestamped_nodes.LowerBoundOrEqualTimestamp(timestamp_2);
  ASSERT_TRUE(equal_timestamp != boost::none);
  EXPECT_EQ(*equal_timestamp, timestamp_2);

  const auto too_low_node = timestamped_nodes.LowerBoundOrEqualNode(1.23);
  EXPECT_TRUE(too_low_node == boost::none);
  const auto lowest_node = timestamped_nodes.LowerBoundOrEqualNode(4.11);
  ASSERT_TRUE(lowest_node != boost::none);
  EXPECT_EQ(*lowest_node, node_1);
  const auto middle_node = timestamped_nodes.LowerBoundOrEqualNode(6.61);
  ASSERT_TRUE(middle_node != boost::none);
  EXPECT_EQ(*middle_node, node_2);
  const auto upper_node = timestamped_nodes.LowerBoundOrEqualNode(900);
  ASSERT_TRUE(upper_node != boost::none);
  EXPECT_EQ(*upper_node, node_3);
  const auto equal_node = timestamped_nodes.LowerBoundOrEqualNode(timestamp_2);
  ASSERT_TRUE(equal_node != boost::none);
  EXPECT_EQ(*equal_node, node_2);
}

TEST(TimestampedNodesTester, Closest) {
  std::shared_ptr<go::Nodes> nodes(new go::Nodes());
  go::TimestampedNodes<double> timestamped_nodes(nodes);
  const double node_1 = 1.23;
  const localization_common::Time timestamp_1 = 3.1;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_1, node_1) != boost::none);
  const double node_2 = 2.22;
  const localization_common::Time timestamp_2 = 5.78;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_2, node_2) != boost::none);
  const double node_3 = 3.98;
  const localization_common::Time timestamp_3 = 7.88;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_3, node_3) != boost::none);
  const auto below_lowest_timestamp = timestamped_nodes.ClosestTimestamp(1.23);
  ASSERT_TRUE(below_lowest_timestamp != boost::none);
  EXPECT_EQ(*below_lowest_timestamp, timestamp_1);
  const auto above_lowest_timestamp = timestamped_nodes.ClosestTimestamp(4.11);
  ASSERT_TRUE(above_lowest_timestamp != boost::none);
  EXPECT_EQ(*above_lowest_timestamp, timestamp_1);
  const auto below_middle_timestamp = timestamped_nodes.ClosestTimestamp(5.61);
  ASSERT_TRUE(below_middle_timestamp != boost::none);
  EXPECT_EQ(*below_middle_timestamp, timestamp_2);
  const auto above_middle_timestamp = timestamped_nodes.ClosestTimestamp(6.61);
  ASSERT_TRUE(above_middle_timestamp != boost::none);
  EXPECT_EQ(*above_middle_timestamp, timestamp_2);
  const auto below_upper_timestamp = timestamped_nodes.ClosestTimestamp(7.61);
  ASSERT_TRUE(below_upper_timestamp != boost::none);
  EXPECT_EQ(*below_upper_timestamp, timestamp_3);
  const auto above_upper_timestamp = timestamped_nodes.ClosestTimestamp(8.61);
  ASSERT_TRUE(above_upper_timestamp != boost::none);
  EXPECT_EQ(*above_upper_timestamp, timestamp_3);
  const auto equal_timestamp = timestamped_nodes.ClosestTimestamp(timestamp_2);
  ASSERT_TRUE(equal_timestamp != boost::none);
  EXPECT_EQ(*equal_timestamp, timestamp_2);

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
  std::shared_ptr<go::Nodes> nodes(new go::Nodes());
  go::TimestampedNodes<double> timestamped_nodes(nodes);
  const double t0 = 0;
  const double n0 = lc::RandomDouble();
  const int k0 = 1;
  const double t1 = 1.001;
  const double n1 = lc::RandomDouble();
  const int k1 = 2;
  const double t2 = 2.100;
  const double n2 = lc::RandomDouble();
  const int k2 = 3;
  const double t3 = 3.0222;
  const double n3 = lc::RandomDouble();
  const int k3 = 4;
  ASSERT_TRUE(timestamped_nodes.Add(t0, n0) != boost::none);
  ASSERT_TRUE(timestamped_nodes.Add(t1, n1) != boost::none);
  ASSERT_TRUE(timestamped_nodes.Add(t2, n2) != boost::none);
  ASSERT_TRUE(timestamped_nodes.Add(t3, n3) != boost::none);
  {
    const auto old_keys = timestamped_nodes.OldKeys(0);
    EXPECT_EQ(old_keys.size(), 0);
    const auto old_nodes = timestamped_nodes.OldNodes(0);
    EXPECT_EQ(old_nodes.size(), 0);
    const auto old_timestamps = timestamped_nodes.OldTimestamps(0);
    EXPECT_EQ(old_timestamps.size(), 0);
  }
  {
    const auto old_keys = timestamped_nodes.OldKeys(0.1);
    EXPECT_EQ(old_keys.size(), 1);
    EXPECT_EQ(old_keys[0], k0);
    const auto old_nodes = timestamped_nodes.OldNodes(0.1);
    ASSERT_EQ(old_nodes.size(), 1);
    EXPECT_EQ(old_nodes[0], n0);
    const auto old_timestamps = timestamped_nodes.OldTimestamps(0.1);
    ASSERT_EQ(old_timestamps.size(), 1);
    EXPECT_EQ(old_timestamps[0], t0);
  }
  {
    const auto old_keys = timestamped_nodes.OldKeys(1.7);
    EXPECT_EQ(old_keys.size(), 2);
    EXPECT_EQ(old_keys[0], k0);
    EXPECT_EQ(old_keys[1], k1);
    const auto old_nodes = timestamped_nodes.OldNodes(1.7);
    ASSERT_EQ(old_nodes.size(), 2);
    EXPECT_EQ(old_nodes[0], n0);
    EXPECT_EQ(old_nodes[1], n1);
    const auto old_timestamps = timestamped_nodes.OldTimestamps(1.7);
    ASSERT_EQ(old_timestamps.size(), 2);
    EXPECT_EQ(old_timestamps[0], t0);
    EXPECT_EQ(old_timestamps[1], t1);
  }
  {
    const auto old_keys = timestamped_nodes.OldKeys(2.333);
    EXPECT_EQ(old_keys.size(), 3);
    EXPECT_EQ(old_keys[0], k0);
    EXPECT_EQ(old_keys[1], k1);
    EXPECT_EQ(old_keys[2], k2);
    const auto old_nodes = timestamped_nodes.OldNodes(2.333);
    ASSERT_EQ(old_nodes.size(), 3);
    EXPECT_EQ(old_nodes[0], n0);
    EXPECT_EQ(old_nodes[1], n1);
    EXPECT_EQ(old_nodes[2], n2);
    const auto old_timestamps = timestamped_nodes.OldTimestamps(2.333);
    ASSERT_EQ(old_timestamps.size(), 3);
    EXPECT_EQ(old_timestamps[0], t0);
    EXPECT_EQ(old_timestamps[1], t1);
    EXPECT_EQ(old_timestamps[2], t2);
  }
  {
    const auto old_keys = timestamped_nodes.OldKeys(1999);
    EXPECT_EQ(old_keys.size(), 4);
    EXPECT_EQ(old_keys[0], k0);
    EXPECT_EQ(old_keys[1], k1);
    EXPECT_EQ(old_keys[2], k2);
    EXPECT_EQ(old_keys[3], k3);
    const auto old_nodes = timestamped_nodes.OldNodes(1999);
    ASSERT_EQ(old_nodes.size(), 4);
    EXPECT_EQ(old_nodes[0], n0);
    EXPECT_EQ(old_nodes[1], n1);
    EXPECT_EQ(old_nodes[2], n2);
    EXPECT_EQ(old_nodes[3], n3);
    const auto old_timestamps = timestamped_nodes.OldTimestamps(1999);
    ASSERT_EQ(old_timestamps.size(), 4);
    EXPECT_EQ(old_timestamps[0], t0);
    EXPECT_EQ(old_timestamps[1], t1);
    EXPECT_EQ(old_timestamps[2], t2);
    EXPECT_EQ(old_timestamps[3], t3);
  }
}

TEST(TimestampedNodesTester, RemoveOldNodes) {
  {
    std::shared_ptr<go::Nodes> nodes(new go::Nodes());
    go::TimestampedNodes<double> timestamped_nodes(nodes);
    const double t0 = 0;
    const double n0 = lc::RandomDouble();
    const double t1 = 1.001;
    const double n1 = lc::RandomDouble();
    const double t2 = 2.100;
    const double n2 = lc::RandomDouble();
    const double t3 = 3.0222;
    const double n3 = lc::RandomDouble();
    ASSERT_TRUE(timestamped_nodes.Add(t0, n0) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t1, n1) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t2, n2) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t3, n3) != boost::none);
    const int num_nodes_removed = timestamped_nodes.RemoveOldNodes(0);
    EXPECT_EQ(num_nodes_removed, 0);
    EXPECT_EQ(timestamped_nodes.size(), 4);
  }

  {
    std::shared_ptr<go::Nodes> nodes(new go::Nodes());
    go::TimestampedNodes<double> timestamped_nodes(nodes);
    const double t0 = 0;
    const double n0 = lc::RandomDouble();
    const double t1 = 1.001;
    const double n1 = lc::RandomDouble();
    const double t2 = 2.100;
    const double n2 = lc::RandomDouble();
    const double t3 = 3.0222;
    const double n3 = lc::RandomDouble();
    ASSERT_TRUE(timestamped_nodes.Add(t0, n0) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t1, n1) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t2, n2) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t3, n3) != boost::none);
    const int num_nodes_removed = timestamped_nodes.RemoveOldNodes(0.1);
    EXPECT_EQ(num_nodes_removed, 1);
    EXPECT_EQ(timestamped_nodes.size(), 3);
    const auto timestamps = timestamped_nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t1);
    EXPECT_EQ(timestamps[1], t2);
    EXPECT_EQ(timestamps[2], t3);
  }
  {
    std::shared_ptr<go::Nodes> nodes(new go::Nodes());
    go::TimestampedNodes<double> timestamped_nodes(nodes);
    const double t0 = 0;
    const double n0 = lc::RandomDouble();
    const double t1 = 1.001;
    const double n1 = lc::RandomDouble();
    const double t2 = 2.100;
    const double n2 = lc::RandomDouble();
    const double t3 = 3.0222;
    const double n3 = lc::RandomDouble();
    ASSERT_TRUE(timestamped_nodes.Add(t0, n0) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t1, n1) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t2, n2) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t3, n3) != boost::none);
    const int num_nodes_removed = timestamped_nodes.RemoveOldNodes(1.334);
    EXPECT_EQ(num_nodes_removed, 2);
    EXPECT_EQ(timestamped_nodes.size(), 2);
    const auto timestamps = timestamped_nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t2);
    EXPECT_EQ(timestamps[1], t3);
  }

  {
    std::shared_ptr<go::Nodes> nodes(new go::Nodes());
    go::TimestampedNodes<double> timestamped_nodes(nodes);
    const double t0 = 0;
    const double n0 = lc::RandomDouble();
    const double t1 = 1.001;
    const double n1 = lc::RandomDouble();
    const double t2 = 2.100;
    const double n2 = lc::RandomDouble();
    const double t3 = 3.0222;
    const double n3 = lc::RandomDouble();
    ASSERT_TRUE(timestamped_nodes.Add(t0, n0) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t1, n1) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t2, n2) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t3, n3) != boost::none);
    const int num_nodes_removed = timestamped_nodes.RemoveOldNodes(2.78);
    EXPECT_EQ(num_nodes_removed, 3);
    EXPECT_EQ(timestamped_nodes.size(), 1);
    const auto timestamps = timestamped_nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t3);
  }

  {
    std::shared_ptr<go::Nodes> nodes(new go::Nodes());
    go::TimestampedNodes<double> timestamped_nodes(nodes);
    const double t0 = 0;
    const double n0 = lc::RandomDouble();
    const double t1 = 1.001;
    const double n1 = lc::RandomDouble();
    const double t2 = 2.100;
    const double n2 = lc::RandomDouble();
    const double t3 = 3.0222;
    const double n3 = lc::RandomDouble();
    ASSERT_TRUE(timestamped_nodes.Add(t0, n0) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t1, n1) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t2, n2) != boost::none);
    ASSERT_TRUE(timestamped_nodes.Add(t3, n3) != boost::none);
    const int num_nodes_removed = timestamped_nodes.RemoveOldNodes(1923.78);
    EXPECT_EQ(num_nodes_removed, 4);
    EXPECT_EQ(timestamped_nodes.size(), 0);
  }
}

TEST(TimestampedNodesTester, Duration) {
  std::shared_ptr<go::Nodes> nodes(new go::Nodes());
  go::TimestampedNodes<double> timestamped_nodes(nodes);
  EXPECT_EQ(timestamped_nodes.Duration(), 0);
  ASSERT_TRUE(timestamped_nodes.Add(1.0, 1) != boost::none);
  EXPECT_EQ(timestamped_nodes.Duration(), 0);
  ASSERT_TRUE(timestamped_nodes.Add(2.0, 2) != boost::none);
  EXPECT_NEAR(timestamped_nodes.Duration(), 1, 1e-6);
  ASSERT_TRUE(timestamped_nodes.Add(3.0, 3) != boost::none);
  EXPECT_NEAR(timestamped_nodes.Duration(), 2, 1e-6);
}

TEST(TimestampedNodesTester, Timestamps) {
  std::shared_ptr<go::Nodes> nodes(new go::Nodes());
  go::TimestampedNodes<double> timestamped_nodes(nodes);
  {
    const auto timestamps = timestamped_nodes.Timestamps();
    EXPECT_EQ(timestamps.size(), 0);
  }
  const double t0 = 0;
  const double t1 = 1;
  const double t2 = 2;
  const double t3 = 3;
  ASSERT_TRUE(timestamped_nodes.Add(t0, t0) != boost::none);
  ASSERT_TRUE(timestamped_nodes.Add(t1, t1) != boost::none);
  ASSERT_TRUE(timestamped_nodes.Add(t2, t2) != boost::none);
  ASSERT_TRUE(timestamped_nodes.Add(t3, t3) != boost::none);
  {
    const auto timestamps = timestamped_nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t0);
    EXPECT_EQ(timestamps[1], t1);
    EXPECT_EQ(timestamps[2], t2);
    EXPECT_EQ(timestamps[3], t3);
  }
}

TEST(TimestampedNodesTester, Serialization) {
  const go::TimestampedNodes<double> nodes;
  const auto serialized_nodes = gtsam::serializeBinary(nodes);
  go::TimestampedNodes<double> deserialized_nodes;
  gtsam::deserializeBinary(serialized_nodes, deserialized_nodes);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
