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

#include <localization_common/timestamped_set.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>

#include <gtsam/base/serialization.h>

#include <gtest/gtest.h>

namespace lc = localization_common;

TEST(TimestampedSetTester, AddRemoveContainsEmptySize) {
  lc::TimestampedSet<double> timestamped_set;
  EXPECT_EQ(timestamped_set.size(), 0);
  EXPECT_TRUE(timestamped_set.empty());

  // Add element 1
  const double value_1 = 100.3;
  const localization_common::Time timestamp_1 = 1.0;
  EXPECT_TRUE(timestamped_set.Add(timestamp_1, value_1));
  EXPECT_EQ(timestamped_set.size(), 1);
  EXPECT_FALSE(timestamped_set.empty());
  {
    EXPECT_TRUE(timestamped_set.Get(2.0) == boost::none);
    const auto accessed_value = timestamped_set.Get(timestamp_1);
    ASSERT_TRUE(accessed_value != boost::none);
    EXPECT_EQ(accessed_value->value, value_1);
    EXPECT_EQ(accessed_value->timestamp, timestamp_1);
    EXPECT_TRUE(timestamped_set.Contains(timestamp_1));
  }

  // Add element 2
  const double value_2 = 100.3;
  const localization_common::Time timestamp_2 = 3.3;
  EXPECT_TRUE(timestamped_set.Add(timestamp_2, value_2));
  EXPECT_EQ(timestamped_set.size(), 2);
  EXPECT_FALSE(timestamped_set.empty());
  {
    EXPECT_TRUE(timestamped_set.Get(7.0) == boost::none);
    const auto accessed_value_1 = timestamped_set.Get(timestamp_1);
    ASSERT_TRUE(accessed_value_1 != boost::none);
    EXPECT_EQ(accessed_value_1->value, value_1);
    EXPECT_EQ(accessed_value_1->timestamp, timestamp_1);
    EXPECT_TRUE(timestamped_set.Contains(timestamp_1));
    const auto accessed_value_2 = timestamped_set.Get(timestamp_2);
    ASSERT_TRUE(accessed_value_2 != boost::none);
    EXPECT_EQ(accessed_value_2->value, value_2);
    EXPECT_EQ(accessed_value_2->timestamp, timestamp_2);
    EXPECT_TRUE(timestamped_set.Contains(timestamp_2));
  }

  // Remove element 1
  EXPECT_TRUE(timestamped_set.Remove(timestamp_1));
  EXPECT_TRUE(timestamped_set.Get(timestamp_1) == boost::none);
  EXPECT_FALSE(timestamped_set.Contains(timestamp_1));
  EXPECT_TRUE(timestamped_set.Get(timestamp_2) != boost::none);
  EXPECT_TRUE(timestamped_set.Contains(timestamp_2));
  EXPECT_TRUE(timestamped_set.Get(timestamp_2) != boost::none);
  EXPECT_EQ(timestamped_set.size(), 1);
  EXPECT_FALSE(timestamped_set.empty());
  {
    const auto good_val = timestamped_set.Get(timestamp_2);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val->value, value_2);
    EXPECT_EQ(good_val->timestamp, timestamp_2);
  }

  // Bad Remove
  EXPECT_FALSE(timestamped_set.Remove(timestamp_1));
  EXPECT_FALSE(timestamped_set.Remove(100));

  // Remove element 2
  EXPECT_TRUE(timestamped_set.Remove(timestamp_2));
  EXPECT_TRUE(timestamped_set.Get(timestamp_1) == boost::none);
  EXPECT_FALSE(timestamped_set.Contains(timestamp_1));
  EXPECT_TRUE(timestamped_set.Get(timestamp_2) == boost::none);
  EXPECT_FALSE(timestamped_set.Contains(timestamp_2));
  EXPECT_EQ(timestamped_set.size(), 0);
  EXPECT_TRUE(timestamped_set.empty());
  {
    const auto bad_val = timestamped_set.Get(timestamp_2);
    EXPECT_TRUE(bad_val == boost::none);
  }
}

TEST(TimestampedSetTester, OldestLatest) {
  lc::TimestampedSet<double> timestamped_set;
  // No elements
  {
    EXPECT_TRUE(timestamped_set.Oldest() == boost::none);
    EXPECT_TRUE(timestamped_set.Latest() == boost::none);
  }
  const double value_1 = 101.0;
  const localization_common::Time timestamp_1 = 1.0;
  ASSERT_TRUE(timestamped_set.Add(timestamp_1, value_1));
  // 1 element
  {
    const auto oldest_value = timestamped_set.Oldest();
    ASSERT_TRUE(oldest_value != boost::none);
    EXPECT_EQ(oldest_value->value, value_1);
    EXPECT_EQ(oldest_value->timestamp, timestamp_1);
    const auto latest_value = timestamped_set.Latest();
    ASSERT_TRUE(latest_value != boost::none);
    EXPECT_EQ(latest_value->value, value_1);
    EXPECT_EQ(latest_value->timestamp, timestamp_1);
  }
  // 2 elements
  const double value_2 = 100.3;
  const localization_common::Time timestamp_2 = 3.3;
  ASSERT_TRUE(timestamped_set.Add(timestamp_2, value_2));
  {
    const auto oldest_value = timestamped_set.Oldest();
    ASSERT_TRUE(oldest_value != boost::none);
    EXPECT_EQ(oldest_value->value, value_1);
    EXPECT_EQ(oldest_value->timestamp, timestamp_1);
    const auto latest_value = timestamped_set.Latest();
    ASSERT_TRUE(latest_value != boost::none);
    EXPECT_EQ(latest_value->value, value_2);
    EXPECT_EQ(latest_value->timestamp, timestamp_2);
  }

  // 3 elements
  const double value_3 = 2100.3;
  const localization_common::Time timestamp_3 = 19.3;
  ASSERT_TRUE(timestamped_set.Add(timestamp_3, value_3));
  {
    const auto oldest_value = timestamped_set.Oldest();
    ASSERT_TRUE(oldest_value != boost::none);
    EXPECT_EQ(oldest_value->value, value_1);
    EXPECT_EQ(oldest_value->timestamp, timestamp_1);
    const auto latest_value = timestamped_set.Latest();
    ASSERT_TRUE(latest_value != boost::none);
    EXPECT_EQ(latest_value->value, value_3);
    EXPECT_EQ(latest_value->timestamp, timestamp_3);
  }

  ASSERT_TRUE(timestamped_set.Remove(timestamp_1));
  {
    const auto oldest_value = timestamped_set.Oldest();
    ASSERT_TRUE(oldest_value != boost::none);
    EXPECT_EQ(oldest_value->value, value_2);
    EXPECT_EQ(oldest_value->timestamp, timestamp_2);
    const auto latest_value = timestamped_set.Latest();
    ASSERT_TRUE(latest_value != boost::none);
    EXPECT_EQ(latest_value->value, value_3);
    EXPECT_EQ(latest_value->timestamp, timestamp_3);
  }

  ASSERT_TRUE(timestamped_set.Remove(timestamp_3));
  {
    const auto oldest_value = timestamped_set.Oldest();
    ASSERT_TRUE(oldest_value != boost::none);
    EXPECT_EQ(oldest_value->value, value_2);
    EXPECT_EQ(oldest_value->timestamp, timestamp_2);
    const auto latest_value = timestamped_set.Latest();
    ASSERT_TRUE(latest_value != boost::none);
    EXPECT_EQ(latest_value->value, value_2);
    EXPECT_EQ(latest_value->timestamp, timestamp_2);
  }

  ASSERT_TRUE(timestamped_set.Remove(timestamp_2));
  {
    EXPECT_TRUE(timestamped_set.Oldest() == boost::none);
    EXPECT_TRUE(timestamped_set.Latest() == boost::none);
  }
}

TEST(TimestampedSetTester, LowerAndUpperBounds) {
  lc::TimestampedSet<double> timestamped_set;
  // No elements
  {
    const auto lower_and_upper_bound_values = timestamped_set.LowerAndUpperBound(1.0);
    EXPECT_TRUE(lower_and_upper_bound_values.first == boost::none);
    EXPECT_TRUE(lower_and_upper_bound_values.second == boost::none);
  }

  // 1 element
  const double value_1 = -77.0;
  const localization_common::Time timestamp_1 = 37.0;
  ASSERT_TRUE(timestamped_set.Add(timestamp_1, value_1));
  // 1 element below
  {
    const auto lower_and_upper_bound_values = timestamped_set.LowerAndUpperBound(10.0);
    EXPECT_TRUE(lower_and_upper_bound_values.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_values.second != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.second->value, value_1);
    EXPECT_EQ(lower_and_upper_bound_values.second->timestamp, timestamp_1);
  }
  // 1 element above
  {
    const auto lower_and_upper_bound_values = timestamped_set.LowerAndUpperBound(57.3);
    ASSERT_TRUE(lower_and_upper_bound_values.first != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.first->value, value_1);
    EXPECT_TRUE(lower_and_upper_bound_values.second == boost::none);
  }
  // 1 element equal
  {
    const auto lower_and_upper_bound_values = timestamped_set.LowerAndUpperBound(timestamp_1);
    EXPECT_TRUE(lower_and_upper_bound_values.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_values.second != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.second->value, value_1);
    EXPECT_EQ(lower_and_upper_bound_values.second->timestamp, timestamp_1);
  }

  // 2 elements
  const double value_2 = 512.0;
  const localization_common::Time timestamp_2 = 2.33;
  ASSERT_TRUE(timestamped_set.Add(timestamp_2, value_2));
  // 2 elements below
  {
    const auto lower_and_upper_bound_values = timestamped_set.LowerAndUpperBound(1.1);
    EXPECT_TRUE(lower_and_upper_bound_values.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_values.second != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.second->value, value_2);
    EXPECT_EQ(lower_and_upper_bound_values.second->timestamp, timestamp_2);
  }
  // 2 elements above
  {
    const auto lower_and_upper_bound_values = timestamped_set.LowerAndUpperBound(111.3);
    ASSERT_TRUE(lower_and_upper_bound_values.first != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.first->value, value_1);
    EXPECT_EQ(lower_and_upper_bound_values.first->timestamp, timestamp_1);
    EXPECT_TRUE(lower_and_upper_bound_values.second == boost::none);
  }
  // 2 elements equal lower
  {
    const auto lower_and_upper_bound_values = timestamped_set.LowerAndUpperBound(timestamp_2);
    EXPECT_TRUE(lower_and_upper_bound_values.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_values.second != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.second->value, value_2);
    EXPECT_EQ(lower_and_upper_bound_values.second->timestamp, timestamp_2);
  }
  // 2 elements equal upper
  {
    const auto lower_and_upper_bound_values = timestamped_set.LowerAndUpperBound(timestamp_1);
    ASSERT_TRUE(lower_and_upper_bound_values.first != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.first->value, value_2);
    EXPECT_EQ(lower_and_upper_bound_values.first->timestamp, timestamp_2);
    ASSERT_TRUE(lower_and_upper_bound_values.second != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.second->value, value_1);
    EXPECT_EQ(lower_and_upper_bound_values.second->timestamp, timestamp_1);
  }
  // 2 elements between
  {
    const auto lower_and_upper_bound_values = timestamped_set.LowerAndUpperBound(15.1);
    ASSERT_TRUE(lower_and_upper_bound_values.first != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.first->value, value_2);
    EXPECT_EQ(lower_and_upper_bound_values.first->timestamp, timestamp_2);
    ASSERT_TRUE(lower_and_upper_bound_values.second != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.second->value, value_1);
    EXPECT_EQ(lower_and_upper_bound_values.second->timestamp, timestamp_1);
  }

  // 3 elements
  const double value_3 = 291.1;
  const localization_common::Time timestamp_3 = 14.1;
  ASSERT_TRUE(timestamped_set.Add(timestamp_3, value_3));
  // 3 elements lower between
  {
    const auto lower_and_upper_bound_values = timestamped_set.LowerAndUpperBound(7.11);
    ASSERT_TRUE(lower_and_upper_bound_values.first != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.first->value, value_2);
    EXPECT_EQ(lower_and_upper_bound_values.first->timestamp, timestamp_2);
    ASSERT_TRUE(lower_and_upper_bound_values.second != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.second->value, value_3);
    EXPECT_EQ(lower_and_upper_bound_values.second->timestamp, timestamp_3);
  }
  // 3 elements upper between
  {
    const auto lower_and_upper_bound_values = timestamped_set.LowerAndUpperBound(22.22);
    ASSERT_TRUE(lower_and_upper_bound_values.first != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.first->value, value_3);
    EXPECT_EQ(lower_and_upper_bound_values.first->timestamp, timestamp_3);
    ASSERT_TRUE(lower_and_upper_bound_values.second != boost::none);
    EXPECT_EQ(lower_and_upper_bound_values.second->value, value_1);
    EXPECT_EQ(lower_and_upper_bound_values.second->timestamp, timestamp_1);
  }
}

TEST(TimestampedSetTester, LowerBoundOrEqual) {
  lc::TimestampedSet<double> timestamped_set;
  const double value_1 = 1.23;
  const localization_common::Time timestamp_1 = 3.1;
  ASSERT_TRUE(timestamped_set.Add(timestamp_1, value_1));
  const double value_2 = 2.22;
  const localization_common::Time timestamp_2 = 5.78;
  ASSERT_TRUE(timestamped_set.Add(timestamp_2, value_2));
  const double value_3 = 3.98;
  const localization_common::Time timestamp_3 = 7.88;
  ASSERT_TRUE(timestamped_set.Add(timestamp_3, value_3));

  const auto too_low_value = timestamped_set.LowerBoundOrEqual(1.23);
  EXPECT_TRUE(too_low_value == boost::none);
  const auto lowest_value = timestamped_set.LowerBoundOrEqual(4.11);
  ASSERT_TRUE(lowest_value != boost::none);
  EXPECT_EQ(lowest_value->value, value_1);
  EXPECT_EQ(lowest_value->timestamp, timestamp_1);
  const auto middle_value = timestamped_set.LowerBoundOrEqual(6.61);
  ASSERT_TRUE(middle_value != boost::none);
  EXPECT_EQ(middle_value->value, value_2);
  EXPECT_EQ(middle_value->timestamp, timestamp_2);
  const auto upper_value = timestamped_set.LowerBoundOrEqual(900);
  ASSERT_TRUE(upper_value != boost::none);
  EXPECT_EQ(upper_value->value, value_3);
  EXPECT_EQ(upper_value->timestamp, timestamp_3);
  const auto equal_value = timestamped_set.LowerBoundOrEqual(timestamp_2);
  ASSERT_TRUE(equal_value != boost::none);
  EXPECT_EQ(equal_value->value, value_2);
  EXPECT_EQ(equal_value->timestamp, timestamp_2);
}

TEST(TimestampedSetTester, Closest) {
  lc::TimestampedSet<double> timestamped_set;
  const double value_1 = 1.23;
  const localization_common::Time timestamp_1 = 3.1;
  ASSERT_TRUE(timestamped_set.Add(timestamp_1, value_1));
  const double value_2 = 2.22;
  const localization_common::Time timestamp_2 = 5.78;
  ASSERT_TRUE(timestamped_set.Add(timestamp_2, value_2));
  const double value_3 = 3.98;
  const localization_common::Time timestamp_3 = 7.88;
  ASSERT_TRUE(timestamped_set.Add(timestamp_3, value_3));

  const auto below_lowest_value = timestamped_set.Closest(1.23);
  EXPECT_TRUE(below_lowest_value != boost::none);
  EXPECT_EQ(below_lowest_value->value, value_1);
  EXPECT_EQ(below_lowest_value->timestamp, timestamp_1);
  const auto above_lowest_value = timestamped_set.Closest(4.11);
  ASSERT_TRUE(above_lowest_value != boost::none);
  EXPECT_EQ(above_lowest_value->value, value_1);
  EXPECT_EQ(above_lowest_value->timestamp, timestamp_1);
  const auto below_middle_value = timestamped_set.Closest(5.61);
  ASSERT_TRUE(below_middle_value != boost::none);
  EXPECT_EQ(below_middle_value->value, value_2);
  EXPECT_EQ(below_middle_value->timestamp, timestamp_2);
  const auto above_middle_value = timestamped_set.Closest(6.61);
  ASSERT_TRUE(above_middle_value != boost::none);
  EXPECT_EQ(above_middle_value->value, value_2);
  EXPECT_EQ(above_middle_value->timestamp, timestamp_2);
  const auto below_upper_value = timestamped_set.Closest(7.61);
  ASSERT_TRUE(below_upper_value != boost::none);
  EXPECT_EQ(below_upper_value->value, value_3);
  EXPECT_EQ(below_upper_value->timestamp, timestamp_3);
  const auto above_upper_value = timestamped_set.Closest(8.61);
  ASSERT_TRUE(above_upper_value != boost::none);
  EXPECT_EQ(above_upper_value->value, value_3);
  EXPECT_EQ(above_upper_value->timestamp, timestamp_3);
  const auto equal_value = timestamped_set.Closest(timestamp_2);
  ASSERT_TRUE(equal_value != boost::none);
  EXPECT_EQ(equal_value->value, value_2);
  EXPECT_EQ(equal_value->timestamp, timestamp_2);
}

TEST(TimestampedSetTester, OldValues) {
  lc::TimestampedSet<double> timestamped_set;
  const double t0 = 0;
  const double v0 = lc::RandomDouble();
  const double t1 = 1.001;
  const double v1 = lc::RandomDouble();
  const double t2 = 2.100;
  const double v2 = lc::RandomDouble();
  const double t3 = 3.0222;
  const double v3 = lc::RandomDouble();
  ASSERT_TRUE(timestamped_set.Add(t0, v0));
  ASSERT_TRUE(timestamped_set.Add(t1, v1));
  ASSERT_TRUE(timestamped_set.Add(t2, v2));
  ASSERT_TRUE(timestamped_set.Add(t3, v3));
  {
    const auto old_values = timestamped_set.OldValues(0);
    EXPECT_EQ(old_values.size(), 0);
  }
  {
    const auto old_values = timestamped_set.OldValues(0.1);
    ASSERT_EQ(old_values.size(), 1);
    EXPECT_EQ(old_values[0].value, v0);
    EXPECT_EQ(old_values[0].timestamp, t0);
  }
  {
    const auto old_values = timestamped_set.OldValues(1.7);
    ASSERT_EQ(old_values.size(), 2);
    EXPECT_EQ(old_values[0].value, v0);
    EXPECT_EQ(old_values[0].timestamp, t0);
    EXPECT_EQ(old_values[1].value, v1);
    EXPECT_EQ(old_values[1].timestamp, t1);
  }
  {
    const auto old_values = timestamped_set.OldValues(1999);
    ASSERT_EQ(old_values.size(), 4);
    EXPECT_EQ(old_values[0].value, v0);
    EXPECT_EQ(old_values[0].timestamp, t0);
    EXPECT_EQ(old_values[1].value, v1);
    EXPECT_EQ(old_values[1].timestamp, t1);
    EXPECT_EQ(old_values[2].value, v2);
    EXPECT_EQ(old_values[2].timestamp, t2);
    EXPECT_EQ(old_values[3].value, v3);
    EXPECT_EQ(old_values[3].timestamp, t3);
  }
}

TEST(TimestampedSetTester, RemoveOldValues) {
  {
    lc::TimestampedSet<double> timestamped_set;
    const double t0 = 0;
    const double v0 = lc::RandomDouble();
    const double t1 = 1.001;
    const double v1 = lc::RandomDouble();
    const double t2 = 2.100;
    const double v2 = lc::RandomDouble();
    const double t3 = 3.0222;
    const double v3 = lc::RandomDouble();
    ASSERT_TRUE(timestamped_set.Add(t0, v0));
    ASSERT_TRUE(timestamped_set.Add(t1, v1));
    ASSERT_TRUE(timestamped_set.Add(t2, v2));
    ASSERT_TRUE(timestamped_set.Add(t3, v3));
    const int num_values_removed = timestamped_set.RemoveOldValues(0);
    EXPECT_EQ(num_values_removed, 0);
    EXPECT_EQ(timestamped_set.size(), 4);
  }
  {
    lc::TimestampedSet<double> timestamped_set;
    const double t0 = 0;
    const double v0 = lc::RandomDouble();
    const double t1 = 1.001;
    const double v1 = lc::RandomDouble();
    const double t2 = 2.100;
    const double v2 = lc::RandomDouble();
    const double t3 = 3.0222;
    const double v3 = lc::RandomDouble();
    ASSERT_TRUE(timestamped_set.Add(t0, v0));
    ASSERT_TRUE(timestamped_set.Add(t1, v1));
    ASSERT_TRUE(timestamped_set.Add(t2, v2));
    ASSERT_TRUE(timestamped_set.Add(t3, v3));
    const int num_values_removed = timestamped_set.RemoveOldValues(0.1);
    EXPECT_EQ(num_values_removed, 1);
    EXPECT_EQ(timestamped_set.size(), 3);
    const auto timestamps = timestamped_set.Timestamps();
    EXPECT_EQ(timestamps[0], t1);
    EXPECT_EQ(timestamps[1], t2);
    EXPECT_EQ(timestamps[2], t3);
  }
  {
    lc::TimestampedSet<double> timestamped_set;
    const double t0 = 0;
    const double v0 = lc::RandomDouble();
    const double t1 = 1.001;
    const double v1 = lc::RandomDouble();
    const double t2 = 2.100;
    const double v2 = lc::RandomDouble();
    const double t3 = 3.0222;
    const double v3 = lc::RandomDouble();
    ASSERT_TRUE(timestamped_set.Add(t0, v0));
    ASSERT_TRUE(timestamped_set.Add(t1, v1));
    ASSERT_TRUE(timestamped_set.Add(t2, v2));
    ASSERT_TRUE(timestamped_set.Add(t3, v3));
    const int num_values_removed = timestamped_set.RemoveOldValues(1.334);
    EXPECT_EQ(num_values_removed, 2);
    EXPECT_EQ(timestamped_set.size(), 2);
    const auto timestamps = timestamped_set.Timestamps();
    EXPECT_EQ(timestamps[0], t2);
    EXPECT_EQ(timestamps[1], t3);
  }

  {
    lc::TimestampedSet<double> timestamped_set;
    const double t0 = 0;
    const double v0 = lc::RandomDouble();
    const double t1 = 1.001;
    const double v1 = lc::RandomDouble();
    const double t2 = 2.100;
    const double v2 = lc::RandomDouble();
    const double t3 = 3.0222;
    const double v3 = lc::RandomDouble();
    ASSERT_TRUE(timestamped_set.Add(t0, v0));
    ASSERT_TRUE(timestamped_set.Add(t1, v1));
    ASSERT_TRUE(timestamped_set.Add(t2, v2));
    ASSERT_TRUE(timestamped_set.Add(t3, v3));
    const int num_values_removed = timestamped_set.RemoveOldValues(2.78);
    EXPECT_EQ(num_values_removed, 3);
    EXPECT_EQ(timestamped_set.size(), 1);
    const auto timestamps = timestamped_set.Timestamps();
    EXPECT_EQ(timestamps[0], t3);
  }

  {
    lc::TimestampedSet<double> timestamped_set;
    const double t0 = 0;
    const double v0 = lc::RandomDouble();
    const double t1 = 1.001;
    const double v1 = lc::RandomDouble();
    const double t2 = 2.100;
    const double v2 = lc::RandomDouble();
    const double t3 = 3.0222;
    const double v3 = lc::RandomDouble();
    ASSERT_TRUE(timestamped_set.Add(t0, v0));
    ASSERT_TRUE(timestamped_set.Add(t1, v1));
    ASSERT_TRUE(timestamped_set.Add(t2, v2));
    ASSERT_TRUE(timestamped_set.Add(t3, v3));
    const int num_values_removed = timestamped_set.RemoveOldValues(1923.78);
    EXPECT_EQ(num_values_removed, 4);
    EXPECT_EQ(timestamped_set.size(), 0);
  }
}

TEST(TimestampedSetTester, Duration) {
  lc::TimestampedSet<double> timestamped_set;
  EXPECT_EQ(timestamped_set.Duration(), 0);
  ASSERT_TRUE(timestamped_set.Add(1.0, 1));
  EXPECT_EQ(timestamped_set.Duration(), 0);
  ASSERT_TRUE(timestamped_set.Add(2.0, 2));
  EXPECT_NEAR(timestamped_set.Duration(), 1, 1e-6);
  ASSERT_TRUE(timestamped_set.Add(3.0, 3));
  EXPECT_NEAR(timestamped_set.Duration(), 2, 1e-6);
}

TEST(TimestampedSetTester, Timestamps) {
  lc::TimestampedSet<double> timestamped_set;
  {
    const auto timestamps = timestamped_set.Timestamps();
    EXPECT_EQ(timestamps.size(), 0);
  }
  const double t0 = 0;
  const double t1 = 1;
  const double t2 = 2;
  const double t3 = 3;
  ASSERT_TRUE(timestamped_set.Add(t0, t0));
  ASSERT_TRUE(timestamped_set.Add(t1, t1));
  ASSERT_TRUE(timestamped_set.Add(t2, t2));
  ASSERT_TRUE(timestamped_set.Add(t3, t3));
  {
    const auto timestamps = timestamped_set.Timestamps();
    EXPECT_EQ(timestamps[0], t0);
    EXPECT_EQ(timestamps[1], t1);
    EXPECT_EQ(timestamps[2], t2);
    EXPECT_EQ(timestamps[3], t3);
  }
}

TEST(TimestampedSetTester, Serialization) {
  const lc::TimestampedSet<double> set;
  const auto serialized_set = gtsam::serializeBinary(set);
  lc::TimestampedSet<double> deserialized_set;
  gtsam::deserializeBinary(serialized_set, deserialized_set);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
