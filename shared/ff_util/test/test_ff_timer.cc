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

// Test timer
#include <ff_util/ff_timer.h>
#include <gtest/gtest.h>

#include <cmath>

bool oneshot = false, oneshot_fired = false, test_done = false;
double timer_duration = 2.0;
rclcpp::Node::SharedPtr test_node;
rclcpp::Time start_time;
rclcpp::TimerBase::SharedPtr timeout_timer;


void OneShotTimerCallback() {
  // If the one shot timer already fired, fail since it shouldn't trigger again
  if (oneshot_fired) {
    test_done = true;
    EXPECT_TRUE(false);
    return;
  }
  oneshot_fired = true;
}

void TimerTimeout() {
  test_done = true;
  // If the timer is one shot, we need to make sure it fired once
  if (oneshot) {
    if (oneshot_fired) {
      EXPECT_TRUE(true);
      return;
    }
  }
  EXPECT_TRUE(false);
}

void TimerCallback() {
  test_done = true;
  double time_diff = std::floor(test_node->now().seconds() - start_time.seconds());
  EXPECT_EQ(timer_duration, time_diff);
}

void CreateTimeoutTimer() {
  timeout_timer = rclcpp::create_timer(test_node,
                                       test_node->get_clock(),
                                       rclcpp::Duration::from_seconds((timer_duration*3)),
                                       &TimerTimeout);
}

TEST(ff_timer, DelayedStartTimer) {
  test_done = false;
  test_node = std::make_shared<rclcpp::Node>("test_ff_timer_delayed_start");
  ff_util::FreeFlyerTimer test_timer;
  test_timer.createTimer(timer_duration, &TimerCallback, test_node, false, false);

  start_time = test_node->now();
  test_timer.start();
  CreateTimeoutTimer();

  while (!test_done) {
    rclcpp::spin_some(test_node);
  }

  timeout_timer->cancel();
  test_timer.stop();
}

TEST(ff_timer, OneShotTimer) {
  test_done = false;
  oneshot = true;
  test_node = std::make_shared<rclcpp::Node>("test_ff_timer_one_shot");
  ff_util::FreeFlyerTimer oneshot_test_timer;
  oneshot_test_timer.createTimer(timer_duration, &OneShotTimerCallback, test_node, true);
  CreateTimeoutTimer();

  while (!test_done) {
    rclcpp::spin_some(test_node);
  }

  timeout_timer->cancel();
  oneshot_test_timer.stop();
  oneshot = false;
  oneshot_fired = false;
}

TEST(ff_timer, AutoStartTimer) {
  test_done = false;
  test_node = std::make_shared<rclcpp::Node>("test_ff_timer_auto_start");
  ff_util::FreeFlyerTimer test_timer;
  start_time = test_node->now();
  test_timer.createTimer(timer_duration, &TimerCallback, test_node);
  CreateTimeoutTimer();

  while (!test_done) {
    rclcpp::spin_some(test_node);
  }

  timeout_timer->cancel();
  test_timer.stop();
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  // google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
