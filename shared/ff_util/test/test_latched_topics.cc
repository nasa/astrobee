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

// Required for the test framework
#include <gtest/gtest.h>

#include <ff_common/ff_names.h>
#include <ff_common/ff_ros.h>

#include <ff_util/ff_component.h>
#include <ff_util/ff_timer.h>

#include <std_msgs/msg/string.hpp>

FF_DEFINE_LOGGER("test_latched_topic")

class TestPublisher : ff_util::FreeFlyerComponent {
 public:
  explicit TestPublisher(const rclcpp::NodeOptions& options) :
    ff_util::FreeFlyerComponent(options, "latched_topic_publisher", true) {}

  void Initialize(NodeHandle nh) {
    latched_publisher_ = FF_CREATE_PUBLISHER(nh,
                                             std_msgs::msg::String,
                                             TOPIC_ROBOT_NAME,
                                             1);
    not_latched_publisher_ = FF_CREATE_PUBLISHER(nh,
                                                 std_msgs::msg::String,
                                                 TOPIC_COMMAND,
                                                 1);

    std_msgs::msg::String msg = std_msgs::msg::String();
    msg.data = "test_robot";
    latched_publisher_->publish(msg);
    msg.data = "stopAllMotion";
    not_latched_publisher_->publish(msg);
    timer_.createTimer(15,
                       std::bind(&TestPublisher::TimerCallback, this),
                       nh,
                       true,
                       true);
  }

  // Timer callback
  void TimerCallback() {
    std_msgs::msg::String msg = std_msgs::msg::String();
    msg.data = "stopAllMotion";
    not_latched_publisher_->publish(msg);
  }

 private:
  Publisher<std_msgs::msg::String> latched_publisher_;
  Publisher<std_msgs::msg::String> not_latched_publisher_;
  ff_util::FreeFlyerTimer timer_;
};

class TestSubscriber : ff_util::FreeFlyerComponent {
 public:
  explicit TestSubscriber(const rclcpp::NodeOptions& options) :
    ff_util::FreeFlyerComponent(options, "latched_topic_subscriber", true) {}

  void InitializeNotLatched(NodeHandle nh) {
    topic_timeout_ = false;
    not_latched_subscriber_ = FF_CREATE_SUBSCRIBER(nh,
      std_msgs::msg::String,
      TOPIC_COMMAND,
      1,
      std::bind(&TestSubscriber::NotLatchedCallback, this, std::placeholders::_1));
  }

  void InitializeLatched(NodeHandle nh) {
    topic_timeout_ = true;
    latched_subscriber_ = FF_CREATE_SUBSCRIBER(nh,
      std_msgs::msg::String,
      TOPIC_ROBOT_NAME,
      1,
      std::bind(&TestSubscriber::LatchedCallback, this, std::placeholders::_1));
  }

  void LatchedCallback(std_msgs::msg::String const& msg) {
    FF_INFO("TS: Latched Callback");
    EXPECT_EQ(msg.data, "test_robot");
  }

  void NotLatchedCallback(std_msgs::msg::String const& msg) {
    FF_INFO("TS: Not Latched Callback");
    EXPECT_TRUE(topic_timeout_);
    EXPECT_EQ(msg.data, "stopAllMotion");
    rclcpp::shutdown();
  }

 private:
  Subscriber<std_msgs::msg::String> latched_subscriber_;
  Subscriber<std_msgs::msg::String> not_latched_subscriber_;
  bool topic_timeout_;
};

TEST(latched_topic, LatchedTopicTimeout) {
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr nh =
                  std::make_shared<rclcpp::Node>("test_latched_topic_timeout");
  TestPublisher publisher(node_options);
  TestSubscriber subscriber(node_options);
  publisher.Initialize(nh);
  subscriber.InitializeNotLatched(nh);
  std::chrono::nanoseconds ns(1000000000);
  // Sleep for 10 seconds
  for (int i = 0; i < 10; i++) {
    rclcpp::sleep_for(ns);
    rclcpp::spin_some(nh);
  }
  subscriber.InitializeLatched(nh);
  rclcpp::spin(nh);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
