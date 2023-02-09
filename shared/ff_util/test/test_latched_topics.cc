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
    FF_CREATE_PUBLISHER(publisher_,
                        nh,
                        std_msgs::msg::String,
                        TOPIC_ROBOT_NAME,
                        1);

    std_msgs::msg::String robot_name_msg = std_msgs::msg::String();
    robot_name_msg.data = "test_robot";
    publisher_->publish(robot_name_msg);
  }

 private:
  Publisher<std_msgs::msg::String> publisher_;
};

class TestSubscriber : ff_util::FreeFlyerComponent {
 public:
  explicit TestSubscriber(const rclcpp::NodeOptions& options) :
    ff_util::FreeFlyerComponent(options, "latched_topic_subscriber", true) {}

  void Initialize(NodeHandle nh) {
    // subscriber_ = FF_CREATE_SUBSCRIBER(nh, std_msgs::msg::String, )
  }

 private:
  Subscriber<std_msgs::msg::String> subscriber_;
  ff_util::FreeFlyerTimer timer_;
};

TEST(latched_topic, LatchedTopicTimeout) {
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr nh =
                  std::make_shared<rclcpp::Node>("test_latched_topic_timeout");
  TestPublisher publisher(node_options);
  publisher.Initialize(nh);
  EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
