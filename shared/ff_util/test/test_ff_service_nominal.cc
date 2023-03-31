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

// Test service
#include <ff_util/ff_component.h>
#include <ff_util/ff_service.h>
#include <ff_common/ff_ros.h>
#include <gtest/gtest.h>

#include <ff_msgs/srv/set_rate.hpp>

#include <std_msgs/msg/string.hpp>

#include <string>

FF_DEFINE_LOGGER("test_ff_service_nominal")

bool test_done = false;

void ResultsCallback(std_msgs::msg::String const& msg) {
  FF_INFO_STREAM("Received results from client: " << msg.data);
  EXPECT_EQ(msg.data, "The response was correct!");
  test_done = true;
}

/*void TimerCallback() {
  std_msgs::msg::String msg = std_msgs::msg::String();
  msg.data = "Nominal service test";
  trigger_pub->publish(msg);
  FF_INFO_STREAM("After publishing trigger message.");
}*/

TEST(ff_service, Nominal) {
  test_done = false;
  rclcpp::Node::SharedPtr test_node =
                  std::make_shared<rclcpp::Node>("test_ff_service_nominal");

  Subscriber<std_msgs::msg::String> results_sub;
  Publisher<std_msgs::msg::String> trigger_pub;

  ff_util::FreeFlyerTimer test_timer_;

  results_sub = FF_CREATE_SUBSCRIBER(test_node,
                            std_msgs::msg::String,
                            "/client_result",
                            1,
                            std::bind(&ResultsCallback, std::placeholders::_1));
  rclcpp::QoS latched_qos(1);
  latched_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  trigger_pub = test_node->create_publisher<std_msgs::msg::String>("/client_trigger", latched_qos);

  // Publish trigger message after test service client starts up
  // test_timer_.createTimer(1.0, &TimerCallback, test_node, true, true);
  std_msgs::msg::String msg = std_msgs::msg::String();
  msg.data = "Nominal service test";
  trigger_pub->publish(msg);
  FF_INFO_STREAM("After publishing trigger message.");

  while (!test_done) {
    rclcpp::spin_some(test_node);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  // google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
