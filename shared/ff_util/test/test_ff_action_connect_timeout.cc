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

// Tests action connect timeout
// In this test a server is not spawned, therefore the client can't connect.
// This test passes if a timeout on connect result is issued.

// Required for the test framework
#include <gtest/gtest.h>

// Required for the test cases
#include <ff_common/ff_ros.h>

// Action interface
#include <ff_util/ff_action.h>
#include <ff_util/ff_component.h>
#include <ff_util/ff_timer.h>

// Use one of the simplest actions
#include <ff_msgs/action/dock.hpp>

// C++ includes
#include <functional>
#include <memory>

FF_DEFINE_LOGGER("test_ff_action_connect_timeout")

// CLIENT CALLBACKS
class Client : ff_util::FreeFlyerComponent {
 public:
  explicit Client(const rclcpp::NodeOptions& options) :
      ff_util::FreeFlyerComponent(options, "action_client_test", true) {}

  void Initialize(NodeHandle nh) {
    // Setters for callbacks
    action_.SetFeedbackCallback(std::bind(&Client::FeedbackCallback,
                                          this,
                                          std::placeholders::_1));
    action_.SetResultCallback(std::bind(&Client::ResultCallback,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
    action_.SetConnectedCallback(std::bind(&Client::ConnectedCallback, this));
    action_.SetActiveCallback(std::bind(&Client::ActiveCallback, this));
    // Setters for timeout values
    action_.SetConnectedTimeout(2.0);
    action_.SetActiveTimeout(4.0);
    action_.SetResponseTimeout(4.0);
    action_.SetDeadlineTimeout(10.0);
    // Call connect
    action_.Create(nh, "test_action");
  }

 protected:
  void FeedbackCallback(
        const std::shared_ptr<const ff_msgs::action::Dock::Feedback> feedback) {
    FF_INFO("C:FeedbackCallback()");
    EXPECT_TRUE(false);
  }

  void ResultCallback(ff_util::FreeFlyerActionState::Enum state,
                  std::shared_ptr<const ff_msgs::action::Dock::Result> result) {
    FF_INFO("C:ResultCallback()");
    EXPECT_TRUE(state == ff_util::FreeFlyerActionState::TIMEOUT_ON_CONNECT);
    rclcpp::shutdown();
  }

  void ConnectedCallback() {
    FF_INFO("C:ConnectedCallback()");
    EXPECT_TRUE(false);;
  }

  void ActiveCallback() {
    FF_INFO("C:ActiveCallback()");
    EXPECT_TRUE(false);
  }

 private:
  ff_util::FreeFlyerActionClient<ff_msgs::action::Dock> action_;
};

// Perform a test of the simple action client
TEST(ff_action, connect_timeout) {
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr nh =
              std::make_shared<rclcpp::Node>("test_ff_action_connect_timeout");
  Client client(node_options);
  client.Initialize(nh);
  rclcpp::spin(nh);
}

// Required for the test framework
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
