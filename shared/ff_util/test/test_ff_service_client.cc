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

FF_DEFINE_LOGGER("test_ff_service_client")

namespace ff_util {

class TestServiceClient : public ff_util::FreeFlyerComponent {
 public :
  explicit TestServiceClient(const rclcpp::NodeOptions& options) :
      FreeFlyerComponent(options, "service_client_test", false) {}

  void Initialize(NodeHandle &node) {
    client_.Create(node, "/testing/set_rate");
    results_pub_ = FF_CREATE_PUBLISHER(node,
                                       std_msgs::msg::String,
                                       "/client_result",
                                       1);
    rclcpp::QoS latched_qos(1);
    latched_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    trigger_sub_ =
          node->create_subscription<std_msgs::msg::String>("/client_trigger",
          latched_qos,
          std::bind(&TestServiceClient::TestCall, this, std::placeholders::_1));
  }

  void TestCall(std_msgs::msg::String const& msg_in) {
    FF_INFO_STREAM("In client test call. The call has been triggered.");
    ff_msgs::srv::SetRate::Request request;
    std_msgs::msg::String msg_out = std_msgs::msg::String();
    auto response = std::make_shared<ff_msgs::srv::SetRate::Response>();
    request.which = ff_msgs::srv::SetRate::Request::DISK_STATE;
    request.rate = 5.0;

    client_.waitForExistence(5.0);

    // Test is valid for fun
    bool status = false;
    if (client_.isValid()) {
      status = client_.call(request, response);
    }

    if (status && response->success &&
                                  response->status == "Which is disk state.") {
      msg_out.data = "The response was correct!";
    } else {
      msg_out.data = "The response was empty!";
    }
    results_pub_->publish(msg_out);
  }

 private:
  ff_util::FreeFlyerServiceClient<ff_msgs::srv::SetRate> client_;
  Publisher<std_msgs::msg::String> results_pub_;
  Subscriber<std_msgs::msg::String> trigger_sub_;
};

}  // namespace ff_util

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ff_util::TestServiceClient)
