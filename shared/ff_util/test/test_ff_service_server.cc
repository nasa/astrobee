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

FF_DEFINE_LOGGER("test_ff_service_server")

namespace ff_util {

class TestServiceServer : public ff_util::FreeFlyerComponent {
 public :
  explicit TestServiceServer(const rclcpp::NodeOptions& options) :
      FreeFlyerComponent(options, "service_server_test", false) {}

  void Initialize(NodeHandle &node) {
    service_ = node->create_service<ff_msgs::srv::SetRate>("/testing/set_rate",
      std::bind(&TestServiceServer::SetRateCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  }

  void SetRateCallback(const std::shared_ptr<ff_msgs::srv::SetRate::Request> req,
                       std::shared_ptr<ff_msgs::srv::SetRate::Response> res) {
    FF_INFO_STREAM("In the set rate callback. which: " << req->which);
    FF_INFO_STREAM("Request rate: "<< req->rate);
    if (req->which == ff_msgs::srv::SetRate::Request::DISK_STATE &&
        req->rate == 5) {
      res->success = true;
      res->status = "Which is disk state.";
    } else {
      res->success = false;
      res->status = "Which is " + std::to_string(req->rate);
    }
  }

 private:
  Service<ff_msgs::srv::SetRate> service_;
};

}  // namespace ff_util

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ff_util::TestServiceServer)
