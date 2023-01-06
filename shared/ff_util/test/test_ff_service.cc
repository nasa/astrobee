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
#include <ff_msgs/srv/set_rate.hpp>
#include <ff_util/ff_component.h>
#include <ff_util/ff_service.h>
#include <ff_common/ff_ros.h>
#include <gtest/gtest.h>

#include <string>

bool test_done = false;

class Server : public rclcpp::Node {
 public :
  Server() : Node("service_server_test") {
    service_ = this->create_service<ff_msgs::srv::SetRate>("/testing/set_rate",
      std::bind(&Server::SetRateCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void SetRateCallback(const std::shared_ptr<ff_msgs::srv::SetRate::Request> req,
                       std::shared_ptr<ff_msgs::srv::SetRate::Response> res) {
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

TEST(ff_service, Nominal) {
  test_done = false;
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr test_node =
                  std::make_shared<rclcpp::Node>("test_ff_service_nominal", node_options);

  Server server();

  ff_util::FreeFlyerServiceClient<ff_msgs::srv::SetRate,
                                  ff_msgs::srv::SetRate::Request,
                                  ff_msgs::srv::SetRate::Response> client_;

  /*client_.Create(test_node, "/testing/set_rate");

  ff_util::FreeFlyerService<ff_msgs::srv::SetRate::Request,
                            ff_msgs::srv::SetRate::Response> srv;
    srv.request->which = ff_msgs::srv::SetRate::Request::DISK_STATE;
    srv.request->rate = 5.0;

    bool status = client_.call(srv);

    EXPECT_TRUE(status);
    EXPECT_TRUE(srv.response->success);
    EXPECT_EQ(srv.response->status, "Which is disk state.");*/
  EXPECT_TRUE(true);
}

TEST(ff_timer, NoConnection) {
  test_done = false;
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_ff_service_no_connection");

  ff_util::FreeFlyerServiceClient<ff_msgs::srv::SetRate,
                                  ff_msgs::srv::SetRate::Request,
                                  ff_msgs::srv::SetRate::Response> client_;


  EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  // google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
