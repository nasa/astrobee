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

class Server : public ff_util::FreeFlyerComponent {
 public :
  Server(const rclcpp::NodeOptions & options) :
    FreeFlyerComponent(options, "service_server_test", false) {
  }

  void Initialize(NodeHandle node) {
    service_ = node->create_service<ff_msgs::srv::SetRate>("/testing/set_rate",
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

class Client : public ff_util::FreeFlyerComponent {
 public :
  Client(const rclcpp::NodeOptions & options) :
    FreeFlyerComponent(options, "service_client_test", false) {
  }

  void Initialize(NodeHandle node) {
    client_.Create(node, "/testing/set_rate");
  }

  void TestCall(bool nominal) {
    ff_util::FreeFlyerService<ff_msgs::srv::SetRate::Request,
                            ff_msgs::srv::SetRate::Response> srv;
    srv.request->which = ff_msgs::srv::SetRate::Request::DISK_STATE;
    srv.request->rate = 5.0;

    client_.waitForExistence(5.0);

    // Test is valid for fun
    bool status = false;
    if (client_.isValid()) {
      status = client_.call(srv);
    }

    if (nominal) {
      EXPECT_TRUE(status);
      EXPECT_TRUE(srv.response->success);
      EXPECT_EQ(srv.response->status, "Which is disk state.");
    } else {
      EXPECT_FALSE(status);
      EXPECT_FALSE(srv.response->success);
      EXPECT_EQ(srv.response->status, "");
    }
    test_done = true;
  }

 private:
  ff_util::FreeFlyerServiceClient<ff_msgs::srv::SetRate,
                                  ff_msgs::srv::SetRate::Request,
                                  ff_msgs::srv::SetRate::Response> client_;

};

TEST(ff_service, Nominal) {
  test_done = false;
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr test_node =
                  std::make_shared<rclcpp::Node>("test_ff_service_nominal");

  Server server(node_options);
  Client client(node_options);

  server.Initialize(test_node);
  client.Initialize(test_node);

  client.TestCall(true);
  while(!test_done) {
    rclcpp::spin_some(test_node);
  }
}

TEST(ff_service, NoConnection) {
  test_done = false;
  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr test_node =
                        std::make_shared<rclcpp::Node>("test_ff_service_no_connection");

  Client client(node_options);

  client.Initialize(test_node);

  client.TestCall(false);
  while(!test_done) {
    rclcpp::spin_some(test_node);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  // google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
