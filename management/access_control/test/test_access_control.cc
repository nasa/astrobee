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

// Required for the test cases
#include <ff_common/ff_names.h>
#include <ff_common/ff_ros.h>

#include <ff_msgs/msg/access_control_state_stamped.hpp>
#include <ff_msgs/msg/ack_stamped.hpp>
#include <ff_msgs/msg/command_constants.hpp>
#include <ff_msgs/msg/command_stamped.hpp>
namespace ff_msgs {
  typedef msg::AckCompletedStatus AckCompletedStatus;
  typedef msg::AccessControlStateStamped AccessControlStateStamped;
  typedef msg::AckStamped AckStamped;
  typedef msg::AckStatus AckStatus;
  typedef msg::CommandArg CommandArg;
  typedef msg::CommandConstants CommandConstants;
  typedef msg::CommandStamped CommandStamped;
}

#include <vector>

// Test access control
// 1) Test Grab Control
// 2) Test sending a command while having control
// 3) Test sending a stop command without being the operator in control
// 4) Test sending an idle propulsion command without being the operator in control
// 5) Test sending a grab control command with an invalid cookie
// 6) Test sending a grab control command with no arguements
// 7) Test sending a grab control command with the wrong argument type
// 8) Test sending a command without being the operator in control

// Publisher and subscribers
Subscriber<ff_msgs::CommandStamped> cmd_sub;
Subscriber<ff_msgs::AckStamped> ack_sub;
Subscriber<ff_msgs::AccessControlStateStamped> state_sub;
Publisher<ff_msgs::CommandStamped> cmd_pub;

FF_DEFINE_LOGGER("test_access_control")

std::string cmd_id = "";
std::string cookie = "";
std::string cmd_name = "";
std::string controller = "";

bool test_done = false;
bool sent_grab = false;
bool got_request_ack = false;
bool got_grab_ack = false;
bool got_state = false;

void PublishCommand(std::vector<ff_msgs::CommandArg> *args) {
  ff_msgs::CommandStamped cmd;
  cmd.cmd_name = cmd_name;
  cmd.cmd_id = cmd_id;
  cmd.cmd_src = controller;
  if (args != NULL) {
    cmd.args = *args;
  }

  cmd_pub->publish(cmd);
}

void PublishGrabControl() {
  cmd_name = ff_msgs::CommandConstants::CMD_NAME_GRAB_CONTROL;

  std::vector<ff_msgs::CommandArg> args;
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = cookie;
  args.push_back(arg);

  PublishCommand(&args);
  sent_grab = true;
}

void GCAckCallback(ff_msgs::AckStamped const& ack) {
  EXPECT_EQ(ack.cmd_id, cmd_id.c_str());
  EXPECT_EQ(ack.status.status, ff_msgs::AckStatus::COMPLETED);
  EXPECT_EQ(ack.completed_status.status, ff_msgs::AckCompletedStatus::OK);
  // check request ack succeed and grab control
  if (ack.cmd_id == "request_control") {
    got_request_ack = true;
    if (cookie != "") {
      cmd_id = "grab_control";
      PublishGrabControl();
      return;
    }
  }

  if (ack.cmd_id == "grab_control") {
    got_grab_ack = true;
    // Make sure got and checked access state before quitting test
    if (got_state) {
      test_done = true;
    }
  }
}

void CWCAckCallback(ff_msgs::AckStamped const& ack) {
  EXPECT_EQ(ack.cmd_id, cmd_id.c_str());
  EXPECT_EQ(ack.status.status, ff_msgs::AckStatus::COMPLETED);
  EXPECT_EQ(ack.completed_status.status, ff_msgs::AckCompletedStatus::OK);

  // send grab control command if just requested control and received cookie
  if (ack.cmd_id == "request_control") {
    got_request_ack = true;
    if (cookie != "") {
      cmd_id = "grab_control";
      PublishGrabControl();
      return;
    }
  }

  if (ack.cmd_id == "grab_control") {
    // Send a command and make sure it is published on the /command topic (i.e.
    // make sure it got passed on to the executive.
    cmd_name = ff_msgs::CommandConstants::CMD_NAME_NO_OP;
    cmd_id = "noOp";
    PublishCommand(NULL);
  }
}

void FailedAckCallback(ff_msgs::AckStamped const& ack) {
  EXPECT_EQ(ack.cmd_id, cmd_id.c_str());
  EXPECT_EQ(ack.status.status, ff_msgs::AckStatus::COMPLETED);

  // Two diffent failures bad syntex and execution failed
  // execution failed only occurs for the invalid cookie test
  if (ack.cmd_id == "invalid_cookie") {
    EXPECT_EQ(ack.completed_status.status,
              ff_msgs::AckCompletedStatus::EXEC_FAILED);
  } else {
    EXPECT_EQ(ack.completed_status.status,
              ff_msgs::AckCompletedStatus::BAD_SYNTAX);
  }
  test_done = true;
}

// Only used for successful commands that get passed to the executive. Thus the
// test is finished
void CmdCallback(ff_msgs::CommandStamped const& cmd) {
  EXPECT_EQ(cmd.cmd_name, cmd_name);
  test_done = true;
}

void StateCallback(ff_msgs::AccessControlStateStamped const& state) {
  cookie = state.cookie;
  if (cookie != "" && got_request_ack) {
    cmd_id = "grab_control";
    PublishGrabControl();
    return;
  }

  if (sent_grab) {
    EXPECT_EQ(state.controller, controller);
    got_state = true;
    // Make sure got and checked ack before quitting
    if (got_grab_ack) {
      test_done = true;
    }
  }
}

// Test Grab Control
TEST(access_control, GrabControl) {
  rclcpp::Node::SharedPtr nh =
                            std::make_shared<rclcpp::Node>("test_grab_control");
  ack_sub = FF_CREATE_SUBSCRIBER(nh,
                                 ff_msgs::AckStamped,
                                 TOPIC_MANAGEMENT_ACK,
                                 10,
                                 &GCAckCallback);
  state_sub = FF_CREATE_SUBSCRIBER(nh,
                                   ff_msgs::AccessControlStateStamped,
                                   TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE,
                                   10,
                                   &StateCallback);
  cmd_pub = FF_CREATE_PUBLISHER(nh,
                                ff_msgs::CommandStamped,
                                TOPIC_COMMUNICATIONS_DDS_COMMAND,
                                10);

  // Reinitialize global variables
  test_done = false;
  sent_grab = false;
  got_request_ack = false;
  got_grab_ack = false;
  got_state = false;
  cookie = "";
  controller = "operator1";

  std::chrono::nanoseconds ns(100000000);
  while (nh->count_subscribers(TOPIC_COMMUNICATIONS_DDS_COMMAND) == 0 ||
         nh->count_publishers(TOPIC_MANAGEMENT_ACK) == 0 ||
         nh->count_publishers(TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE) == 0) {
    rclcpp::sleep_for(ns);
    rclcpp::spin_some(nh);
  }

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_REQUEST_CONTROL;
  cmd_id = "request_control";
  PublishCommand(NULL);

  while (!test_done) {
    rclcpp::spin_some(nh);
  }
}

// Test sending a command while having control
TEST(access_control, CommandWithControl) {
  rclcpp::Node::SharedPtr nh =
                    std::make_shared<rclcpp::Node>("test_command_with_control");
  cmd_sub = FF_CREATE_SUBSCRIBER(nh,
                                 ff_msgs::CommandStamped,
                                 TOPIC_COMMAND,
                                 10,
                                 &CmdCallback);
  ack_sub = FF_CREATE_SUBSCRIBER(nh,
                                 ff_msgs::AckStamped,
                                 TOPIC_MANAGEMENT_ACK,
                                 10,
                                 &CWCAckCallback);
  state_sub = FF_CREATE_SUBSCRIBER(nh,
                                   ff_msgs::AccessControlStateStamped,
                                   TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE,
                                   10,
                                   &StateCallback);
  cmd_pub = FF_CREATE_PUBLISHER(nh,
                                ff_msgs::CommandStamped,
                                TOPIC_COMMUNICATIONS_DDS_COMMAND,
                                10);

  std::chrono::nanoseconds ns(1000000000);
  while (nh->count_subscribers(TOPIC_COMMUNICATIONS_DDS_COMMAND) == 0 ||
         nh->count_publishers(TOPIC_COMMAND) == 0 ||
         nh->count_publishers(TOPIC_MANAGEMENT_ACK) == 0 ||
         nh->count_publishers(TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE) == 0) {
    rclcpp::sleep_for(ns);
    rclcpp::spin_some(nh);
  }

  test_done = false;
  sent_grab = false;
  got_request_ack = false;
  got_grab_ack = false;
  got_state = false;
  cookie = "";
  controller = "operator2";

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_REQUEST_CONTROL;
  cmd_id = "request_control";
  PublishCommand(NULL);

  while (!test_done) {
    rclcpp::spin_some(nh);
  }
}

// Test sending a stop command without being the operator in control
TEST(access_control, StopCommandWithoutControl) {
  rclcpp::Node::SharedPtr nh =
            std::make_shared<rclcpp::Node>("test_stop_command_without_control");
  cmd_sub = FF_CREATE_SUBSCRIBER(nh,
                                 ff_msgs::CommandStamped,
                                 TOPIC_COMMAND,
                                 10,
                                 &CmdCallback);
  cmd_pub = FF_CREATE_PUBLISHER(nh,
                                ff_msgs::CommandStamped,
                                TOPIC_COMMUNICATIONS_DDS_COMMAND,
                                10);

  std::chrono::nanoseconds ns(1000000000);
  while (nh->count_subscribers(TOPIC_COMMUNICATIONS_DDS_COMMAND) == 0 ||
         nh->count_publishers(TOPIC_COMMAND) == 0) {
    rclcpp::sleep_for(ns);
    rclcpp::spin_some(nh);
  }

  test_done = false;
  controller = "engineer";

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_STOP_ALL_MOTION;
  cmd_id = "stop";
  PublishCommand(NULL);

  while (!test_done) {
    rclcpp::spin_some(nh);
  }
}

// Test sending an idle propulsion command without being the operator in control
TEST(access_control, IdlePropulsionCommandWithoutControl) {
  rclcpp::Node::SharedPtr nh =
    std::make_shared<rclcpp::Node>("test_idle_propulsion_command_wo_control");
  cmd_sub = FF_CREATE_SUBSCRIBER(nh,
                                 ff_msgs::CommandStamped,
                                 TOPIC_COMMAND,
                                 10,
                                 &CmdCallback);
  cmd_pub = FF_CREATE_PUBLISHER(nh,
                                ff_msgs::CommandStamped,
                                TOPIC_COMMUNICATIONS_DDS_COMMAND,
                                10);

  std::chrono::nanoseconds ns(1000000000);
  while (nh->count_subscribers(TOPIC_COMMUNICATIONS_DDS_COMMAND) == 0 ||
         nh->count_publishers(TOPIC_COMMAND) == 0) {
    rclcpp::sleep_for(ns);
    rclcpp::spin_some(nh);
  }

  test_done = false;
  controller = "engineer";

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_IDLE_PROPULSION;
  cmd_id = "idle_propulsion";
  PublishCommand(NULL);

  while (!test_done) {
    rclcpp::spin_some(nh);
  }
}

// Test sending a grab control command with an invalid cookie
TEST(access_control, GrabControlCommandInvalidCookie) {
  rclcpp::Node::SharedPtr nh =
    std::make_shared<rclcpp::Node>("test_grab_control_command_invalid_cookie");
  ack_sub = FF_CREATE_SUBSCRIBER(nh,
                                 ff_msgs::AckStamped,
                                 TOPIC_MANAGEMENT_ACK,
                                 10,
                                 &FailedAckCallback);
  cmd_pub = FF_CREATE_PUBLISHER(nh,
                                ff_msgs::CommandStamped,
                                TOPIC_COMMUNICATIONS_DDS_COMMAND,
                                10);

  std::chrono::nanoseconds ns(1000000000);
  while (nh->count_subscribers(TOPIC_COMMUNICATIONS_DDS_COMMAND) == 0 ||
         nh->count_publishers(TOPIC_MANAGEMENT_ACK) == 0) {
    rclcpp::sleep_for(ns);
    rclcpp::spin_some(nh);
  }

  test_done = false;
  controller = "rogue_operator";

  cmd_id = "invalid_cookie";
  cookie = "yummy_cookies";
  PublishGrabControl();

  while (!test_done) {
    rclcpp::spin_some(nh);
  }
}

// Test sending a grab control command with no arguments
TEST(access_control, GrabControlNoArgs) {
  rclcpp::Node::SharedPtr nh =
                    std::make_shared<rclcpp::Node>("test_grab_control_no_args");
  ack_sub = FF_CREATE_SUBSCRIBER(nh,
                                 ff_msgs::AckStamped,
                                 TOPIC_MANAGEMENT_ACK,
                                 10,
                                 &FailedAckCallback);
  cmd_pub = FF_CREATE_PUBLISHER(nh,
                                ff_msgs::CommandStamped,
                                TOPIC_COMMUNICATIONS_DDS_COMMAND,
                                10);

  std::chrono::nanoseconds ns(1000000000);
  while (nh->count_subscribers(TOPIC_COMMUNICATIONS_DDS_COMMAND) == 0 ||
         nh->count_publishers(TOPIC_MANAGEMENT_ACK) == 0) {
    rclcpp::sleep_for(ns);
    rclcpp::spin_some(nh);
  }

  test_done = false;
  controller = "rogue_operator";

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_GRAB_CONTROL;
  cmd_id = "no_args";
  PublishCommand(NULL);

  while (!test_done) {
    rclcpp::spin_some(nh);
  }
}

// Test sending a grab control command with the wrong argument type
TEST(access_control, GrabControlWrongArgType) {
  rclcpp::Node::SharedPtr nh =
            std::make_shared<rclcpp::Node>("test_grab_control_wrong_arg_type");
  ack_sub = FF_CREATE_SUBSCRIBER(nh,
                                 ff_msgs::AckStamped,
                                 TOPIC_MANAGEMENT_ACK,
                                 10,
                                 &FailedAckCallback);
  cmd_pub = FF_CREATE_PUBLISHER(nh,
                                ff_msgs::CommandStamped,
                                TOPIC_COMMUNICATIONS_DDS_COMMAND,
                                10);

  std::chrono::nanoseconds ns(1000000000);
  while (nh->count_subscribers(TOPIC_COMMUNICATIONS_DDS_COMMAND) == 0 ||
         nh->count_publishers(TOPIC_MANAGEMENT_ACK) == 0) {
    rclcpp::spin_some(nh);
  }

  test_done = false;
  controller = "rogue_operator";

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_GRAB_CONTROL;
  cmd_id = "invalid_argument";
  std::vector<ff_msgs::CommandArg> args;
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_BOOL;
  arg.b = false;
  args.push_back(arg);
  PublishCommand(&args);

  while (!test_done) {
    rclcpp::spin_some(nh);
  }
}

// Test sending a command without being the operator in control
TEST(access_control, CommandWithoutControl) {
  rclcpp::Node::SharedPtr nh =
                std::make_shared<rclcpp::Node>("test_command_without_control");
  ack_sub = FF_CREATE_SUBSCRIBER(nh,
                                 ff_msgs::AckStamped,
                                 TOPIC_MANAGEMENT_ACK,
                                 10,
                                 &FailedAckCallback);
  cmd_pub = FF_CREATE_PUBLISHER(nh,
                                ff_msgs::CommandStamped,
                                TOPIC_COMMUNICATIONS_DDS_COMMAND,
                                10);

  std::chrono::nanoseconds ns(1000000000);
  while (nh->count_subscribers(TOPIC_COMMUNICATIONS_DDS_COMMAND) == 0 ||
         nh->count_publishers(TOPIC_MANAGEMENT_ACK) == 0) {
    rclcpp::sleep_for(ns);
    rclcpp::spin_some(nh);
  }

  test_done = false;
  controller = "rogue_operator";

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_NO_OP;
  cmd_id = "rogue_operator";
  PublishCommand(NULL);

  while (!test_done) {
    rclcpp::spin_some(nh);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Initialize the gtesttest framework
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
