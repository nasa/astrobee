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
#include <glog/logging.h>

// Required for the test cases
#include <ros/ros.h>
#include <ff_msgs/AccessControlStateStamped.h>
#include <ff_msgs/AckStamped.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_util/ff_names.h>
#include <vector>

// Publisher and subscribers
ros::Subscriber cmd_sub, ack_sub, state_sub;
ros::Publisher cmd_pub;

std::string cmd_id = "";
std::string cmd_origin = "";
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
  cmd.cmd_origin = cmd_origin;
  cmd.cmd_src = controller;
  if (args != NULL) {
    cmd.args = *args;
  }
  cmd_pub.publish(cmd);
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

void GCAckCallback(ff_msgs::AckStampedConstPtr const& ack) {
  EXPECT_EQ(ack->cmd_id, cmd_id.c_str());
  EXPECT_EQ(ack->cmd_origin, cmd_origin.c_str());
  EXPECT_EQ(ack->status.status, ff_msgs::AckStatus::COMPLETED);
  EXPECT_EQ(ack->completed_status.status, ff_msgs::AckCompletedStatus::OK);
  // check request ack succeed and grab control
  if (ack->cmd_id == "request_control") {
    got_request_ack = true;
    if (cookie != "") {
      cmd_id = "grab_control";
      PublishGrabControl();
      return;
    }
  }

  if (ack->cmd_id == "grab_control") {
    got_grab_ack = true;
    // Make sure got and checked access state before quitting test
    if (got_state) {
      test_done = true;
    }
  }
}

void CWCAckCallback(ff_msgs::AckStampedConstPtr const& ack) {
  EXPECT_EQ(ack->cmd_id, cmd_id.c_str());
  EXPECT_EQ(ack->cmd_origin, cmd_origin.c_str());
  EXPECT_EQ(ack->status.status, ff_msgs::AckStatus::COMPLETED);
  EXPECT_EQ(ack->completed_status.status, ff_msgs::AckCompletedStatus::OK);

  // send grab control command if just requested control and received cookie
  if (ack->cmd_id == "request_control") {
    got_request_ack = true;
    if (cookie != "") {
      cmd_id = "grab_control";
      PublishGrabControl();
      return;
    }
  }

  if (ack->cmd_id == "grab_control") {
    // Send a command and make sure it is published on the /command topic (i.e.
    // make sure it got passed on to the executive.
    cmd_name = ff_msgs::CommandConstants::CMD_NAME_NO_OP;
    cmd_id = "noOp";
    PublishCommand(NULL);
  }
}

void FailedAckCallback(ff_msgs::AckStampedConstPtr const& ack) {
  EXPECT_EQ(ack->cmd_id, cmd_id.c_str());
  EXPECT_EQ(ack->cmd_origin, cmd_origin.c_str());
  EXPECT_EQ(ack->status.status, ff_msgs::AckStatus::COMPLETED);

  // Two diffent failures bad syntex and execution failed
  // execution failed only occurs for the invalid cookie test
  if (ack->cmd_id == "invalid_cookie") {
    EXPECT_EQ(ack->completed_status.status,
              ff_msgs::AckCompletedStatus::EXEC_FAILED);
  } else {
    EXPECT_EQ(ack->completed_status.status,
              ff_msgs::AckCompletedStatus::BAD_SYNTAX);
  }
  test_done = true;
}

// Only used for successful commands that get passed to the executive. Thus the
// test is finished
void CmdCallback(ff_msgs::CommandStampedConstPtr const& cmd) {
  EXPECT_EQ(cmd->cmd_name, cmd_name);
  test_done = true;
}

void StateCallback(ff_msgs::AccessControlStateStampedConstPtr const& state) {
  cookie = state->cookie;
  if (cookie != "" && got_request_ack) {
    cmd_id = "grab_control";
    cmd_origin = "grab_control";
    PublishGrabControl();
    return;
  }

  if (sent_grab) {
    EXPECT_EQ(state->controller, controller);
    got_state = true;
    // Make sure got and checked ack before quitting
    if (got_grab_ack) {
      test_done = true;
    }
  }
}


// Test Grab Control
TEST(access_control, GrabControl) {
  ros::NodeHandle n;
  ack_sub = n.subscribe<ff_msgs::AckStamped>(TOPIC_MANAGEMENT_ACK, 10,
                                             &GCAckCallback);
  state_sub = n.subscribe<ff_msgs::AccessControlStateStamped>(
                    TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE, 10, &StateCallback);
  cmd_pub = n.advertise<ff_msgs::CommandStamped>(
                                          TOPIC_COMMUNICATIONS_DDS_COMMAND, 10);

  // Reinitialize global variables
  test_done = false;
  sent_grab = false;
  got_request_ack = false;
  got_grab_ack = false;
  got_state = false;
  cookie = "";
  controller = "operator1";

  if (ack_sub.getNumPublishers() == 0 || state_sub.getNumPublishers() == 0 ||
      cmd_pub.getNumSubscribers() == 0) {
    ros::Duration(1.0).sleep();
  }

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_REQUEST_CONTROL;
  cmd_id = "request_control";
  cmd_origin = "request_control";
  PublishCommand(NULL);

  while (!test_done) {
    ros::spinOnce();
  }

  ack_sub.shutdown();
  state_sub.shutdown();
  cmd_pub.shutdown();
}

// Test sending a command while having control
TEST(access_control, CommandWithControl) {
  ros::NodeHandle n;
  cmd_sub = n.subscribe<ff_msgs::CommandStamped>(TOPIC_COMMAND, 10,
                                                 &CmdCallback);
  ack_sub = n.subscribe<ff_msgs::AckStamped>(TOPIC_MANAGEMENT_ACK, 10,
                                             &CWCAckCallback);
  state_sub = n.subscribe<ff_msgs::AccessControlStateStamped>(
                    TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE, 10, &StateCallback);
  cmd_pub = n.advertise<ff_msgs::CommandStamped>(
                                          TOPIC_COMMUNICATIONS_DDS_COMMAND, 10);

  if (cmd_sub.getNumPublishers() == 0 || ack_sub.getNumPublishers() == 0 ||
      state_sub.getNumPublishers() == 0 || cmd_pub.getNumSubscribers() == 0) {
    ros::Duration(1.0).sleep();
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
  cmd_origin = "request_control";
  PublishCommand(NULL);

  while (!test_done) {
    ros::spinOnce();
  }

  cmd_sub.shutdown();
  ack_sub.shutdown();
  state_sub.shutdown();
  cmd_pub.shutdown();
}

// Test sending a stop command without being the operator in control
TEST(access_control, StopCommandWithoutControl) {
  ros::NodeHandle n;
  cmd_sub = n.subscribe<ff_msgs::CommandStamped>(TOPIC_COMMAND, 10,
                                                 &CmdCallback);
  cmd_pub = n.advertise<ff_msgs::CommandStamped>(
                                          TOPIC_COMMUNICATIONS_DDS_COMMAND, 10);

  if (cmd_sub.getNumPublishers() == 0 || cmd_pub.getNumSubscribers() == 0) {
    ros::Duration(1.0).sleep();
  }
  test_done = false;
  controller = "engineer";

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_STOP_ALL_MOTION;
  cmd_id = "stop";
  cmd_origin = "stop";
  PublishCommand(NULL);

  while (!test_done) {
    ros::spinOnce();
  }

  cmd_sub.shutdown();
  cmd_pub.shutdown();
}

// Test sending an idle propulsion command without being the operator in control
TEST(access_control, IdlePropulsionCommandWithoutControl) {
  ros::NodeHandle n;
  cmd_sub = n.subscribe<ff_msgs::CommandStamped>(TOPIC_COMMAND, 10,
                                                 &CmdCallback);
  cmd_pub = n.advertise<ff_msgs::CommandStamped>(
                                          TOPIC_COMMUNICATIONS_DDS_COMMAND, 10);

  if (cmd_sub.getNumPublishers() == 0 || cmd_pub.getNumSubscribers() == 0) {
    ros::Duration(1.0).sleep();
  }
  test_done = false;
  controller = "engineer";

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_IDLE_PROPULSION;
  cmd_id = "idle_propulsion";
  cmd_origin = "idle_propulsion";
  PublishCommand(NULL);

  while (!test_done) {
    ros::spinOnce();
  }

  cmd_sub.shutdown();
  cmd_pub.shutdown();
}

// Test sending a grab control command with an invalid cookie
TEST(access_control, GrabControlCommandInvalidCookie) {
  ros::NodeHandle n;
  ack_sub = n.subscribe<ff_msgs::AckStamped>(TOPIC_MANAGEMENT_ACK, 10,
                                             &FailedAckCallback);
  cmd_pub = n.advertise<ff_msgs::CommandStamped>(
                                          TOPIC_COMMUNICATIONS_DDS_COMMAND, 10);

  if (cmd_pub.getNumSubscribers() == 0 || ack_sub.getNumPublishers() == 0) {
    ros::Duration(1.0).sleep();
  }
  test_done = false;
  controller = "rogue_operator";

  cmd_id = "invalid_cookie";
  cmd_origin = "invalid_cookie";
  cookie = "yummy_cookies";
  PublishGrabControl();

  while (!test_done) {
    ros::spinOnce();
  }

  cmd_pub.shutdown();
  ack_sub.shutdown();
}

// Test sending a grab control command with no arguements
TEST(access_control, GrabControlNoArgs) {
  ros::NodeHandle n;
  ack_sub = n.subscribe<ff_msgs::AckStamped>(TOPIC_MANAGEMENT_ACK, 10,
                                             &FailedAckCallback);
  cmd_pub = n.advertise<ff_msgs::CommandStamped>(
                                          TOPIC_COMMUNICATIONS_DDS_COMMAND, 10);

  if (cmd_pub.getNumSubscribers() == 0 || ack_sub.getNumPublishers() == 0) {
    ros::Duration(1.0).sleep();
  }
  test_done = false;
  controller = "rogue_operator";

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_GRAB_CONTROL;
  cmd_id = "no_args";
  cmd_origin = "no_args";
  PublishCommand(NULL);

  while (!test_done) {
    ros::spinOnce();
  }

  cmd_pub.shutdown();
  ack_sub.shutdown();
}

// Test sending a grab control command with the wrong argument type
TEST(access_control, GrabControlWrongArgType) {
  ros::NodeHandle n;
  ack_sub = n.subscribe<ff_msgs::AckStamped>(TOPIC_MANAGEMENT_ACK, 10,
                                             &FailedAckCallback);
  cmd_pub = n.advertise<ff_msgs::CommandStamped>(
                                          TOPIC_COMMUNICATIONS_DDS_COMMAND, 10);

  if (cmd_pub.getNumSubscribers() == 0 || ack_sub.getNumPublishers() == 0) {
    ros::Duration(1.0).sleep();
  }
  test_done = false;
  controller = "rogue_operator";

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_GRAB_CONTROL;
  cmd_id = "invalid_argument";
  cmd_origin = "invalid_argument";
  std::vector<ff_msgs::CommandArg> args;
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_BOOL;
  arg.b = false;
  args.push_back(arg);
  PublishCommand(&args);

  while (!test_done) {
    ros::spinOnce();
  }

  ack_sub.shutdown();
  cmd_pub.shutdown();
}

// Test sending a command without being the operator in control
TEST(access_control, CommandWithoutControl) {
  ros::NodeHandle n;
  ack_sub = n.subscribe<ff_msgs::AckStamped>(TOPIC_MANAGEMENT_ACK, 10,
                                             &FailedAckCallback);
  cmd_pub = n.advertise<ff_msgs::CommandStamped>(
                                    TOPIC_COMMUNICATIONS_DDS_COMMAND, 10);

  if (cmd_pub.getNumSubscribers() == 0 || ack_sub.getNumPublishers() == 0) {
    ros::Duration(1.0).sleep();
  }
  test_done = false;
  controller = "rogue_operator";

  cmd_name = ff_msgs::CommandConstants::CMD_NAME_NO_OP;
  cmd_id = "rogue_operator";
  cmd_origin = "rogue_operator";
  PublishCommand(NULL);

  while (!test_done) {
    ros::spinOnce();
  }

  cmd_pub.shutdown();
  ack_sub.shutdown();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Initialize the gtesttest framework
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  // Initialize ROS
  ros::init(argc, argv, "test_access_control", ros::init_options::AnonymousName);
  // Run all test procedures
  return RUN_ALL_TESTS();
}
