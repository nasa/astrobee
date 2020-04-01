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

// Command line flags
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>

// Include ROS
#include <ros/ros.h>

// FSW includes
#include <ff_util/ff_names.h>
#include <ff_util/ff_action.h>

// Action
#include <ff_msgs/ReturnToDockAction.h>
#include <ff_msgs/DockAction.h>

// C++ STL inclues
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <cmath>

// Robot namespace
DEFINE_string(ns, "", "Robot namespace");

// Toggle based commands
// DEFINE_bool(rtd, false, "Send a return to dock command");
DEFINE_int32(berth, 0, "Berth ID (1 = left, 2 = right)");

// Timeout values
DEFINE_double(connect, 1000.0, "Action connect timeout");
DEFINE_double(active, 1000.0, "Action active timeout");
DEFINE_double(response, 1000.0, "Action response timeout");
DEFINE_double(deadline, -1.0, "Action deadline timeout");

// Match the internal states and responses with the message definition
using STATE = ff_msgs::ReturnToDockState;

// Dock action feedback
void FeedbackCallback(ff_msgs::ReturnToDockFeedbackConstPtr const& feedback) {
  std::cout << "\r                                                   "
            << "\rFSM: " << feedback->state.fsm_event
            << " -> " << feedback->state.fsm_state << std::flush;
}

// Dock action result
void ResultCallback(ff_util::FreeFlyerActionState::Enum code,
  ff_msgs::ReturnToDockResultConstPtr const& result) {
  std::cout << std::endl << "Result: ";
  // Print general response code
  switch (code) {
  case ff_util::FreeFlyerActionState::Enum::SUCCESS:
    std::cout << "[SUCCESS] ";      break;
  case ff_util::FreeFlyerActionState::Enum::PREEMPTED:
    std::cout << "[PREEMPT] ";               break;
  case ff_util::FreeFlyerActionState::Enum::ABORTED:
    std::cout << "[ABORTED] ";   break;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_CONNECT:
    std::cout << "Action timed out on connect";        goto teardown;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE:
    std::cout << "Action timed out on active";         goto teardown;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE:
    std::cout << "Action timed out on response";       goto teardown;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE:
    std::cout << "Action timed out on deadline";       goto teardown;
  }
  // If we get there then we have some response data
  std::cout << result->fsm_result
            << " (Code " << result->response << ")" << std::endl;
teardown:
  std::cout << std::endl;
  // In all cases we must shutdown
  ros::shutdown();
}

// Ensure all clients are connected
void ConnectedCallback(
  ff_util::FreeFlyerActionClient<ff_msgs::ReturnToDockAction> *client) {
  // Check to see if connected
  std::cout << "ConnectedCallback" << std::flush;
  if (!client->IsConnected()) return;
  // Print out a status message
  std::cout << "\r                                                   "
            << "\rState: CONNECTED" << std::flush;
  // Prepare the goal
  ff_msgs::ReturnToDockGoal goal;
  goal.command = ff_msgs::ReturnToDockGoal::RETURN_TO_DOCK;
  switch (FLAGS_berth) {
  case 1: goal.berth = ff_msgs::ReturnToDockGoal::BERTH_1; break;
  case 2: goal.berth = ff_msgs::ReturnToDockGoal::BERTH_2; break;
  default:
    std::cout << "Berth not specified or not valid" << std::endl;
    break;
  }
  client->SendGoal(goal);
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "return_to_dock_tool", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun return_to_dock return_to_dock_tool <opts>");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Action clients
  ff_util::FreeFlyerActionClient<ff_msgs::ReturnToDockAction> client;
  // Create a node handle
  ros::NodeHandle nh(std::string("/") + FLAGS_ns);
  // Setup return to dock action
  client.SetConnectedTimeout(FLAGS_connect);
  client.SetActiveTimeout(FLAGS_active);
  client.SetResponseTimeout(FLAGS_response);
  if (FLAGS_deadline > 0)
    client.SetDeadlineTimeout(FLAGS_deadline);
  client.SetFeedbackCallback(std::bind(FeedbackCallback,
    std::placeholders::_1));
  client.SetResultCallback(std::bind(ResultCallback,
    std::placeholders::_1, std::placeholders::_2));
  client.SetConnectedCallback(std::bind(ConnectedCallback, &client));
  client.Create(&nh, ACTION_BEHAVIORS_RETURN_TO_DOCK);
  // Print out a status message
  std::cout << "\r                                                   "
            << "\rState: CONNECTING" << std::flush;
  // Synchronous mode
  ros::spin();
  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
