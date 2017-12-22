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

// Include RPOS
#include <ros/ros.h>

// FSW includes
#include <ff_util/ff_names.h>
#include <ff_util/ff_action.h>

// Action
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
DEFINE_bool(dock, false, "Send a dock command");
DEFINE_bool(undock, false, "Send an undock command");
DEFINE_string(berth, "left", "Berth (left or right, for docking only)");

// Timeout values
DEFINE_double(connect, 10.0, "Action connect timeout");
DEFINE_double(active, 10.0, "Action active timeout");
DEFINE_double(response, 30.0, "Action response timeout");
DEFINE_double(deadline, -1.0, "Action deadline timeout");

// Match the internal states and responses with the message definition
using STATE = ff_msgs::DockState;

// Arm action feedback
void FeedbackCallback(ff_msgs::DockFeedbackConstPtr const& feedback) {
  // Determine the state
  std::string state = "UNKNOWN";
  switch (feedback->state.state) {
  default:
  case STATE::UNKNOWN:
    state = "UNKNOWN";                                            break;
  case STATE::INITIALIZING:
    state = "INITIALIZING";                                       break;
  case STATE::UNDOCKED:
    state = "UNDOCKED";                                           break;
  case STATE::DOCKING_SWITCHING_TO_AR_LOC:
    state = "DOCKING_SWITCHING_TO_AR_LOC";                        break;
  case STATE::DOCKING_MOVING_TO_APPROACH_POSE:
    state = "DOCKING_MOVING_TO_APPROACH_POSE";                    break;
  case STATE::DOCKING_MOVING_TO_COMPLETE_POSE:
    state = "DOCKING_MOVING_TO_COMPLETE_POSE";                    break;
  case STATE::DOCKING_CHECKING_ATTACHED:
    state = "DOCKING_CHECKING_ATTACHED";                          break;
  case STATE::DOCKING_WAITING_FOR_SPIN_DOWN:
    state = "DOCKING_WAITING_FOR_SPIN_DOWN";                      break;
  case STATE::DOCKING_SWITCHING_TO_NO_LOC:
    state = "DOCKING_SWITCHING_TO_NO_LOC";                        break;
  case STATE::DOCKED:
    state = "DOCKED";                                             break;
  case STATE::UNDOCKING_SWITCHING_TO_ML_LOC:
    state = "UNDOCKING_SWITCHING_TO_ML_LOC";                      break;
  case STATE::UNDOCKING_WAITING_FOR_SPIN_UP:
    state = "UNDOCKING_WAITING_FOR_SPIN_UP";                      break;
  case STATE::UNDOCKING_MOVING_TO_APPROACH_POSE:
    state = "UNDOCKING_MOVING_TO_APPROACH_POSE";                  break;
  }
  // Print out a status message
  std::cout << "\r                                                   "
            << "\rState: " << state << std::flush;
}

// Arm action result
void ResultCallback(ff_util::FreeFlyerActionState::Enum code,
  ff_msgs::DockResultConstPtr const& result) {
  std::cout << std::endl << "Result: ";
  // Print general response code
  switch (code) {
  case ff_util::FreeFlyerActionState::Enum::SUCCESS:
    std::cout << "Action completed successfully";      break;
  case ff_util::FreeFlyerActionState::Enum::PREEMPTED:
    std::cout << "Action was preempted";               break;
  case ff_util::FreeFlyerActionState::Enum::ABORTED:
    std::cout << "Action was aborted by the server";   break;
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
  std::cout << std::endl << "Message: ";
  switch (result->response) {
  case ff_msgs::DockResult::CANCELLED:
    std::cout << "CANCELLED";                          break;
  case ff_msgs::DockResult::ALREADY_DOCKED:
    std::cout << "ALREADY_DOCKED";                     break;
  case ff_msgs::DockResult::ALREADY_UNDOCKED:
    std::cout << "ALREADY_UNDOCKED";                   break;
  case ff_msgs::DockResult::UNDOCKED:
    std::cout << "UNDOCKED";                           break;
  case ff_msgs::DockResult::DOCKED:
    std::cout << "DOCKED";                             break;
  case ff_msgs::DockResult::PREEMPTED:
    std::cout << "PREEMPTED";                          break;
  case ff_msgs::DockResult::INVALID_COMMAND:
    std::cout << "INVALID_COMMAND";                    break;
  case ff_msgs::DockResult::INVALID_BERTH:
    std::cout << "INVALID_BERTH";                      break;
  case ff_msgs::DockResult::NOT_IN_UNDOCKED_STATE:
    std::cout << "NOT_IN_UNDOCKED_STATE";              break;
  case ff_msgs::DockResult::NOT_IN_DOCKED_STATE:
    std::cout << "NOT_IN_DOCKED_STATE";                break;
  case ff_msgs::DockResult::SWITCH_TO_ML_FAILED:
    std::cout << "SWITCH_TO_ML_FAILED";                break;
  case ff_msgs::DockResult::SWITCH_TO_AR_FAILED:
    std::cout << "SWITCH_TO_AR_FAILED";                break;
  case ff_msgs::DockResult::SWITCH_TO_NO_FAILED:
    std::cout << "SWITCH_TO_NO_FAILED";                break;
  case ff_msgs::DockResult::PREP_DISABLE_FAILED:
    std::cout << "PREP_DISABLE_FAILED";                break;
  case ff_msgs::DockResult::PREP_ENABLE_FAILED:
    std::cout << "PREP_ENABLE_FAILED";                 break;
  case ff_msgs::DockResult::MOTION_APPROACH_FAILED:
    std::cout << "MOTION_APPROACH_FAILED";             break;
  case ff_msgs::DockResult::MOTION_COMPLETE_FAILED:
    std::cout << "MOTION_COMPLETE_FAILED";             break;
  case ff_msgs::DockResult::MOTION_ATTACHED_FAILED:
    std::cout << "MOTION_ATTACHED_FAILED";             break;
  case ff_msgs::DockResult::EPS_UNDOCK_FAILED:
    std::cout << "EPS_UNDOCK_FAILED";                  break;
  case ff_msgs::DockResult::EPS_DOCK_FAILED:
    std::cout << "EPS_DOCK_FAILED";                    break;
  }
teardown:
  std::cout << std::endl;
  // In all cases we must shutdown
  ros::shutdown();
}

// Ensure all clients are connected
void ConnectedCallback(
  ff_util::FreeFlyerActionClient<ff_msgs::DockAction> *client) {
  // Check to see if connected
  if (!client->IsConnected()) return;
  // Print out a status message
  std::cout << "\r                                                   "
            << "\rState: CONNECTED" << std::flush;
  // Prepare the goal
  ff_msgs::DockGoal goal;
  if (FLAGS_dock) {
    goal.command = ff_msgs::DockGoal::DOCK;
    if (FLAGS_berth == "right")
      goal.berth = ff_msgs::DockGoal::BERTH_RIGHT;
    else
      goal.berth = ff_msgs::DockGoal::BERTH_LEFT;
  } else if (FLAGS_undock) {
    goal.command = ff_msgs::DockGoal::UNDOCK;
  }
  client->SendGoal(goal);
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "control", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun dock dock_tool <opts>");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Some simple checks
  uint8_t cmd = 0;
  if (FLAGS_dock)   cmd++;
  if (FLAGS_undock) cmd++;
  // Check we have specified one of the required switches
  if (cmd != 1) {
    std::cerr << "You must specify one of -dock or -undock" << std::endl;
    return 1;
  }
  // Check that the berth makes sense
  if (FLAGS_berth != "left" && FLAGS_berth != "right") {
    std::cerr << "The berth must be 1 or 2" << std::endl;
    return 1;
  }
  // Action clients
  ff_util::FreeFlyerActionClient<ff_msgs::DockAction> client;
  // Create a node handle
  ros::NodeHandle nh(std::string("/") + FLAGS_ns);
  // Setup SWITCH action
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
  client.Create(&nh, ACTION_PROCEDURES_DOCK);
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
