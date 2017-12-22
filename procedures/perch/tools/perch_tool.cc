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
#include <ff_msgs/PerchAction.h>

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
DEFINE_bool(perch, false, "Send a perch command");
DEFINE_bool(unperch, false, "Send an unperch command");

// Timeout values
DEFINE_double(connect, 10.0, "Action connect timeout");
DEFINE_double(active, 10.0, "Action active timeout");
DEFINE_double(response, 10.0, "Action response timeout");
DEFINE_double(deadline, -1.0, "Action deadline timeout");

// Arm action feedback
void FeedbackCallback(ff_msgs::PerchFeedbackConstPtr const& feedback) {
  // Determine the state
  std::string state = "UNKNOWN";
  switch (feedback->state) {
  default:
  case ff_msgs::PerchFeedback::UNKNOWN:
    state = "UNKNOWN";                                              break;
  case ff_msgs::PerchFeedback::INITIALIZING:
    state = "INITIALIZING";                                         break;
  case ff_msgs::PerchFeedback::TESTING_SWITCHING_TO_HR_LOC:
    state = "TESTING_SWITCHING_TO_HR_LOC";                          break;
  case ff_msgs::PerchFeedback::TESTING_SWITCHING_TO_PL_LOC:
    state = "TESTING_SWITCHING_TO_PL_LOC";                          break;
  case ff_msgs::PerchFeedback::UNPERCHED:
    state = "UNPERCHED";                                            break;
  case ff_msgs::PerchFeedback::PERCHING_SWITCHING_TO_HR_LOC:
    state = "PERCHING_SWITCHING_TO_HR_LOC";                         break;
  case ff_msgs::PerchFeedback::PERCHING_MOVING_TO_APPROACH_POSE:
    state = "PERCHING_MOVING_TO_APPROACH_POSE";                     break;
  case ff_msgs::PerchFeedback::PERCHING_DEPLOYING_ARM:
    state = "PERCHING_DEPLOYING_ARM";                               break;
  case ff_msgs::PerchFeedback::PERCHING_OPENING_GRIPPER:
    state = "PERCHING_OPENING_GRIPPER";                             break;
  case ff_msgs::PerchFeedback::PERCHING_CALIBRATING_GRIPPER:
    state = "PERCHING_CALIBRATING_GRIPPER";                         break;
  case ff_msgs::PerchFeedback::PERCHING_INGRESSING:
    state = "PERCHING_INGRESSING";                                  break;
  case ff_msgs::PerchFeedback::PERCHING_CLOSING_GRIPPER:
    state = "PERCHING_CLOSING_GRIPPER";                             break;
  case ff_msgs::PerchFeedback::PERCHING_CHECKING_ATTACHED:
    state = "PERCHING_CHECKING_ATTACHED";                           break;
  case ff_msgs::PerchFeedback::PERCHING_SWITCHING_TO_PL_LOC:
    state = "PERCHING_SWITCHING_TO_PL_LOC";                         break;
  case ff_msgs::PerchFeedback::PERCHED:
    state = "PERCHED";                                              break;
  case ff_msgs::PerchFeedback::UNPERCHING_RESETTING_PAN_TILT:
    state = "UNPERCHING_RESETTING_PAN_TILT";                        break;
  case ff_msgs::PerchFeedback::UNPERCHING_SWITCHING_TO_HR_LOC:
    state = "UNPERCHING_SWITCHING_TO_HR_LOC";                       break;
  case ff_msgs::PerchFeedback::UNPERCHING_OPENING_GRIPPER:
    state = "UNPERCHING_OPENING_GRIPPER";                           break;
  case ff_msgs::PerchFeedback::UNPERCHING_CALIBRATING_GRIPPER:
    state = "UNPERCHING_CALIBRATING_GRIPPER";                       break;
  case ff_msgs::PerchFeedback::UNPERCHING_EGRESSING:
    state = "UNPERCHING_EGRESSING";                                 break;
  case ff_msgs::PerchFeedback::UNPERCHING_CLOSING_GRIPPER:
    state = "UNPERCHING_CLOSING_GRIPPER";                           break;
  case ff_msgs::PerchFeedback::UNPERCHING_STOWING_ARM:
    state = "UNPERCHING_STOWING_ARM";                               break;
  case ff_msgs::PerchFeedback::UNPERCHING_SWITCHING_TO_ML_LOC:
    state = "UNPERCHING_SWITCHING_TO_ML_LOC";                       break;
  }
  // Print out a status message
  std::cout << '\r' << std::flush
            << "State: " << state
            << "                                 ";
}

// Arm action result
void ResultCallback(ff_util::FreeFlyerActionState::Enum code,
  ff_msgs::PerchResultConstPtr const& result) {
  // Print out a status message that includes the state and joint angles
  std::cout << std::endl << "Response: ";
  // Print general response code
  switch (code) {
  case ff_util::FreeFlyerActionState::Enum::SUCCESS:
    std::cout << "Action completed successfully";      break;
  case ff_util::FreeFlyerActionState::Enum::PREEMPTED:
    std::cout << "Action was preempted";               break;
  case ff_util::FreeFlyerActionState::Enum::ABORTED:
    std::cout << "Action was aborted by the server";   break;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_CONNECT:
    std::cout << "Action timed out on connect";        return;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE:
    std::cout << "Action timed out on active";         return;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE:
    std::cout << "Action timed out on response";       return;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE:
    std::cout << "Action timed out on deadline";       return;
  }
  // If we get there then we have some response data
  std::cout << std::endl << "Error: ";
  switch (result->response) {
  case ff_msgs::PerchResult::CANCELLED:
    std::cout << "CANCELLED";                          break;
  case ff_msgs::PerchResult::ALREADY_PERCHED:
    std::cout << "ALREADY_PERCHED";                    break;
  case ff_msgs::PerchResult::ALREADY_UNPERCHED:
    std::cout << "ALREADY_UNPERCHED";                  break;
  case ff_msgs::PerchResult::UNPERCHED:
    std::cout << "UNPERCHED";                          break;
  case ff_msgs::PerchResult::PERCHED:
    std::cout << "PERCHED";                            break;
  case ff_msgs::PerchResult::PREEMPTED:
    std::cout << "PREEMPTED";                          break;
  case ff_msgs::PerchResult::INVALID_COMMAND:
    std::cout << "INVALID_COMMAND";                    break;
  case ff_msgs::PerchResult::SWITCH_TO_HR_LOC_FAILED:
    std::cout << "SWITCH_TO_HR_LOC_FAILED";            break;
  case ff_msgs::PerchResult::SWITCH_TO_PL_LOC_FAILED:
    std::cout << "SWITCH_TO_PL_LOC_FAILED";            break;
  case ff_msgs::PerchResult::SWITCH_TO_ML_LOC_FAILED:
    std::cout << "SWITCH_TO_ML_LOC_FAILED";            break;
  case ff_msgs::PerchResult::APPROACH_FAILED:
    std::cout << "APPROACH_FAILED";                    break;
  case ff_msgs::PerchResult::DEPLOY_FAILED:
    std::cout << "DEPLOY_FAILED";                      break;
  case ff_msgs::PerchResult::CALIBRATE_FAILED:
    std::cout << "CALIBRATE_FAILED";                   break;
  case ff_msgs::PerchResult::OPEN_FAILED:
    std::cout << "OPEN_FAILED";                        break;
  case ff_msgs::PerchResult::RESET_FAILED:
    std::cout << "RESET_FAILED";                       break;
  case ff_msgs::PerchResult::INGRESS_FAILED:
    std::cout << "INGRESS_FAILED";                     break;
  case ff_msgs::PerchResult::CLOSE_FAILED:
    std::cout << "CLOSE_FAILED";                       break;
  case ff_msgs::PerchResult::EGRESS_FAILED:
    std::cout << "EGRESS_FAILED";                      break;
  case ff_msgs::PerchResult::ATTACH_FAILED:
    std::cout << "ATTACH_FAILED";                      break;
  case ff_msgs::PerchResult::DETACH_FAILED:
    std::cout << "DETACH_FAILED";                      break;
  case ff_msgs::PerchResult::NOT_ALLOWED:
    std::cout << "NOT_ALLOWED";                        break;
  case ff_msgs::PerchResult::TOGGLE_FAILED:
    std::cout << "TOGGLE_FAILED";                      break;
  case ff_msgs::PerchResult::MOTION_FAILED:
    std::cout << "MOTION_FAILED";                      break;
  case ff_msgs::PerchResult::SWITCH_FAILED:
    std::cout << "SWITCH_FAILED";                      break;
  case ff_msgs::PerchResult::ARM_FAILED:
    std::cout << "ARM_FAILED";                         break;
  }
  std::cout << std::endl;
  // In all cases we must shutdown
  ros::shutdown();
}

// Ensure all clients are connected
void ConnectedCallback(
  ff_util::FreeFlyerActionClient<ff_msgs::PerchAction> *client) {
  // Check to see if connected
  if (!client->IsConnected()) return;
  // Print out a status message
  std::cout << '\r' << std::flush
            << "State: CONNECTED"
            << "                                 ";
  // Prepare the goal
  ff_msgs::PerchGoal goal;
  if (FLAGS_perch)
    goal.command = ff_msgs::PerchGoal::PERCH;
  else if (FLAGS_unperch)
    goal.command = ff_msgs::PerchGoal::UNPERCH;
  client->SendGoal(goal);
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "control", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun arm perch_tool <opts>");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Some simple checks
  uint8_t cmd = 0;
  if (FLAGS_perch)   cmd++;
  if (FLAGS_unperch) cmd++;
  // Check we have specified one of the required switches
  if (cmd != 1) {
    std::cerr << "You must specify one of -perch or -unperch" << std::endl;
    return 1;
  }
  // Action clients
  ff_util::FreeFlyerActionClient<ff_msgs::PerchAction> client;
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
  client.Create(&nh, ACTION_PROCEDURES_PERCH);
  // Print out a status message
  std::cout << '\r' << std::flush
            << "State: CONNECTING"
            << "                                 ";
  // Synchronous mode
  ros::spin();
  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
