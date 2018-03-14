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
#include <ff_util/config_client.h>

// Action
#include <ff_msgs/ArmState.h>
#include <ff_msgs/ArmAction.h>
#include <ff_msgs/JointSampleStamped.h>
#include <ff_msgs/JointSample.h>

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

// Argument based ommands
DEFINE_double(pan, INFINITY, "Pan the arm (in degrees -90 to 90)");
DEFINE_double(tilt, INFINITY, "Tilt the arm (in degrees -20 to 90)");
DEFINE_double(set, INFINITY, "Set the gripper (in percent)");

// Toggle based commands
DEFINE_bool(open, false, "Open the gripper");
DEFINE_bool(close, false, "Close the gripper");
DEFINE_bool(stow, false, "Stow the arm");
DEFINE_bool(deploy, false, "Deploy the arm");
DEFINE_bool(cal, false, "Calibrate the gripper");
DEFINE_bool(stop, false, "Stop the arm");

// Action timeout values
DEFINE_double(connect, 10.0, "Action connect timeout");
DEFINE_double(active, 60.0, "Action active timeout");
DEFINE_double(response, 60.0, "Action response timeout");
DEFINE_double(deadline, -1.0, "Action deadline timeout");

// Arm action result
void ResultCallback(ff_util::FreeFlyerActionState::Enum code,
  ff_msgs::ArmResultConstPtr const& result) {
  // Print general response code
  std::cout << std::endl << "Response: ";
  switch (code) {
  // Result will be a null pointer
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_CONNECT:
    std::cout << "Timeout on connecting to action" << std::endl;
    break;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE:
    std::cout << "Timeout on action going active" << std::endl;
    break;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE:
    std::cout << "Timeout on receiving a response" << std::endl;
    break;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE:
    std::cout << "Timeout on result deadline" << std::endl;
    break;
  // Result expected
  case ff_util::FreeFlyerActionState::Enum::SUCCESS:
  case ff_util::FreeFlyerActionState::Enum::PREEMPTED:
  case ff_util::FreeFlyerActionState::Enum::ABORTED: {
    // Print a meningful response
    switch (result->response) {
      case ff_msgs::ArmResult::SUCCESS:
        std::cout << "Successfully completed";             break;
      case ff_msgs::ArmResult::PREEMPTED:
        std::cout << "Action was preempted";               break;
      case ff_msgs::ArmResult::INVALID_COMMAND:
        std::cout << "Invalid command";                    break;
      case ff_msgs::ArmResult::BAD_TILT_VALUE:
        std::cout << "Invalid value for tilt";             break;
      case ff_msgs::ArmResult::BAD_PAN_VALUE:
        std::cout << "Invalid value for pan";              break;
      case ff_msgs::ArmResult::BAD_GRIPPER_VALUE:
        std::cout << "Invalid value for gripper";          break;
      case ff_msgs::ArmResult::NOT_ALLOWED:
        std::cout << "Not allowed";                        break;
      case ff_msgs::ArmResult::NEED_TO_CALIBRATE:
        std::cout << "Cannot OPEN/CLOSE/SET gripper";      break;
      case ff_msgs::ArmResult::TILT_FAILED:
        std::cout << "Tilt command failed";                break;
      case ff_msgs::ArmResult::PAN_FAILED:
        std::cout << "Pan command failed";                 break;
      case ff_msgs::ArmResult::GRIPPER_FAILED:
        std::cout << "Gripper command failed";             break;
      case ff_msgs::ArmResult::COMMUNICATION_ERROR:
        std::cout << "Cannot communicate with arm";        break;
      case ff_msgs::ArmResult::COLLISION_AVOIDED:
        std::cout << "Panning disabled when tilt > 90";    break;
      }
    }
    std::cout << std::endl;
  default:
    break;
  }
  // In all cases we must shutdown
  ros::shutdown();
}

// Arm action feedback
void FeedbackCallback(ff_msgs::ArmFeedbackConstPtr const& feedback) {
  // Determine the state
  static std::string str;
  switch (feedback->state.state) {
  default:
  case ff_msgs::ArmState::UNKNOWN:           str = "UNKNOWN";           break;
  case ff_msgs::ArmState::INITIALIZING:      str = "INITIALIZING";      break;
  case ff_msgs::ArmState::STOWED:            str = "STOWED";            break;
  case ff_msgs::ArmState::DEPLOYED:          str = "DEPLOYED";          break;
  case ff_msgs::ArmState::PANNING:           str = "PANNING";           break;
  case ff_msgs::ArmState::TILTING:           str = "TILTING";           break;
  case ff_msgs::ArmState::SETTING:           str = "SETTING";           break;
  case ff_msgs::ArmState::CALIBRATING:       str = "CALIBRATING";       break;
  case ff_msgs::ArmState::STOWING_SETTING:   str = "STOWING_SETTING";   break;
  case ff_msgs::ArmState::STOWING_PANNING:   str = "STOWING_PANNING";   break;
  case ff_msgs::ArmState::STOWING_TILTING:   str = "STOWING_TILTING";   break;
  case ff_msgs::ArmState::DEPLOYING_PANNING: str = "DEPLOYING_PANNING"; break;
  case ff_msgs::ArmState::DEPLOYING_TILTING: str = "DEPLOYING_TILTING"; break;
  }
  // Print out a summary
  std::cout << '\r' << std::flush;
  if (feedback->gripper < 0) {
    std::cout << std::fixed << std::setprecision(2)
      << "PAN: " << feedback->pan << " deg "
      << "TILT: " << feedback->tilt << " deg "
      << "GRIPPER: uncalibrated "
      << "[" << str << "]           ";
  } else {
    std::cout << std::fixed << std::setprecision(2)
      << "PAN: " << feedback->pan << " deg "
      << "TILT: " << feedback->tilt << " deg "
      << "GRIPPER: " << feedback->gripper << " deg "
      << "[" << str << "]           ";
  }
}

// Ensure all clients are connected
void ConnectedCallback(
  ff_util::FreeFlyerActionClient<ff_msgs::ArmAction> *client) {
  // Check to see if connected
  if (!client->IsConnected()) return;
  // Print out a status message
  std::cout << '\r' << std::flush
            << "State: CONNECTED"
            << "                                 ";
  // Prepare the goal
  ff_msgs::ArmGoal goal;
  if (FLAGS_open)        goal.command = ff_msgs::ArmGoal::GRIPPER_OPEN;
  else if (FLAGS_close)  goal.command = ff_msgs::ArmGoal::GRIPPER_CLOSE;
  else if (FLAGS_cal)    goal.command = ff_msgs::ArmGoal::GRIPPER_CALIBRATE;
  else if (FLAGS_stow)   goal.command = ff_msgs::ArmGoal::ARM_STOW;
  else if (FLAGS_deploy) goal.command = ff_msgs::ArmGoal::ARM_DEPLOY;
  else if (FLAGS_stop)   goal.command = ff_msgs::ArmGoal::ARM_STOP;
  if (!std::isinf(FLAGS_set)) {
    goal.command = ff_msgs::ArmGoal::GRIPPER_SET;
    goal.gripper = FLAGS_set;
  }
  if (!std::isinf(FLAGS_pan)) {
    goal.command = ff_msgs::ArmGoal::ARM_PAN;
    goal.pan = FLAGS_pan;
  }
  if (!std::isinf(FLAGS_tilt)) {
    goal.command = ff_msgs::ArmGoal::ARM_TILT;
    goal.tilt= FLAGS_tilt;
  }
  if (!std::isinf(FLAGS_pan) && !std::isinf(FLAGS_tilt))
    goal.command = ff_msgs::ArmGoal::ARM_MOVE;
  // Send the goal
  client->SendGoal(goal);
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "control", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun arm arm_tool <opts>");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Some simple checks
  uint8_t cmd = 0;
  if (!std::isinf(FLAGS_pan) || !std::isinf(FLAGS_tilt)) cmd++;
  if (!std::isinf(FLAGS_set)) cmd++;
  if (FLAGS_open) cmd++;
  if (FLAGS_close) cmd++;
  if (FLAGS_stow) cmd++;
  if (FLAGS_deploy) cmd++;
  if (FLAGS_cal) cmd++;
  if (FLAGS_stop) cmd++;
  // Check we have specified one of the required switches
  if (cmd != 1) {
    std::cerr << "You must specify one of: " << std::endl;
    std::cerr << "> -pan and/or -tilt" << std::endl;
    std::cerr << "> -grip, -open, -close -stow, -deploy, -stop or -cal"
              << std::endl;
    return 1;
  }
  // Action clients
  ff_util::FreeFlyerActionClient<ff_msgs::ArmAction> client;
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
  client.Create(&nh, ACTION_BEHAVIORS_ARM);
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
