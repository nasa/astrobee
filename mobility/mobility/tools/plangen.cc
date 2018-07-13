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
#include <ff_util/ff_flight.h>
#include <ff_util/ff_action.h>

// Primitive actions
#include <ff_msgs/PlanAction.h>

// C++ STL inclues
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

// Gflags
DEFINE_string(ns, "", "Robot namespace");
DEFINE_string(planner, "trapezoidal", "Planner name (trapezoidal, qp)");
DEFINE_string(input, "", "Input file with rows [t x y z angle ax_x ax_y ax_z]");
DEFINE_string(output, "", "Output file with linear and angular accelerations");
DEFINE_bool(ff, false, "Plan in face-forward mode");
DEFINE_double(rate, 62.5, "Segment sampling rate");
DEFINE_double(vel, 0.2, "Desired velocity");
DEFINE_double(accel, 0.02, "Desired acceleration");
DEFINE_double(omega, 0.1745, "Desired angular velocity");
DEFINE_double(alpha, 0.1745, "Desired angular acceleration");
DEFINE_double(connect, 30.0, "Plan action connect timeout");
DEFINE_double(active, 30.0, "Plan action active timeout");
DEFINE_double(response, 30.0, "Plan action response timeout");
DEFINE_double(deadline, -1.0, "Plan action deadline timeout");

// Ensure all clients are connected
void ConnectedCallback(
  ff_util::FreeFlyerActionClient<ff_msgs::PlanAction> * client) {
  // Goal metadata
  ff_msgs::PlanGoal plan_goal;
  plan_goal.faceforward = FLAGS_ff;
  plan_goal.desired_vel = FLAGS_vel;
  plan_goal.desired_accel = FLAGS_accel;
  plan_goal.desired_omega = FLAGS_omega;
  plan_goal.desired_alpha = FLAGS_alpha;
  plan_goal.desired_rate = FLAGS_rate;
  plan_goal.check_obstacles = false;
  plan_goal.max_time = ros::Duration(60.0);
  // Add states
  double ts;
  geometry_msgs::PoseStamped msg;
  std::ifstream ifile(FLAGS_input.c_str());
  if (ifile.is_open()) {
    while (ifile >> ts
      >> msg.pose.position.x >> msg.pose.position.y >> msg.pose.position.z
      >> msg.pose.orientation.x >> msg.pose.orientation.y
      >> msg.pose.orientation.z >> msg.pose.orientation.w) {
      msg.header.stamp = ros::Time(ts);
      plan_goal.states.push_back(msg);
    }
    ROS_INFO_STREAM(plan_goal);
    ifile.close();
    client->SendGoal(plan_goal);
    std::cout << "Input read successfully" << std::endl;
    return;
  }
  // Send the goal
  std::cerr << "Input could not be read" << std::endl;
  ros::shutdown();
}

// Planner feedback - simply forward to motion feeedback
void FeedbackCallback(ff_msgs::PlanFeedbackConstPtr const& feedback) {
  std::cout << "Planner feedback received" << std::endl;
}

// Planner result -- trigger an update to the FSM
void ResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
  ff_msgs::PlanResultConstPtr const& result) {
  if (result_code ==  ff_util::FreeFlyerActionState::SUCCESS) {
    std::ofstream ofile(FLAGS_output.c_str());
    if (ofile.is_open()) {
      ff_util::Segment::const_iterator it;
      for (it = result->segment.begin(); it != result->segment.end(); it++) {
        ofile << (it->when - result->segment.begin()->when).toSec()
              << "," << it->accel.linear.x
              << "," << it->accel.linear.y
              << "," << it->accel.linear.z
              << "," << it->accel.angular.x
              << "," << it->accel.angular.y
              << "," << it->accel.angular.z
              << std::endl;
      }
      ofile.close();
      std::cerr << "Output file written successfully" << std::endl;
    } else {
      std::cerr << "Output file could not be written" << std::endl;
    }
  }
  ros::shutdown();
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "plangen", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun mobility plangen <opts>");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_input.empty()) {
    std::cerr << "Please specify an -input file" << std::endl;
    return -1;
  }
  if (FLAGS_output.empty()) {
    std::cerr << "Please specify an -output file" << std::endl;
    return -2;
  }
  // Create a node handle
  ros::NodeHandle nh(std::string("/") + FLAGS_ns);
  // Get the topic
  std::ostringstream oss;
  oss << PREFIX_MOBILITY_PLANNER;
  oss << FLAGS_planner;
  oss << SUFFIX_MOBILITY_PLANNER;
  // Setup a PLAN action
  ff_util::FreeFlyerActionClient<ff_msgs::PlanAction> client_p_;
  client_p_.SetConnectedTimeout(FLAGS_connect);
  client_p_.SetActiveTimeout(FLAGS_active);
  client_p_.SetResponseTimeout(FLAGS_response);
  if (FLAGS_deadline > 0)
    client_p_.SetDeadlineTimeout(FLAGS_deadline);
  client_p_.SetFeedbackCallback(std::bind(FeedbackCallback,
    std::placeholders::_1));
  client_p_.SetResultCallback(std::bind(ResultCallback,
    std::placeholders::_1, std::placeholders::_2));
  client_p_.SetConnectedCallback(std::bind(ConnectedCallback, &client_p_));
  client_p_.Create(&nh, oss.str());
  // Synchronous mode
  ros::spin();
  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
