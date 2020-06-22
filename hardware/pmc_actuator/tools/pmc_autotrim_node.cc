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

// Common freeflyer code
#include <ff_common/init.h>

// Gflag code
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>

// General ROS
#include <ros/ros.h>

// Flight software messages
#include <ff_hw_msgs/EpsHousekeeping.h>
#include <ff_hw_msgs/PmcCommand.h>

// Flight software names
#include <ff_util/ff_names.h>

// STL includes
#include <map>
#include <string>
#include <sstream>
#include <utility>
#include <algorithm>
#include <vector>
#include <numeric>

// Options
DEFINE_string(ns, "", "Robot namespace");
DEFINE_int32(nozzle, 0, "Trim a specific nozzle [1 ... 6 ], 0 = ALL");
DEFINE_int32(idle, 128, "Idle value for all other nozzles");
DEFINE_double(rate, 62.5, "PMC command rate");
DEFINE_double(wait, 10.0, "Time to wait to allow PMCs to spin up");
DEFINE_double(threshold, 0.3, "Current threshold for hitting endstops");
DEFINE_bool(high, true, "Calibrate high stop");
DEFINE_bool(low, true, "Calibrate low stop");
DEFINE_int32(pmc, 0, "PMC [0:port, 1:stbd]");
DEFINE_int32(speed, 132, "PMC motor speed [0 - 255]");
DEFINE_int32(samples, 10, "Number of load samples per test");

typedef std::map<std::pair<size_t, size_t>, std::pair<size_t, size_t>> TrimMap;
TrimMap trims_;
TrimMap::iterator curr_;

// Publisher for commands
ros::Publisher pub_;
ros::Subscriber sub_;
bool ready_ = false;

// Current load
std::vector<double> samples_;

// Called once to print the trim result to screen
void Complete() {
  std::ostringstream oss;
  if (FLAGS_pmc == 0) oss << "robot_port_nozzle_trims = ";
  if (FLAGS_pmc == 1) oss << "robot_stbd_nozzle_trims = ";
  for (size_t i = 0; i < 2; i++) {
    oss << " { ";
    for (size_t j = 0; j < 6; j++)
      oss << trims_[std::make_pair(j, i)].second << ", ";
    oss << " }";
    if (i == 0)
      oss << ",";
    oss << std::endl;
  }
  oss << "}" << std::endl;
  ROS_INFO_STREAM(oss.str());
  ros::shutdown();
}

// EPS telemetry callback
void TelemetryCallback(const ff_hw_msgs::EpsHousekeeping::ConstPtr& msg) {
  if (!ready_)
    return;
  std::string str;
  switch (curr_->first.second) {
  case 0: str == "MOTOR1_I"; break;
  case 1: str == "MOTOR2_I"; break;
  default: return;
  }
  for (auto const& i : msg->values) {
    if (i.name != str)
      continue;
    samples_.push_back(i.value);
    break;
  }
  // Check if we have sufficient samples
  if (samples_.size() < static_cast<size_t>(FLAGS_samples))
    return;
  // Calculate average load
  double load = std::accumulate(samples_.begin(), samples_.end(), 0);
  if (!samples_.empty())
    load /= static_cast<double>(samples_.size());
  // A very simple binary-style search
  int x = (curr_->second.second - curr_->second.first) / 2;
  if (load > FLAGS_threshold)
    curr_->second.first = x;
  else
    curr_->second.second = x;
  // When the interval edges meet, that's the trim we use
  if (curr_->second.second == curr_->second.first) {
    // Advance to the next trim
    if (++curr_ == trims_.end())
      Complete();
  }
  samples_.clear();
}

// PMC command
void CommandCallback(const ros::TimerEvent&) {
  uint8_t rest = static_cast<uint8_t>(FLAGS_idle);
  uint8_t speed = static_cast<uint8_t>(FLAGS_speed);
  // Work out the nozzle (k) and the value (depends on high or low stop)
  size_t k = curr_->first.first;
  int x = (curr_->second.second - curr_->second.first) / 2;
  if (curr_->first.second)
    x = 255 - x;
  // Set PMC to nominal mode, with one side disabled if needed
  ff_hw_msgs::PmcCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.goals.resize(2);
  msg.goals[0].motor_speed = (FLAGS_pmc == 0 ? speed : 0);
  msg.goals[1].motor_speed = (FLAGS_pmc == 1 ? speed : 0);
  for (size_t i = 0; i < 6; i++) {
    msg.goals[0].nozzle_positions[i] = (FLAGS_pmc == 0 && i == k ? x : rest);
    msg.goals[1].nozzle_positions[i] = (FLAGS_pmc == 1 && i == k ? x : rest);
  }
  pub_.publish(msg);
}

void DelayCallback(const ros::TimerEvent&) {
  ready_ = true;
}

// General idea
int main(int argc, char **argv) {
  ros::init(argc, argv, "pmc_autotrim_node");
  google::SetUsageMessage("Usage: rosrun mobility teleop <opts>");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::NodeHandle nh = ros::NodeHandle(std::string("/") + FLAGS_ns);
  // Work out the upper and lower bound
  uint8_t rest = static_cast<uint8_t>(FLAGS_idle);
  size_t nozzle = static_cast<size_t>(FLAGS_nozzle);
  size_t lb = (nozzle && nozzle < 7 ? nozzle : 1);
  size_t ub = (nozzle && nozzle < 7 ? nozzle : 6);
  for (size_t i = lb; i <= ub; i++) {
    if (FLAGS_low)
      trims_[std::make_pair(i - 1, 0)] = std::make_pair(0, rest);
    if (FLAGS_high)
      trims_[std::make_pair(i - 1, 1)] = std::make_pair(0, rest);
  }
  curr_ = trims_.begin();
  // Now start the timer to throttle
  pub_ = nh.advertise<ff_hw_msgs::PmcCommand>(TOPIC_HARDWARE_PMC_COMMAND, 1);
  ros::Timer tc = nh.createTimer(ros::Duration(ros::Rate(FLAGS_rate)),
    CommandCallback, false, true);
  ros::Timer td = nh.createTimer(ros::Duration(FLAGS_wait),
    DelayCallback, true, true);
  sub_ = nh.subscribe<ff_hw_msgs::EpsHousekeeping>(
    TOPIC_HARDWARE_EPS_HOUSEKEEPING, 1, TelemetryCallback);
  // Block until node is shutdown
  ros::spin();
  return 0;
}
