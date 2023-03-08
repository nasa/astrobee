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

#include <ros/ros.h>

#include <ff_hw_msgs/PmcCommand.h>
#include <ff_common/ff_names.h>

#include <stdlib.h>

#include <cstdint>
#include <cstring>

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#define ROS_NODE_NAME "pmc_actuator_cmd_test"

// Number of PMCs to control
int num_pmcs_ = 2;

// ROS topics
std::string topic_command_ = TOPIC_HARDWARE_PMC_COMMAND;

// Control rate in Hz.
double kControlRateHz = 62.5;

bool kLooping = false;

// nozzle fully closed cmd
uint8_t nozzle_min_cmd_ = 0;

// nozzle fully open cmd
uint8_t nozzle_max_cmd_ = 255;

// blower max speed
uint8_t blower_max_cmd_ = 232;

std::vector<ff_hw_msgs::PmcCommand> pmc_commands_;

void usage() {
  std::cerr << "Usage:" << std::endl;
  std::cerr << "       pmc_actuator_feeder input_file [frequency]" << std::endl;
  std::cerr << "           frequency=0 --> loop the sequence" << std::endl;
}

size_t parse_inputs(const char *filename) {
  std::ifstream input(filename);
  std::string line;
  int counter = 0;

  while (std::getline(input, line)) {
    counter++;
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\0') {
      // skip comments
      continue;
    }
    std::stringstream str(line);
    float value;
    std::vector<double> numbers;
    while (str >> value) {
      numbers.push_back(value);
    }
    if (numbers.size() != 15) {
      std::cerr << "Malformatted input at line " << counter << std::endl;
    } else {
      ff_hw_msgs::PmcCommand cmd;
      ros::Time time(numbers.at(0));
      cmd.header.stamp = time;
      for (int p = 0; p < num_pmcs_; p++) {
        ff_hw_msgs::PmcGoal goal;
        uint8_t speed = (uint8_t)numbers.at(1 + p * 7);
        goal.motor_speed = (speed > blower_max_cmd_) ? blower_max_cmd_ : speed;
        for (int n = 0; n < 6; n++) {
          uint8_t pos = (uint8_t)numbers.at(2 + n + p * 7);
          goal.nozzle_positions[n] =
              (pos < nozzle_min_cmd_)
                  ? nozzle_min_cmd_
                  : (pos > nozzle_max_cmd_) ? nozzle_max_cmd_ : pos;
        }
        cmd.goals.push_back(goal);
      }
      pmc_commands_.push_back(cmd);
    }
  }
  return pmc_commands_.size();
}

ros::Publisher cmd_pub_;

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_NODE_NAME);

  if (argc < 2) {
    usage();
    return -1;
  }
  if (argc > 2) {
    if (atoi(argv[2]) == 0) {
      kLooping = true;
    } else {
      kControlRateHz = atof(argv[2]);
    }
  }

  int lines = parse_inputs(argv[1]);
  if (lines < 1) {
    std::cerr << "no data to run!" << std::endl;
    return -1;
  }

  ros::NodeHandle nh;
  std::string topic_cmd_name = topic_command_;
  if (nh.getNamespace().length() > 1) {
    topic_cmd_name = nh.getNamespace().substr(1) + topic_command_;
  }

  cmd_pub_ = nh.advertise<ff_hw_msgs::PmcCommand>(topic_cmd_name, 4);

  ros::Rate rate(kControlRateHz);

  size_t index = 0;
  bool last_command = false;
  std::cout << pmc_commands_.at(index);
  double start_time = ros::Time::now().toSec();

  while (ros::ok()) {
    cmd_pub_.publish(pmc_commands_.at(index));

    double current_time = ros::Time::now().toSec();
    if (!last_command) {
      double next_time = pmc_commands_.at(index + 1).header.stamp.toSec();
      double elapsed_time = current_time - start_time;
      // std::cout << "next_time=" << next_time
      //           << " / elapsed_time=" << elapsed_time << std::endl;
      if (elapsed_time > next_time) {
        index++;
        std::cout << pmc_commands_.at(index);
        if (index >= pmc_commands_.size() - 1) {
          if (kLooping) {
            index = 0;
            start_time = ros::Time::now().toSec();
          } else {
            last_command = true;
          }
        }
      }
      pmc_commands_.at(index).header.seq++;
    }

    rate.sleep();
  }

  return 0;
}
