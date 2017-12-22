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

#include <ff_util/ff_names.h>
#include <ff_hw_msgs/PmcCommand.h>

#include <math.h>

#include <cerrno>
#include <cstring>

#define ROS_NODE_NAME "pmc_actuator_cmd_test"

// Number of PMCs to control
int num_pmcs_ = 2;

// Nozzle ID to control
// -1: control all nozzles
// [0..5]: valid nozzle IDs
int nozzle_id_ = -1;

// ROS topics
std::string topic_command_ = TOPIC_HARDWARE_PMC_COMMAND;

// Control rate in Hz.
double control_rate_hz_ = 62.5;

// nozzle fully closed cmd
uint8_t nozzle_min_cmd_ = 25;

// nozzle fully open cmd
uint8_t nozzle_max_cmd_ = 90;

// blower max speed
uint8_t blower_max_cmd_ = 249;

// #define DEBUG_MODE 1

class CmdGenerator {
 public:
  CmdGenerator(uint8_t bs1, uint8_t bs2,
               float np1, float np2,
               float ns1, float ns2,
               float time2start) {
    blower_speeds_[0] = bs1;
    blower_speeds_[1] = bs2;
    nozzle_positions_[0] = static_cast<float>(np1);
    nozzle_positions_[1] = static_cast<float>(np2);
    nozzle_speed_cur_[0] = ns1;
    nozzle_speed_cur_[1] = ns2;
    nozzle_speed_abs_[0] = fabs(ns1);
    nozzle_speed_abs_[1] = fabs(ns2);
    time2start_ = time2start;
    ros::Time::init();
    start_time_ = ros::Time::now().toSec();
  }

  void NextCommand(ff_hw_msgs::PmcCommand& command) {
    static int counter = 0;

    for (int i = 0; i < num_pmcs_; i++) {
      ff_hw_msgs::PmcGoal goal;
      goal.motor_speed = blower_speeds_[i];

      for (int n = 0; n < 6; n++) {
        if (nozzle_id_ == -1 || n == nozzle_id_) {
          goal.nozzle_positions[n]
            = (uint8_t)round(nozzle_positions_[i]);
        } else {
          goal.nozzle_positions[n] = nozzle_min_cmd_;
        }
      }

      command.goals.push_back(goal);

      if ( ros::Time::now().toSec() - start_time_ > time2start_ ) {
        if ((uint8_t)roundf(nozzle_positions_[i]) >= nozzle_max_cmd_) {
          nozzle_speed_cur_[i] = -nozzle_speed_abs_[i];
        }

        if ((uint8_t)roundf(nozzle_positions_[i]) <= nozzle_min_cmd_) {
          nozzle_speed_cur_[i] = nozzle_speed_abs_[i];
        }
        nozzle_positions_[i] += nozzle_speed_cur_[i];
      }
    }
    counter++;
  }

 private:
  uint8_t blower_speeds_[2];
  float nozzle_positions_[2];
  float nozzle_speed_cur_[2];
  float nozzle_speed_abs_[2];
  float time2start_;
  double start_time_;
};

ros::Publisher cmd_pub_;

void PrintUsage(void) {
  std::cout << "Usage: pmc_actuator_cmd_test [OPTIONS]..." << std::endl;
  std::cout << "Generate FAM command" << std::endl;
  std::cout << "  -h        Show usage and help" << std::endl;
  std::cout << "  -b speed  Blower speed [0.."
            << static_cast<int>(blower_max_cmd_)  << "]"
            << std::endl;
  std::cout << "  -n nozzle Nozzle ID [0..5], -1 for all." << std::endl;
  std::cout << std::endl;
}

int main(int argc, char **argv) {
  uint8_t blower_speed[2] = { 207, 207 };
  float nozzle_start[2] = { 25, 25 };
  float nozzle_speed[2] = { 1, 1 };
  double time2start = 6;

	/*
  if ( argc > 2 ) {
    blower_speed[0] = atoi(argv[1]);
    blower_speed[1] = atoi(argv[2]);
  }
  if ( argc > 4 ) {
    nozzle_start[0] = atof(argv[3]);
    nozzle_start[1] = atof(argv[4]);
  }
  if ( argc > 6 ) {
    nozzle_speed[0] = atof(argv[5]);
    nozzle_speed[1] = atof(argv[6]);
  }
  if ( argc > 7 ) {
    time2start = atof(argv[7]);
  }
	*/

  int c;

  while ((c = getopt(argc, argv, "hb:n:")) != -1) {
    switch (c) {
      case 'b': {
          int speed = atoi(optarg);
          std::cout << "Blower speed: " << speed << std::endl;

          if (speed < 0 || speed > static_cast<int>(blower_max_cmd_)) {
            std::cout << "Invalid speed command: " << speed << std::endl;
            return -1;
          }

          blower_speed[0] = static_cast<uint8_t>(speed & 0x00FF);
          blower_speed[1] = static_cast<uint8_t>(speed & 0x00FF);
        }

      break;

      case 'n': {
          int id = atoi(optarg);
          std::cout << "Nozzle ID: " << id << std::endl;

          if (id == -1) {
            std::cout << "Control all nozzles" << std::endl;
          } else if (id < 0 || id > 5) {
            std::cout << "Invalid nozzle ID: " << id << std::endl;
            return -1;
          }

          nozzle_id_ = id;
        }

      break;

      default:
       PrintUsage();
       return -1;
    }
  }


  CmdGenerator gen(
    blower_speed[0], blower_speed[1],
    nozzle_start[0], nozzle_start[1],
    nozzle_speed[0], nozzle_speed[1],
    time2start);

#ifdef DEBUG_MODE
  // test the generator only
  for (int k = 0; k < 600; k++) {
    ff_hw_msgs::PmcCommand command;
    gen.NextCommand(command);
    printf("bs: %d %d np: %d %d\n",
           command.goals[0].speed,
           command.goals[1].speed,
           command.goals[0].nozzle_positions[0],
           command.goals[1].nozzle_positions[1]);
  }
  return 1;
#endif

  ros::init(argc, argv, ROS_NODE_NAME);
  ros::NodeHandle nh;
  std::string topic_cmd_name = topic_command_;
  if (nh.getNamespace().length() > 1) {
    topic_cmd_name = nh.getNamespace().substr(1) + topic_command_;
  }

  cmd_pub_ = nh.advertise<ff_hw_msgs::PmcCommand>(
    topic_cmd_name, 4);

  ros::Rate rate(control_rate_hz_);

  while (ros::ok()) {
    ff_hw_msgs::PmcCommand command;

    gen.NextCommand(command);
    cmd_pub_.publish(command);

    rate.sleep();
  }

  return 0;
}
