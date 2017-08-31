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

#include <ff_msgs/FamCommand.h>
#include <i2c/i2c_new.h>

#include <boost/tokenizer.hpp>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define ROS_NODE_NAME "fam_cmd_gen"

typedef struct _Data {
  double time;
  double lin_acc_x;
  double lin_acc_y;
  double lin_acc_z;
  double ang_acc_x;
  double ang_acc_y;
  double ang_acc_z;
} Data;

void Exit(int status) {
  ROS_INFO("Shutting down the node: (status = %d)", status);
  ros::shutdown();
}

void PrintUsage(void) {
  std::cout << "Usage: fam_cmd_gen [OPTIONS]..." << std::endl;
  std::cout << "Generate FAM command" << std::endl;
  std::cout << "  -h        Show usage and help" << std::endl;
  std::cout << "  -f file   csv file" << std::endl;
  std::cout << "  -t time   start to feed acceleration from the specified time";
  std::cout << std::endl;
}

std::vector<Data> LoadData(const char *filename, float starttime) {
  std::vector<Data> data;

  std::ifstream file;

  file.open(filename);

  if (!file.is_open()) {
    std::cerr << "Failed to open file '" << filename << "': "
      << std::strerror(errno) << std::endl;

    // Return an empty vector.
    return data;
  }

  std::string line;

  double max_value = -1e10;
  double min_value = 1e10;

  while (!file.eof()) {
    std::getline(file, line);

    boost::char_separator<char> sep {","};
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);

    std::vector<double> values;
    for (const auto &t : tok) {
      double v = std::stod(t);
      values.push_back(v);
    }

    if (values.size() == 7) {
      for (size_t i = 1; i < 7; i++) {
        if (values[i] > max_value)
          max_value = values[i];
        if (values[i] < min_value)
          min_value = values[i];
      }

      Data d;

      d.time = values[0];
      d.lin_acc_x = values[1];
      d.lin_acc_y = values[2];
      d.lin_acc_z = values[3];
      d.ang_acc_x = values[4];
      d.ang_acc_y = values[5];
      d.ang_acc_z = values[6];

      if (d.time >= starttime) {
        data.push_back(d);
      }
    }
  }  // end while

  // Caculate scale for int16_t
  double scale = 32767.0 / std::max(max_value, -1.0 * min_value);

  std::cout << "Max value: " << max_value << std::endl;
  std::cout << "Min value: " << min_value << std::endl;
  std::cout << "Proposed scale for int16_t: " << scale << std::endl;

  file.close();

  return data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_NODE_NAME);
  // Private node handle
  ros::NodeHandle nh("~");

  opterr = 0;

  const char *filename = NULL;

  int c;

  float starttime = 0;

  while ((c = getopt(argc, argv, "hf:t:")) != -1) {
    switch (c) {
      case 'f':
        filename = optarg;
        std::cout << "Input file: " << filename << std::endl;
        break;

      case 't':
        starttime = atof(optarg);
        std::cout << "Start time = " << starttime << std::endl;
        break;

      default:
        PrintUsage();
        Exit(EXIT_FAILURE);
    }
  }

  std::vector<Data> data = LoadData(filename, starttime);

  std::cout << "Data size: " << data.size() << std::endl;

  if (data.size() == 0) {
    // Nothing to do.
    Exit(EXIT_SUCCESS);
  }

  ros::Publisher pub_data = nh.advertise<ff_msgs::FamCommand>("/fam", 1);

  /**
   * FIXME: This version assume 62.5 Hz fixed rate.
   */
  double control_rate = 62.5;  // Hz

  // Simulation time in seconds.
  double sim_time = 1.0 / control_rate * (data.size() - 1);

  // Data validity check.
  float offset = fabs(sim_time - data[data.size() - 1].time + starttime);
  if ( offset > 1.0 / control_rate ) {
    std::cerr << "Simulation time error" << std::endl;
    std::cerr << "offset = " << offset << std::endl;
    std::cerr << "This version assumes fixed rate: " << control_rate
      << std::endl;
    Exit(EXIT_FAILURE);
  }

  ros::Rate rate(control_rate);
  int seq = 0;

  for (size_t i = 0; i < data.size(); i++) {
    ff_msgs::FamCommand cmd;

    cmd.header.frame_id = "fam_cmd_gen";
    cmd.header.stamp = ros::Time::now();
    cmd.header.seq = seq++;

    cmd.accel.x = data[i].lin_acc_x;
    cmd.accel.y = data[i].lin_acc_y;
    cmd.accel.z = data[i].lin_acc_z;

    cmd.alpha.x = data[i].ang_acc_x;
    cmd.alpha.y = data[i].ang_acc_y;
    cmd.alpha.z = data[i].ang_acc_z;

    pub_data.publish(cmd);

    if (!ros::ok())
      break;

    rate.sleep();
  }

  Exit(EXIT_SUCCESS);
}
