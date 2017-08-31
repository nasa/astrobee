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


#include <ff_util/ff_names.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>

#include <iostream>

// #define ASTROBEE_P4C

int main(int argc, char **argv) {
  boost::posix_time::ptime start;
  boost::posix_time::ptime stop;

  ros::init(argc, argv, "pac_arm_emi_test_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Int16MultiArray>(TOPIC_HARDWARE_ARM_COMMAND, 1);

  std_msgs::Int16MultiArray pac_command;
  pac_command.data.resize(3);

  int target      = 0;
  int address     = 0;
  int data        = 0;

  int cmd_order   = 0;
  int cmd_delay   = 1000;

  // ros::Rate loop_rate(1.0);
#ifdef ASTROBEE_P4C
  while (ros::ok() && cmd_order < 14) {
    start = boost::posix_time::microsec_clock::local_time();

    switch (cmd_order) {
      case 0:
        target = address = data = 0;
        break;
      case 1:
        target  = 0;
        address = 1;
        data    = 0;
        cmd_delay = 6000;
        break;
      case 2:
        target  = 0;
        address = 1;
        data    = -85;
        cmd_delay = 9000;
        break;
      case 3:
        target  = 4;
        address = 51;
        data    = 0;
        cmd_delay = 4000;
        break;
      case 4:
        target  = 4;
        address = 52;
        data    = 0;
        cmd_delay = 2500;
        break;
      case 5:
        target  = 4;
        address = 53;
        data    = 0;
        cmd_delay = 2500;
        break;
      case 6:
        target  = 1;
        address = 1;
        data    = -45;
        cmd_delay = 4000;
        break;
      case 7:
        target  = 1;
        address = 1;
        data    = 90;
        cmd_delay = 12000;
        break;
      case 8:
        target  = 1;
        address = 1;
        data    = 0;
        cmd_delay = 6500;
        break;
      case 9:
        target  = 4;
        address = 52;
        data    = 0;
        cmd_delay = 2500;
        break;
      case 10:
        target  = 4;
        address = 53;
        data    = 0;
        cmd_delay = 2500;
        break;
      case 11:
        target  = 0;
        address = 1;
        data    = 0;
        cmd_delay = 4000;
        break;
      case 12:
        target = address = data = 0;
        cmd_delay = 4000;
        break;
      case 13:
        target  = -1;
        address = data = 0;
        break;
      default:
        target = address = data = 0;
        cmd_delay = 3000;
        break;
    }
#else
  while (ros::ok() && cmd_order < 12) {
    start = boost::posix_time::microsec_clock::local_time();

    switch (cmd_order) {
      case 0:
        target  = 0;
        address = 1;
        data    = 67;
        cmd_delay = 1000;
        break;
      case 1:
        target  = 0;
        address = 1;
        data    = -85;
        cmd_delay = 15000;
        break;
      case 2:
        target  = 4;
        address = 51;
        data    = 0;
        cmd_delay = 4000;
        break;
      case 3:
        target  = 4;
        address = 52;
        data    = 0;
        cmd_delay = 2500;
        break;
      case 4:
        target  = 4;
        address = 53;
        data    = 0;
        cmd_delay = 2500;
        break;
      case 5:
        target  = 1;
        address = 1;
        data    = -45;
        cmd_delay = 4000;
        break;
      case 6:
        target  = 1;
        address = 1;
        data    = 45;
        cmd_delay = 8000;
        break;
      case 7:
        target  = 1;
        address = 1;
        data    = 0;
        cmd_delay = 4000;
        break;
      case 8:
        target  = 4;
        address = 52;
        data    = 0;
        cmd_delay = 2500;
        break;
      case 9:
        target  = 4;
        address = 53;
        data    = 0;
        cmd_delay = 2500;
        break;
      case 10:
        target  = 0;
        address = 1;
        data    = 67;
        cmd_delay = 15000;
        break;
      case 11:
        target  = -1;
        address = data = 0;
        break;
      default:
        target  = 0;
        address = 1;
        data    = 67;
        cmd_delay = 3000;
        break;
    }
#endif
    cmd_order++;

    pac_command.data[0] = target;
    pac_command.data[1] = address;
    pac_command.data[2] = data;

    pub.publish(pac_command);

    ros::spinOnce();

    stop = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration dur = stop - start;
    boost::this_thread::sleep(boost::posix_time::milliseconds(cmd_delay - dur.total_milliseconds()));
  }

  ros::shutdown();
  return 0;
}

