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

#include <std_msgs/Int16MultiArray.h>

#include <iostream>

// #define ASTROBEE_P4C

int main(int argc, char **argv) {
  ros::init(argc, argv, "pac_arm_test_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Int16MultiArray>(TOPIC_HARDWARE_ARM_COMMAND, 1);

  std_msgs::Int16MultiArray pac_command;
  pac_command.data.resize(3);

#ifdef ASTROBEE_P4C
  const int jointNum = 2;
  int selection = 0;
#endif
  int target    = 0;
  int address   = 0;
  int value     = 0;

  ros::Rate loop_rate(1.0);

  while (ros::ok() && (target != -1) && (target != 100)) {
#ifdef ASTROBEE_P4C
    std::cout << "\n***** Enter Selection *****" << std::endl;
    std::cout << "*** Arm ***" << std::endl;
    std::cout << "Enter Joint (from 0 to " << jointNum-1 << ") to Set Position and Velocity" << std::endl;
    std::cout << "2: Set Torque Limit" << std::endl;
    std::cout << "3: Enable/Disable Torque" << std::endl;
    std::cout << "4: Set Register Manually" << std::endl;
    std::cout << "*** Gripper ***" << std::endl;
    std::cout << "51: Calibration" << std::endl;
    std::cout << "52: Open" << std::endl;
    std::cout << "53: Close" << std::endl;
    std::cout << "54: Position" << std::endl;
    std::cout << "55: Reset Zero Position" << std::endl;
    std::cout << "60: Set DTR Low" << std::endl;
    std::cout << "61: Set DTR High (default)" << std::endl;
    std::cout << "100: Reset Board" << std::endl;
    std::cout << "***** Enter '-1' to Quit *****" << std::endl;

    std::cin >> target;
    address   = 0;
    selection = 0;
    value     = 0;

    switch (target) {
      case -1:
        std::cout << "\nTerminate Process!! Good Bye!!" << std::endl;
        break;
      case 0:
      case 1:
        std::cout << "\n0. Set to Initial Config" << std::endl;
        std::cout << "1. Set Position" << std::endl;
        std::cout << "2. Set Velocity" << std::endl;
        std::cin >> address;
        switch (address) {
          case 0:
            break;
          case 1:
            std::cout << "\nProvide the goal position [deg] (from -150 to 150)" << std::endl;
            std::cin >> value;
            if ((value < -150) || (value > 150)) {
              address = 255;
              std::cout << "Wrong position input!!" << std::endl;
            }
            break;
          case 2:
            std::cout << "\nProvide the goal velocity "
                      << "[1 = 0.111 rpm, 0 = MAX RPM, default = 50] "
                      << "(from 0 to 1023)"
                      << std::endl;
            std::cin >> value;
            if ((value < 0) || (value > 1023)) {
              address = 255;
              std::cout << "Wrong velocity input!!" << std::endl;
            }
            break;
          default:
            break;
        }
        break;
      case 2:
        std::cout << "\nProvide the torque limit [%] (from 0 to 100)" << std::endl;
        std::cin >> value;
        if ((value >= 0) && (value <= 100)) {
          target = 3;
          address = 34;
          value = static_cast<int>(value/100.0*1023.0);
        } else {
          address = 255;
          std::cout << "Wrong torque limit input!!" << std::endl;
        }
        break;
      case 3:
        std::cout << "\nProvide 0 to 'disable' or 1 to 'enable'" << std::endl;
        std::cin >> value;
        if ((value == 0) || (value == 1)) {
          target = 2;
          address = 24;
        } else {
          address = 255;
          std::cout << "Wrong input!!" << std::endl;
        }
        break;
      case 4:
        std::cout << "\nProvide 2 to set 'one' register or 3 to set 'two' registers" << std::endl;
        std::cin >> selection;
        std::cout << "\nAddress location" << std::endl;
        std::cin >> address;
        std::cout << "\nValue" << std::endl;
        std::cin >> value;

        if (((selection == 2) || (selection == 3)) && (address >= 3) && (address <= 48)) {
          target = selection;
        } else {
          address = 255;
          std::cout << "Wrong input!!" << std::endl;
        }
        break;
      case 51:
      case 52:
      case 53:
      case 55:
        address = target;
        target = 4;
        break;
      case 54:
        std::cout << "\nProvide encoder value (1200 ticks per 1 rotation)" << std::endl;
        std::cin >> value;
        address = target;
        target = 4;
        break;
      case 60:
      case 61:
        address = target;
        break;
      case 100:
        std::cout << "Reset Board" << std::endl;
        address = target;
        target  = 100;
        break;
      default:
        address = 255;
        std::cout << "Wrong input!!" << std::endl;
        break;
    }
#else
    std::cout << "\n***** Enter Target (-1 to QUIT) *****" << std::endl;
    std::cout << "0: Set Tilt Joint" << std::endl;
    std::cout << "1: Set Pan Joint" << std::endl;
    std::cout << "2: Set Two Joints" << std::endl;
    std::cout << "4: Set Gripper" << std::endl;
    std::cout << "10: Reset Controller Board (FTDI)" << std::endl;
    std::cout << "100: Reset Controller Board (SW)" << std::endl;
    std::cout << "*************************************" << std::endl;

    std::cin >> target;
    address   = 0;
    value     = 0;

    if (target == -1) {
      std::cout << "\n***** Terminating Test Program! ... Good Bye! *****" << std::endl;
    } else if (target == 0 || target == 1) {
      std::cout << "\n0. Set to Initial Configuration" << std::endl;
      std::cout << "1. Set Position" << std::endl;
      std::cout << "2. Set Velocity" << std::endl;
      std::cout << "-- Set Address Manually (refer to XM430 W210 datasheet)" << std::endl;
      std::cin >> address;
      switch (address) {
        case 0:
          break;
        case 1:
          std::cout << "\nEnter Goal Position [deg] (from -150 to 150)" << std::endl;
          std::cin >> value;
          if ((value < -150) || (value > 150)) {
            address = 255;
            std::cout << "Invalid Value!" << std::endl;
          }
          break;
        case 2:
          std::cout << "\nEnter Goal Velocity "
                      << "[1 = 0.229 rpm, 0 = MAX RPM, default = 10] "
                      << "(from 0 to 1223)"
                      << std::endl;
          std::cin >> value;
          if ((value < 0) || (value > 1223)) {
            address = 255;
            std::cout << "Invalid Value!" << std::endl;
          }
          break;
        case 7:
        case 8:
        case 9:
        case 11:
        case 13:
        case 20:
        case 24:
        case 31:
        case 32:
        case 34:
        case 36:
        case 38:
        case 40:
        case 44:
        case 48:
        case 52:
        case 63:
        case 64:
        case 65:
        case 68:
        case 76:
        case 78:
        case 80:
        case 82:
        case 84:
        case 88:
        case 90:
        case 100:
        case 102:
        case 104:
        case 108:
        case 112:
        case 116:
        case 168:  // indirect address 1
        case 170:  // indirect address 2
        case 172:  // indirect address 3
        case 174:  // indirect address 4
        case 176:  // indirect address 5
        case 178:  // indirect address 6
        case 180:  // indirect address 7
        case 182:  // indirect address 8
        case 184:  // indirect address 9
        case 186:  // indirect address 10
        case 188:  // indirect address 11
        case 190:  // indirect address 12
        case 192:  // indirect address 13
        case 194:  // indirect address 14
        case 196:  // indirect address 15
        case 198:  // indirect address 16
        case 200:  // indirect address 17
        case 202:  // indirect address 18
        case 204:  // indirect address 19
        case 206:  // indirect address 20
        case 208:  // indirect address 21
        case 210:  // indirect address 22
        case 212:  // indirect address 23
        case 214:  // indirect address 24
        case 216:  // indirect address 25
        case 218:  // indirect address 26
        case 220:  // indirect address 27
        case 222:  // indirect address 28
          std::cout << "\n*** WARNING :: ENTER -1 NOT TO CHANGE ***" << std::endl;
          std::cout << "Enter Value" << std::endl;
          std::cin >> value;
          if (value == -1)
            address = 255;
          break;
        default:
          std::cout << "\nInvalid Command!" << std::endl;
          address = 255;
          break;
      }
    } else if (target == 2) {
      std::cout << "Set Address Manually (refer to XM430 W210 datasheet)" << std::endl;
      std::cin >> address;
      switch (address) {
        case 8:
        case 9:
        case 11:
        case 13:
        case 20:
        case 24:
        case 31:
        case 32:
        case 34:
        case 36:
        case 38:
        case 40:
        case 44:
        case 48:
        case 52:
        case 63:
        case 64:
        case 65:
        case 68:
        case 76:
        case 78:
        case 80:
        case 82:
        case 84:
        case 88:
        case 90:
        case 100:
        case 102:
        case 104:
        case 108:
        case 112:
        case 116:
        case 168:  // indirect address 1
        case 170:  // indirect address 2
        case 172:  // indirect address 3
        case 174:  // indirect address 4
        case 176:  // indirect address 5
        case 178:  // indirect address 6
        case 180:  // indirect address 7
        case 182:  // indirect address 8
        case 184:  // indirect address 9
        case 186:  // indirect address 10
        case 188:  // indirect address 11
        case 190:  // indirect address 12
        case 192:  // indirect address 13
        case 194:  // indirect address 14
        case 196:  // indirect address 15
        case 198:  // indirect address 16
        case 200:  // indirect address 17
        case 202:  // indirect address 18
        case 204:  // indirect address 19
        case 206:  // indirect address 20
        case 208:  // indirect address 21
        case 210:  // indirect address 22
        case 212:  // indirect address 23
        case 214:  // indirect address 24
        case 216:  // indirect address 25
        case 218:  // indirect address 26
        case 220:  // indirect address 27
        case 222:  // indirect address 28
          std::cout << "\n*** WARNING :: ENTER -1 NOT TO CHANGE ***" << std::endl;
          std::cout << "Enter Value" << std::endl;
          std::cin >> value;
          if (value == -1)
            address = 255;
          break;
        default:
          std::cout << "\nInvalid Command!" << std::endl;
          address = 255;
          break;
      }
    } else if (target == 4) {
      std::cout << "51: Calibration" << std::endl;
      std::cout << "52: Open" << std::endl;
      std::cout << "53: Close" << std::endl;
      std::cout << "54: Set Position" << std::endl;
      std::cout << "55: Reset Zero Position" << std::endl;
      std::cout << "100: Disable Motor Driver" << std::endl;
      std::cout << "101: Enable Motor Driver" << std::endl;
      std::cin >> address;
      switch (address) {
        case 51:
        case 52:
        case 53:
        case 55:
          break;
        case 54:
          std::cout << "\nEnter Encoder Value (3576 ticks per 1 rotation)" << std::endl;
          std::cin >> value;
          break;
        case 100:
        case 101:
          break;
        default:
          std::cout << "\nInvalid Command!" << std::endl;
          address = 255;
          break;
      }
    } else if (target == 100) {
      std::cout << "***** Reset Board! *****" << std::endl;
    }
#endif

    pac_command.data[0] = target;
    pac_command.data[1] = address;
    pac_command.data[2] = value;

    pub.publish(pac_command);

    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}

