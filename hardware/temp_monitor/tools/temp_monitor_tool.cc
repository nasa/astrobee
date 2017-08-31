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

#include <temp_monitor/temp_monitor.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <cerrno>
#include <cstring>

// Grab an unsigned integer from user input with type and value checking
uint32_t InputUnsignedInteger(uint32_t min, uint32_t max) {
  uint32_t choice;
  while (true) {
    std::cout << "Input choice > ";
    std::string input;
    getline(std::cin, input);
    std::stringstream ss(input);
    if (ss >> choice &&  choice >= min && choice <= max)
      return choice;
    std::cerr << "Number not in range [" << min << ":" << max << "], please try again" << std::endl;
  }
  std::cerr << "Got to an unreachable section of code" << std::endl;
  return 0;
}

// Grab an unsigned integer from user input with type and value checking
uint16_t InputHexidecimalByte() {
  uint16_t choice;
  while (true) {
    std::cout << "Input choice > ";
    std::string input;
    getline(std::cin, input);
    std::stringstream ss(input);
    ss << std::hex << input;
    ss >> choice;
    return choice;
  }
  std::cerr << "Got to an unreachable section of code" << std::endl;
  return 0;
}

// Print a main menu
bool MainMenu(temp_monitor::TempMonitorPtr & sensor) {
  // Print the menu
  std::cout << "*******************************************************" << std::endl;
  std::cout << "************************* MENU ************************" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "0. Quit" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "1. Connect to temperature sensor" << std::endl;
  std::cout << "2. Query temperature" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  // Keep looping until we have valid input
  uint8_t choice = InputUnsignedInteger(0, 2);
  // Do something based on the choice
  switch (choice) {
    case 0: {
      std::cout << "Goodbye." << std::endl;
      return false;
    }
    case 1: {
      // Grab the i2c device number
      std::cout << "Input i2c device number" << std::endl;
      uint8_t dev_num = static_cast < uint8_t >(InputUnsignedInteger(0, 4));

      // Grab the device address
      std::cout << "Input num retries" << std::endl;
      uint32_t retries = InputUnsignedInteger(0, 512);

      // Grab the device address
      std::cout << "Select slave" << std::endl;
      std::cout << "- [0] Backplane (0x48)" << std::endl;
      std::cout << "- [1] LLP (0x49)" << std::endl;
      std::cout << "- [2] MLP (0x4A)" << std::endl;
      std::cout << "- [3] HLP (0x4B)" << std::endl;
      uint32_t address_id = InputUnsignedInteger(0, 3);
      uint16_t address = 0x48;
      std::string type = "ADT7410";
      switch (address_id) {
      default:
      case 0: address = 0x48; type = "ADT7410"; break;
      case 1: address = 0x49; type = "TCN75A";  break;
      case 2: address = 0x4A; type = "TCN75A";  break;
      case 3: address = 0x4B; type = "TCN75A";  break;
      }

      // Try and open the bus
      i2c::Error err;
      auto bus = i2c::Open("/dev/i2c-" + std::to_string(dev_num), &err);
      if (!bus) {
        std::cerr << "Could not open the i2c bus" << std::endl;
        return true;
      }
      bus->SetRetries(retries);
      // Try and contact the slave device
      auto device = bus->DeviceAt(address);
      if (!device) {
        std::cerr << "Could not find the i2c slave address" << std::endl;
        return true;
      }

      // Check if we already have this bus open
      sensor = temp_monitor::TempMonitorFactory::Create(type, device);
      if (!sensor) {
        std::cerr << "Could not instantiate the device" << std::endl;
        return true;
      }

      return true;
    }
    case 2: {
      double temp;
      if (sensor == nullptr)
        std::cerr << "Please connect to a temperature sensor" <<  std::endl;
      else if (sensor->GetTemperature(&temp))
        std::cout << "Temperature: " << temp << " celcius" << std::endl;
      else
        std::cerr << "No response from temperature sensor" <<  std::endl;
      return true;
    }
    default: {
      std::cerr << "Invalid selection" << std::endl;
      return true;
    }
  }
  return true;
}

// Main entry point for application
int main(int argc, char *argv[]) {
  std::cout << std::endl << "Astrobee temperature monitor host test" << std::endl << std::endl;
  temp_monitor::TempMonitorPtr sensor = temp_monitor::TempMonitorPtr();
  while (MainMenu(sensor)) {}
  return 0;
}
