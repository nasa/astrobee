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

#include <eps_driver/eps_driver.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <cerrno>
#include <cstring>

#define DEFAULT_I2C_DEV     "/dev/i2c-1"
#define DEFAULT_I2C_ADDRESS 0x40
#define DEFAULT_I2C_RETRIES 3

// Sleep function --  since this application doesn't use ROS, we don't
// have to use ROS timer to sleep. Let's use high resolution timers
void Sleep(uint32_t microseconds) {
  struct timespec req, rem;
  req.tv_sec = microseconds / 1000000;
  req.tv_nsec = (microseconds % 1000000) * 100;
  while ((req.tv_sec != 0) || (req.tv_nsec != 0)) {
    if (nanosleep(&req, &rem) == 0)
      break;
    if (errno == EINTR) {
      req.tv_sec = rem.tv_sec;
      req.tv_nsec = rem.tv_nsec;
      continue;
    }
    std::cerr << "Nanosleep terminated prematurely" << std::endl;
    break;
  }
}

// Print an error message and fail gracefully
bool Error(std::string const& msg) {
  std::cerr << "Error: " << msg << std::endl;
  return false;
}

// Grab an unsigned integer from user input with type and value checking
uint32_t InputUnsignedInteger(uint32_t min, uint32_t max) {
  uint32_t choice;
  while (true) {
    std::cout << std::endl << "Input choice > ";
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

// Grab a string
std::string InputFile() {
  std::string choice;
  while (true) {
    std::cout << std::endl << "Input choice > ";
    getline(std::cin, choice);
    std::ifstream f(choice.c_str());
    if (f.good())
      return choice;
    std::cerr << "Could not open the specified file " << choice << std::endl;
  }
  std::cerr << "Got to an unreachable section of code" << std::endl;
  return "";
}

void PrintPowerChannels(const std::vector<std::string> &channels) {
  for (size_t i = 0; i < channels.size(); i++) {
    if ((i % 8) == 0)
      std::cout << "==================================" << std::endl;

    std::cout << "channel[" << i + 1 << "]:\t" << channels[i] << std::endl;
  }
  std::cout << "==================================" << std::endl;
}

void PrintPowerChannelStates(const std::vector<std::string> &channels,
    const std::vector<eps_driver::PowerState> &states) {
  if ((states.size() != EPS_NUM_PWR_CHANS) ||
    (channels.size() != EPS_NUM_PWR_CHANS)) {
    std::cerr << "Vector size mismatch" << std::endl;
    return;
  }

  std::cout << std::endl;
  for (size_t i = 0; i < states.size(); i++) {
    if (i % 8 == 0)
      std::cout << "----------------------------" << std::endl;

    std::cout << channels[i] << ": ";
    std::cout << (states[i] == eps_driver::ENABLED ? "On" : "Off");
    std::cout << std::endl;
  }
  std::cout << "----------------------------" << std::endl;
  std::cout << std::endl;
}


// Print a main menu
bool MainMenu(eps_driver::EpsDriver &eps) {
  // Print title
  std::cout << std::endl << "Astrobee EPS host test" << std::endl << std::endl;
  // Print version and build info
  std::string version, build;
  if (!eps.GetString(eps_driver::VERSION, version))
    return Error("Could not get the version string");
  std::cout << " - FW Version " << version << std::endl;
  if (!eps.GetString(eps_driver::BUILD, build))
    return Error("Could not get the build string");
  std::cout << " - FW Build: " << build << std::endl;
  // Print the menu
  std::cout << std::endl << "Main menu:" << std::endl;
  std::cout << "0. Quit" << std::endl;
  std::cout << "1. Get version" << std::endl;
  std::cout << "2. Get build time" << std::endl;
  std::cout << "3. Get switch states" << std::endl;
  std::cout << "4. Get housekeeping" << std::endl;
  std::cout << "5. Power channel ON" << std::endl;
  std::cout << "6. Power channel OFF" << std::endl;
  std::cout << "7. Read digital (I2C) temperature sensors" << std::endl;
  std::cout << "8. Reboot" << std::endl;
  std::cout << "9. Jump to bootloader" << std::endl;
  std::cout << "10. Get serial number" << std::endl;
  std::cout << "11. Get battery status" << std::endl;
  std::cout << "12. Ring buzzer." << std::endl;
  std::cout << "13. Undock." << std::endl;
  // Keep looping until we have valid input
  uint8_t choice = InputUnsignedInteger(0, 15);
  // Do something based on the choice
  switch (choice) {
    case 0: {
      std::cout << "Goodbye." << std::endl;
      return false;
    }
    case 1: {
      std::string v;
      if (!eps.GetString(eps_driver::VERSION, v))
        return Error("Could not get the version.");
      std::cout << " - Version " << v << std::endl;
      return true;
    }
    case 2: {
      std::string b;
      if (!eps.GetString(eps_driver::BUILD, b))
        return Error("Could not get the build time.");
      std::cout << " - Build: " << b << std::endl;
      return true;
    }
    case 3: {
      std::vector<eps_driver::PowerState> states;
      if (!eps.GetAllPowerChannelState(states))
        return Error("Could not get power channel states.");
      PrintPowerChannelStates(eps.GetPowerChannelNames(), states);
      return true;
    }
    case 4: {
      std::vector < eps_driver::HousekeepingInfo > data;
      if (!eps.ReadHousekeeping(data))
        return Error("Could not read the housekeeping data");
      for (size_t i = 0; i < data.size(); i++) {
        std::cout << "- [CH" << (i + 1) << "] ";
        std::cout << data[i].description << " : ";
        std::cout << data[i].value << std::endl;
      }
      return true;
    }
    case 5: {
      PrintPowerChannels(eps.GetPowerChannelNames());
      std::cout << std::endl;
      std::cout << "Input channel number from 1 - " << eps_driver::EpsDriver::NUM_CHANNELS << std::endl;
      uint8_t channel = static_cast<uint8_t>(InputUnsignedInteger(1, eps_driver::EpsDriver::NUM_CHANNELS));
      if (!eps.SetPowerChannelState(channel - 1, eps_driver::ENABLED))
        return Error("Could not change the power state to enabled");
      std::cout << "Success!" << std::endl;
      std::cout << std::endl;
      return true;
    }
    case 6: {
      PrintPowerChannels(eps.GetPowerChannelNames());
      std::cout << std::endl;
      std::cout << "Input channel number from 1 - " << eps_driver::EpsDriver::NUM_CHANNELS << std::endl;
      uint8_t channel = static_cast<uint8_t>(InputUnsignedInteger(1, eps_driver::EpsDriver::NUM_CHANNELS));
      if (!eps.SetPowerChannelState(channel - 1, eps_driver::DISABLED))
        return Error("Could not change the power state to disabled");
      std::cout << "Success!" << std::endl;
      std::cout << std::endl;
      return true;
    }
    case 7: {
      std::vector<double> data;
      if (!eps.ReadTemperatureSensors(data))
        return Error("Could not read the digital temperature sensors");
      for (size_t i = 0; i < data.size(); i++)
        std::cout << "- [Temp " << (i + 1) << "] " << data[i] << std::endl;
      return true;
    }
    case 11: {
      std::cout << "Which battery" << std::endl;
      std::cout << "- [0] TOP_LEFT" << std::endl;
      std::cout << "- [1] BOTTOM_LEFT" << std::endl;
      std::cout << "- [2] TOP_RIGHT" << std::endl;
      std::cout << "- [3] BOTTOM_RIGHT" << std::endl;
      std::cout << "Input battery index" << std::endl;
      eps_driver::BatteryIndex battery = static_cast < eps_driver::BatteryIndex > (
        InputUnsignedInteger(0, eps_driver::NUM_BATTERIES - 1));
      eps_driver::BatteryStatus data;
      if (!eps.GetBatteryStatus(battery, data))
        return Error("Could not query the battery status");
      std::cout << "- Channel: " << static_cast<int>(data.chan) << std::endl;
      std::cout << "- Present: " << data.present << std::endl;
      std::cout << "- Voltage: " << data.voltage << " mV" << std::endl;
      std::cout << "- Current: " << data.current << " mA" << std::endl;
      std::cout << "- Remaining capacity: " << data.charge << " mAh" << std::endl;
      std::cout << "- Full capacity: " << data.capacity << " mAh" << std::endl;
      std::cout << "- Design capacity: " << data.design_capacity << " mAh" << std::endl;
      std::cout << "- Percentage: "  << data.percentage << " %" << std::endl;
      std::cout << "- Cell voltage: " << std::endl;
      for (int i = 0; i < 4; i++) {
        std::cout << "--- Cell[" << i << "]: " << data.cell_voltage[i]  << " mV" << std::endl;
      }
      std::cout << "- Status bits: 0x" << std::hex << std::uppercase << data.status
        << std::dec << std::nouppercase << std::endl;
      std::cout << "- Temperature: " << static_cast<float>(data.temperature) / 10.0f - 273.15 << " deg C" << std::endl;
      std::cout << "- Serial number: " << static_cast<unsigned int>(data.serial_number) << std::endl;
      return true;
    }
    /*
    case 11: {
      std::cout << "Which LED" << std::endl;
      std::cout << "- [0] STATUS 1" << std::endl;
      std::cout << "- [1] STATUS 2" << std::endl;
      std::cout << "- [2] STATUS 3" << std::endl;
      std::cout << "- [3] STATUS 4" << std::endl;
      std::cout << "- [4] STATUS 5" << std::endl;
      std::cout << "- [5] STATUS 6" << std::endl;
      std::cout << "- [6] STREAMING" << std::endl;
      std::cout << "- [7] CAMERA" << std::endl;
      std::cout << "- [8] MICROPHONE" << std::endl;
      eps_driver::LedIndex led = static_cast < eps_driver::LedIndex > (
        InputUnsignedInteger(0, eps_driver::NUM_LEDS - 1));
      std::cout << "Input LED state (0: persist, 1: enabled, 2: disabled)" << std::endl;
      int32_t state = InputUnsignedInteger(0, 2);
      switch (state) {
      case 1  : eps.SetLedState(led, eps_driver::ENABLED);  break;
      case 2  : eps.SetLedState(led, eps_driver::DISABLED); break;
      default : break;
      }
      std::cout << "Success!" << std::endl;
      return true;
    }
    case 12: {
      std::cout << "Which payload" << std::endl;
      std::cout << "- [0] TOP_FRONT" << std::endl;
      std::cout << "- [1] BOTTOM_FRONT" << std::endl;
      std::cout << "- [2] TOP_AFT" << std::endl;
      std::cout << "- [3] BOTTOM_AFT" << std::endl;
      eps_driver::PayloadIndex payload = static_cast < eps_driver::PayloadIndex > (
        InputUnsignedInteger(0, eps_driver::NUM_PAYLOADS - 1));
      std::cout << "Input payload power state (0: persist, 1: enabled, 2: disabled)" << std::endl;
      int32_t state = InputUnsignedInteger(0, 2);
      switch (state) {
      case 1  : eps.SetPayloadState(payload, eps_driver::ENABLED);  break;
      case 2  : eps.SetPayloadState(payload, eps_driver::DISABLED); break;
      default : break;
      }
      std::cout << "Success!" << std::endl;
      return true;
    }
    */
    case 12: {
      std::cout << "Input buzzer frequency from "
        << EPS_MIN_BUZZER_FREQUENCY << " - " << EPS_MAX_BUZZER_FREQUENCY << std::endl;
      uint16_t freq = static_cast<uint16_t>(
        InputUnsignedInteger(EPS_MIN_BUZZER_FREQUENCY, EPS_MAX_BUZZER_FREQUENCY));
      std::cout << "Input buzzer duration in seconds from "
        << EPS_MIN_BUZZER_DURATION << " - " << EPS_MAX_BUZZER_DURATION << std::endl;
      uint8_t secs = static_cast<uint8_t>(
        InputUnsignedInteger(EPS_MIN_BUZZER_DURATION, EPS_MAX_BUZZER_DURATION));
      if (!eps.RingBuzzer(freq, secs))
        return Error("Could not ring the buzzer");
      std::cout << "Success!" << std::endl;
      return true;
    }
    case 13: {
      std::cout << "Sending UNDOCK command to the dock..." << std::endl;
      if (!eps.Undock())
        return Error("Failed to send UNDOCK command to the dock");
      std::cout << "Success!" << std::endl;
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
  // Set default parameter values
  std::string devfile = DEFAULT_I2C_DEV;
  uint8_t address = DEFAULT_I2C_ADDRESS;
  uint16_t retries = DEFAULT_I2C_RETRIES;
  // Get command line parameters
  switch (argc) {
    case 4: retries = atoi(argv[3]);        // Retries
    case 3: address = atoi(argv[2]);        // Address
    case 2: devfile = argv[1];               // Device name
    case 1: break;
    default:
      std::cout << "Usage: [device] [address] [retries]" << std::endl;
      return 0;
  }
  // Try open the bus
  i2c::Error err;
  auto bus = i2c::Open(devfile, &err);
  if (!bus) {
    std::cerr << "Unable to open i2c bus ('" << devfile << "'): " << std::strerror(err) << std::endl;
    return 1;
  }
  bus->SetRetries(retries);
  // Try and contact the slave device
  auto device = bus->DeviceAt(address);
  if (!device) {
    std::cerr << "Unable to contact i2c slave ('" << address << "') on bus " << devfile << std::endl;
    return 1;
  }
  // Set or replace the eps interface and
  eps_driver::EpsDriver eps(device, std::bind(&Sleep, std::placeholders::_1));
  // Keep taking commands until
  while (MainMenu(eps)) {}
  // Success
  return 0;
}
