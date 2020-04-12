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

// Basic LUA config
#include <config_reader/config_reader.h>

// C++ includes
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <csignal>

// Implementation
#include "smart_dock/smart_dock_node.h"

// Make sure we exit gracefully
bool running_ = true;

// Don't allow the thread and comms loop concurrent access
std::mutex mutex_;

// CTRL + C stops execution
void InterruptHandler(int dummy) {
  running_ = false;
}

// Poll the smart dock node periodically to flush message queue
void CommsLoop(smart_dock::SmartDockNode *sdn, unsigned int targ) {
  std::chrono::milliseconds dt(targ);
  while (running_) {
    auto st = std::chrono::high_resolution_clock::now();
    mutex_.lock();
    sdn->ProcessComms();
    mutex_.unlock();
    auto et = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(dt - (et - st));
  }
  std::cout << "Killing comms thread." << std::endl;
}

// Poll the smart dock node periodically to check for dock state changes
void TelemLoop(smart_dock::SmartDockNode *sdn, unsigned int targ) {
  std::chrono::milliseconds dt(targ);
  while (running_) {
    auto st = std::chrono::high_resolution_clock::now();
    mutex_.lock();
    sdn->ProcessTelem();
    mutex_.unlock();
    auto et = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(dt - (et - st));
  }
  std::cout << "Killing telemetry thread." << std::endl;
}

// Makes code flow simpler
int Message(int code, const char * msg) {
  if (code > 1)
    std::cerr << msg << std::endl;
  else
    std::cout << msg << std::endl;
  return code;
}

int main(int argc, char** argv) {
  // Read the configuration
  config_reader::ConfigReader config;
  config.AddFile("hw/smart_dock.config");
  if (!config.ReadFiles())
    return Message(-4, "Cannot read the smart dock config file");

  // Get i2c info
  std::string i2c_bus_dev;
  if (!config.GetStr("i2c_bus_dev", &i2c_bus_dev))
    return Message(-1, "Couldn't find device in LUA config");
  uint32_t i2c_address;
  if (!config.GetUInt("i2c_address", &i2c_address))
    return Message(-2, "couldn't find address in LUA config");
  uint32_t i2c_retries;
  if (!config.GetUInt("i2c_retries", &i2c_retries))
    return Message(-3, "EEPS: couldn't find i2c retry count in LUA config");

  // Try open the i2c device
  i2c::Error err;
  auto bus = i2c::Open(i2c_bus_dev, &err);
  if (!bus)
    return Message(-4, "Unable to open i2c bus");
  bus->SetRetries(i2c_retries);
  auto device = bus->DeviceAt(i2c_address);
  if (!device)
    return Message(-5, "Unable to open i2c slave");

  // Create the smart dock node
  smart_dock::SmartDockNode sdn(argc, argv, device, "RapidCommand");

  // Add the devices
  config_reader::ConfigReader::Table devices;
  if (!config.GetTable("devices", &devices))
    return Message(-7, "Cannot read the device table");
  for (int i = 0; i < devices.GetSize(); i++) {
    config_reader::ConfigReader::Table info;
    if (!devices.GetTable(i + 1, &info))
      return Message(-8, "Cannot read the device table");
    std::string serial;
    if (!info.GetStr("serial", &serial))
      return Message(-9, "Cannot read the serial number of a device");
    std::string name;
    if (!info.GetStr("name", &name))
      return Message(-10, "Cannot read the name of a device");
    // Register this device with the smart dock node
    if (!sdn.AddDevice(serial, name))
      return Message(-11, "Duplicate serial number in devicetable");
  }

  // Get polling info
  unsigned int comms_poll_dt;
  if (!config.GetUInt("comms_poll_dt", &comms_poll_dt))
    return Message(-12, "Could not read the communication poll time");
  unsigned int telem_poll_dt;
  if (!config.GetUInt("telem_poll_dt", &telem_poll_dt))
    return Message(-13, "Could not read the telemetry poll time");

  // Install a signal handler to ensure threads die safely
  signal(SIGINT, InterruptHandler);

  // Create two threads to do the work in this node
  std::thread thread_c(CommsLoop, &sdn, comms_poll_dt);
  std::thread thread_t(TelemLoop, &sdn, telem_poll_dt);

  // Block until threads return naturally
  thread_t.join();
  thread_c.join();

  // Success
  std::cout << "Exiting." << std::endl;
  return 0;
}
