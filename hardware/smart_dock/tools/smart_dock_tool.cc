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

// Proxy library
#include <smart_dock/smart_dock.h>

// C++ includes
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

// Gflag defaults
DEFINE_string(device, "/dev/i2c-2", "i2c bus of smart dock");
DEFINE_int32(address, 0x42, "i2c address of smart dock");
DEFINE_int32(retries, 3, "i2c retries");

// Single-switch options
DEFINE_bool(state, false, "Get the state of the dock");
DEFINE_bool(reboot, false, "Reboot");
DEFINE_bool(hk, false, "View housekeeping information");
DEFINE_bool(string, false, "View string information");
DEFINE_bool(fault, false, "View and clear faults");
DEFINE_bool(power, false, "View and toggle power channels");
DEFINE_bool(berth, false, "View berth information");
DEFINE_bool(command, false, "Send a command to a berth");
DEFINE_bool(led, false, "Configure LEDs");

// Second-level options for VIEW [read-only multiple options]
DEFINE_string(set, "", "Set a new value for all specified indexes");
DEFINE_bool(get, false, "Get the current value for all specified indexes");
DEFINE_bool(list, false, "List all indexes and keys for the given command");
DEFINE_bool(clear, false, "Send a clear command (only for -fault)");

// What we'll use to index various channels, payloads, batteries, etc.
typedef std::vector<std::string> Keywords;
typedef std::pair<std::string, Keywords> Value;
typedef std::map<uint32_t, Value> ValueMap;

// Avoid long constants
using SD = smart_dock::SmartDock;
using EPS = eps_driver::EPS;

// Helper function for displaying an error
int Error(std::string const& msg,  // Message
          int code = -1) {         // Error code
  std::cerr << msg << std::endl;
  return code;
}

// Helper function for displaying metadata
int Print(std::string const& msg) {
  std::cout << msg << std::endl;
  return 0;
}

// Helper function to print a channel list
int Print(std::string const& title, ValueMap const& values, bool ro = false) {
  if (!title.empty()) {
    std::cout << title;
    if (ro) std::cout << " (READ ONLY)";
    std::cout << std::endl;
  }
  // Print out the indexes
  if (!values.empty()) {
    ValueMap::const_iterator it;
    for (it = values.begin(); it != values.end(); it++) {
      std::cout << "- " << it->second.first << " (";
      Keywords::const_iterator jt = it->second.second.begin();
      for (; jt != it->second.second.end(); jt++) {
        if (jt != it->second.second.begin()) std::cout << ", ";
        std::cout << *jt;
      }
      std::cout << ")" << std::endl;
    }
  }
  return 0;
}

// View operation
int HelpView(std::string const& flag, std::string const& desc, bool i = true) {
  std::string extra = (i ? "<index>" : "");
  std::cout << "DESCRIPTION" << std::endl
            << " " << desc << std::endl
            << std::endl
            << "USAGE OVERVIEW" << std::endl
            << " The -" << flag << " flag requires either -list or -get to"
            << std::endl
            << " also be specified. You can use  -list to see all available"
            << std::endl
            << " indexes and values, and -get to query the current values."
            << std::endl
            << std::endl
            << "USAGE PATTERN" << std::endl
            << " smart_dock_tool -" << flag << " [ -list | -get ] " << extra
            << std::endl;
  return 0;
}

// Configure operation
int HelpConf(std::string const& flag, std::string const& desc, bool i = true) {
  std::string extra = (i ? "<index>" : "");
  std::cout << "DESCRIPTION" << std::endl
            << " " << desc << std::endl
            << std::endl
            << "USAGE OVERVIEW" << std::endl
            << " The -" << flag << " flag is a requires one of -list, -get"
            << std::endl
            << " or -set to also be specified. You can use -list to see all"
            << std::endl
            << " available indexes and values, -get to query the current"
            << std::endl
            << " value and -set to set the value." << std::endl
            << std::endl
            << "USAGE PATTERN" << std::endl
            << " smart_dock_tool -" << flag << " [ -list | -get | -set <val> ]"
            << extra << std::endl;
  return 0;
}

// Help buzzer
int Help() {
  std::cout << "DESCRIPTION" << std::endl
            << " Command-line tool for interacting with the smart dock"
            << std::endl
            << std::endl
            << "SUBSYSTEM CONTROL" << std::endl
            << " The tool supports the following subsystem flags" << std::endl
            << "  -power    : Turn on an off power to dock channels" << std::endl
            << "  -berth    : View information about a given berth" << std::endl
            << "  -command  : Send a command to a given berth" << std::endl
            << "  -hk       : View dock housekeeping information" << std::endl
            << "  -fault    : View and clear dock fault information"
            << std::endl
            << "  -string   : View dock build info, software and serial"
            << std::endl
            << " For more help about a specific subsystem, add the flag"
            << std::endl
            << "  eg. smart_dock_tool -power" << std::endl
            << std::endl
            << "ONE-SHOT COMMANDS" << std::endl
            << " The tool supports the following oneshot commands" << std::endl
            << "  -reboot       : Reboot the smart dock" << std::endl
            << "  -state        : Get the state of the smart dock" << std::endl
            << " To use the command, just add the flag" << std::endl
            << "  eg. smart_dock_tool -reboot";
  std::cout << std::endl;
  return 0;
}

// Get a mask from an index of strings
bool Mask(ValueMap const& values, std::vector<std::string> const& idxs,
          uint32_t& mask, bool allow_all_to_be_selected = true) {
  bool clean = true;
  mask = 0x0;
  if (idxs.empty()) {
    if (allow_all_to_be_selected) {
      ValueMap::const_iterator jt;
      for (jt = values.begin(); jt != values.end(); jt++)
        mask |= (1 << jt->first);
    } else {
      clean = false;
    }
  } else {
    std::vector<std::string>::const_iterator it;
    for (it = idxs.begin(); it != idxs.end(); it++) {
      bool found = false;
      ValueMap::const_iterator jt;
      for (jt = values.begin(); jt != values.end(); jt++) {
        if (std::find(jt->second.second.begin(), jt->second.second.end(),
                      *it) == jt->second.second.end())
          continue;
        mask |= (1 << jt->first);
        found = true;
      }
      if (!found) {
        std::cerr << "Warning: index '" << *it << "' not found" << std::endl;
        clean = false;
      }
    }
  }
  return clean;
}

// Get a value from a value map
bool Valid(ValueMap const& values, std::string const& input, uint32_t& val) {
  ValueMap::const_iterator jt;
  for (jt = values.begin(); jt != values.end(); jt++) {
    if (std::find(jt->second.second.begin(), jt->second.second.end(), input) ==
        jt->second.second.end())
      continue;
    val = jt->first;
    return true;
  }
  return false;
}

// Main entry point for application
int main(int argc, char** argv) {
  // Set up gflags
  google::SetUsageMessage("Usage: smart_dock_tool <options> [value]\n");
  google::SetVersionString("0.3.0");
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  // Get the channel list, if exists
  std::vector<std::string> input;
  for (int i = 1; i < argc; i++) input.push_back(argv[i]);

  // Try open the i2c device
  i2c::Error err;
  auto bus = i2c::Open(FLAGS_device, &err);
  if (!bus) return Error("Unable to open i2c bus", -1);
  bus->SetRetries(FLAGS_retries);
  auto device = bus->DeviceAt(FLAGS_address);
  if (!device) return Error("Unable to open i2c slave", -2);

  // Attach an smart dock to the device on the bus
  SD sd(device);

  // REBOOT
  if (FLAGS_reboot) {
    if (!sd.Reboot()) return Error("Failed to send a REBOOT command");
    return Print("Sent a REBOOT command successfully");
  }

  // REBOOT
  if (FLAGS_state) {
    SD::DockState state;
    if (!sd.GetSystemState(state))
      return Error("Failed to get the system state");
    std::cout << "Current dock state:" << std::endl;
    std::cout << "- Serial: 0x";
    for (int i = 5; i >= 0; i--) {
      std::cout << std::setfill('0') << std::setw(2) << std::hex
                << static_cast<uint32_t>(state.serial_number[i]);
    }
    std::cout << std::endl;
    std::cout << "- Subsys 1 state: 0x" << std::setfill('0') << std::setw(2)
              << std::hex << static_cast<uint32_t>(state.subsys_1_pwr)
              << std::endl;
    std::cout << "- Subsys 2 state: 0x" << std::setfill('0') << std::setw(2)
              << std::hex << static_cast<uint32_t>(state.subsys_2_pwr)
              << std::endl;
    std::cout << "- LED state: 0x" << std::setfill('0') << std::setw(2)
              << std::hex << static_cast<uint32_t>(state.led_state)
              << std::endl;
    std::cout << "- Actuator 1 state: ";
    switch (static_cast<SD::ActuatorState>(state.actuator_state[0])) {
      default:
        std::cout << "UNKNOWN";
        break;
      case SD::ACT_RETRACT:
        std::cout << "RECTRACTED";
        break;
      case SD::ACT_RETRACTING:
        std::cout << "RETRACTING";
        break;
      case SD::ACT_DEPLOYED:
        std::cout << "DEPLOYED";
        break;
    }
    std::cout << std::endl;
    std::cout << "- Actuator 2 state: ";
    switch (static_cast<SD::ActuatorState>(state.actuator_state[1])) {
      default:
        std::cout << "UNKNOWN";
        break;
      case SD::ACT_RETRACT:
        std::cout << "RECTRACTED";
        break;
      case SD::ACT_RETRACTING:
        std::cout << "RETRACTING";
        break;
      case SD::ACT_DEPLOYED:
        std::cout << "DEPLOYED";
        break;
    }
    std::cout << std::endl;
    std::cout << "- Connection 1 state: ";
    switch (static_cast<SD::ConnectionState>(state.conn_state[0])) {
      default:
        std::cout << "UNKNOWN";
        break;
      case SD::CONN_DISCONNECTED:
        std::cout << "DISCONNECTED";
        break;
      case SD::CONN_CONNECTING:
        std::cout << "CONNECTING";
        break;
      case SD::CONN_CONNECTED:
        std::cout << "CONNECTED";
        break;
    }
    std::cout << std::endl;
    std::cout << "- Connection 2 state: ";
    switch (static_cast<SD::ConnectionState>(state.conn_state[1])) {
      default:
        std::cout << "UNKNOWN";
        break;
      case SD::CONN_DISCONNECTED:
        std::cout << "DISCONNECTED";
        break;
      case SD::CONN_CONNECTING:
        std::cout << "CONNECTING";
        break;
      case SD::CONN_CONNECTED:
        std::cout << "CONNECTED";
        break;
    }
    std::cout << std::endl;
    std::cout << "- Loopback 1 state: "
              << (state.loop_back[0] == 1 ? "DISCONNECTED" : "CONNECTED")
              << std::endl;
    std::cout << "- Loopback 2 state: "
              << (state.loop_back[1] == 1 ? "DISCONNECTED" : "CONNECTED")
              << std::endl;
    std::cout << "- Flags : " << std::setfill('0') << std::setw(8) << std::hex
              << state.flags << std::endl;
    return 0;
  }

  // VIEW STRING INFORMATION
  if (FLAGS_string) {
    ValueMap idxs;
    idxs[SD::STRING_SW_VERSION] = Value("Software version", {"sw"});
    idxs[SD::STRING_BUILD] = Value("Build", {"build"});
    idxs[SD::STRING_SERIAL] = Value("Serial", {"serial"});
    if (FLAGS_list) return Print("Indexes", idxs, true);
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the request. Aborting.");
      std::map<SD::String, std::string> data;
      if (!sd.GetStrings(mask, data))
        return Error("There was a problem querying the strings. Aborting.");
      std::map<SD::String, std::string>::iterator it;
      std::cout << "Current string values: " << std::endl;
      for (it = data.begin(); it != data.end(); it++) {
        std::cout << " -" << idxs[it->first].first << ": " << it->second
                  << std::endl;
      }
      return 0;
    }
    if (!FLAGS_set.empty())
      return Error("The -string argument does not support -set");
    return HelpView("string", "View string information");
  }

  // VIEW EPS INFORMATION
  if (FLAGS_berth) {
    ValueMap idxs;
    idxs[SD::BERTH_1] = Value("Berth 1", {"1"});
    idxs[SD::BERTH_2] = Value("Berth 2", {"2"});
    if (FLAGS_list) return Print("Indexes", idxs, true);
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes. Aborted.");
      std::map<SD::Berth, SD::BerthState> data;
      if (!sd.GetBerthStates(mask, data))
        return Error("There was a problem querying the indexes. Aborted.");
      std::map<SD::Berth, SD::BerthState>::iterator it;
      for (it = data.begin(); it != data.end(); it++) {
        std::cout << idxs[it->first].first << std::endl;
        SD::BerthState& berth = it->second;
        // Print the connection state
        std::cout << "- Connection state: ";
        switch (static_cast<EPS::DockStateValue>(berth.dock_state)) {
          default:
            std::cout << "UNKNOWN" << std::endl;
            continue;
          case EPS::DOCK_DISCONNECTED:
            std::cout << "DISCONNECTED" << std::endl;
            continue;
          case EPS::DOCK_CONNECTING:
            std::cout << "CONNECTING" << std::endl;
            continue;
          case EPS::DOCK_CONNECTED:
            std::cout << "CONNECTED" << std::endl;
            break;
        }
        // We only get to this point if we actually have a robot connected. This
        // prevents us printing a whole bunch of nulled values.
        std::cout << "- Serial: " << EPS::SerialToString(berth.serial)
                  << std::endl;
        std::cout << "- Power state: ";
        switch (static_cast<EPS::PowerStateValue>(berth.power_state)) {
          default:
          case EPS::POWER_STATE_UNKNOWN:
            std::cout << "UNKNOWN" << std::endl;
            break;
          case EPS::POWER_STATE_HIBERNATE:
            std::cout << "HIBERNATE" << std::endl;
            break;
          case EPS::POWER_STATE_AWAKE_NOMINAL:
            std::cout << "AWAKE_NOMINAL" << std::endl;
            break;
          case EPS::POWER_STATE_AWAKE_SAFE:
            std::cout << "AWAKE_SAFE" << std::endl;
            break;
          case EPS::POWER_STATE_CRITICAL_FAULT:
            std::cout << "CRITICAL_FAULT" << std::endl;
            break;
        }
        std::cout << "- Terminate: " << (berth.terminate ? "ON" : "OFF")
                  << std::endl;
        std::cout << "- Asserted hardware faults: " << std::endl;
        if (!berth.fault_mask) std::cout << "    NONE" << std::endl;
        for (uint32_t i = 0; i < EPS::NUM_FAULTS; i++) {
          if (!(berth.fault_mask & (1 << i))) continue;
          switch (static_cast<EPS::Fault>(i)) {
            case EPS::FAULT_OC_ENET:
              std::cout << "  FAULT_OC_ENET" << std::endl;
              break;
            case EPS::FAULT_OT_FLASHLIGHT_1:
              std::cout << "  FAULT_OT_FLASHLIGHT_1" << std::endl;
              break;
            case EPS::FAULT_OT_FLASHLIGHT_2:
              std::cout << "  FAULT_OT_FLASHLIGHT_2" << std::endl;
              break;
            case EPS::FAULT_OC_FAN:
              std::cout << "  FAULT_OC_FAN" << std::endl;
              break;
            case EPS::FAULT_OT_MLP:
              std::cout << "  FAULT_OT_MLP" << std::endl;
              break;
            case EPS::FAULT_OT_LLP:
              std::cout << "  FAULT_OT_LLP" << std::endl;
              break;
            case EPS::FAULT_OT_HLP:
              std::cout << "  FAULT_OT_HLP" << std::endl;
              break;
            case EPS::FAULT_OC_USB:
              std::cout << "  FAULT_OC_USB" << std::endl;
              break;
            case EPS::FAULT_OC_LLP:
              std::cout << "  FAULT_OC_LLP" << std::endl;
              break;
            case EPS::FAULT_OC_MLP:
              std::cout << "  FAULT_OC_MLP" << std::endl;
              break;
            case EPS::FAULT_OC_HLP:
              std::cout << "  FAULT_OC_HLP" << std::endl;
              break;
            case EPS::FAULT_OC_AUX:
              std::cout << "  FAULT_OC_AUX" << std::endl;
              break;
            case EPS::FAULT_ST_5A_REG_3:
              std::cout << "  FAULT_ST_5A_REG_3" << std::endl;
              break;
            case EPS::FAULT_OC_5A_REG_2:
              std::cout << "  FAULT_OC_5A_REG_2" << std::endl;
              break;
            case EPS::FAULT_OC_5A_REG_1:
              std::cout << "  FAULT_OC_5A_REG_1" << std::endl;
              break;
            case EPS::FAULT_ST_5A_REG_2:
              std::cout << "  FAULT_ST_5A_REG_2" << std::endl;
              break;
            case EPS::FAULT_OC_PAYLOAD_4:
              std::cout << "  FAULT_OC_PAYLOAD_4" << std::endl;
              break;
            case EPS::FAULT_ST_5A_REG_1:
              std::cout << "  FAULT_ST_5A_REG_1" << std::endl;
              break;
            case EPS::FAULT_OC_PAYLOAD_1:
              std::cout << "  FAULT_OC_PAYLOAD_1" << std::endl;
              break;
            case EPS::FAULT_OC_5A_REG_3:
              std::cout << "  FAULT_OC_5A_REG_3" << std::endl;
              break;
            case EPS::FAULT_OC_PAYLOAD_2:
              std::cout << "  FAULT_OC_PAYLOAD_2" << std::endl;
              break;
            case EPS::FAULT_OC_PAYLOAD_3:
              std::cout << "  FAULT_OC_PAYLOAD_3" << std::endl;
              break;
            default:
              std::cout << "UNKNOWN[" << i << "]";
              break;
          }
        }
        // Active power channels
        std::cout << "- Active power channels: " << std::endl;
        if (!berth.channel_mask) std::cout << "    NONE" << std::endl;
        for (uint32_t i = 0; i < EPS::NUM_CHANNELS; i++) {
          if (!(berth.channel_mask & (1 << i))) continue;
          std::cout << "  ";
          switch (static_cast<EPS::Channel>(i)) {
            case EPS::CHANNEL_LLP_EN:
              std::cout << "  LLP_EN" << std::endl;
              break;
            case EPS::CHANNEL_MLP_EN:
              std::cout << "  MLP_EN" << std::endl;
              break;
            case EPS::CHANNEL_HLP_EN:
              std::cout << "  HLP_EN" << std::endl;
              break;
            case EPS::CHANNEL_USB_PWR_EN:
              std::cout << "  USB_PWR_EN" << std::endl;
              break;
            case EPS::CHANNEL_AUX_PWR_EN:
              std::cout << "  AUX_PWR_EN" << std::endl;
              break;
            case EPS::CHANNEL_ENET_PWR_EN:
              std::cout << "  ENET_PWR_EN" << std::endl;
              break;
            case EPS::CHANNEL_FAN_EN:
              std::cout << "  FAN_EN" << std::endl;
              break;
            case EPS::CHANNEL_SPEAKER_EN:
              std::cout << "  SPEAKER_EN" << std::endl;
              break;
            case EPS::CHANNEL_PAYLOAD_EN_TOP_AFT:
              std::cout << "  PAYLOAD_EN_TOP_AFT" << std::endl;
              break;
            case EPS::CHANNEL_PAYLOAD_EN_BOT_AFT:
              std::cout << "  PAYLOAD_EN_BOT_AFT" << std::endl;
              break;
            case EPS::CHANNEL_PAYLOAD_EN_BOT_FRONT:
              std::cout << "  PAYLOAD_EN_BOT_FRONT" << std::endl;
              break;
            case EPS::CHANNEL_PAYLOAD_EN_TOP_FRONT:
              std::cout << "  PAYLOAD_EN_TOP_FRONT" << std::endl;
              break;
            case EPS::CHANNEL_MOTOR_EN1:
              std::cout << "  MOTOR_EN1" << std::endl;
              break;
            case EPS::CHANNEL_MOTOR_EN2:
              std::cout << "  MOTOR_EN2" << std::endl;
              break;
            case EPS::CHANNEL_STATUSA2_LED:
              std::cout << "  STATUSA2_LED" << std::endl;
              break;
            case EPS::CHANNEL_STATUSA1_LED:
              std::cout << "  STATUSA1_LED" << std::endl;
              break;
            case EPS::CHANNEL_STATUSB2_LED:
              std::cout << "  STATUSB2_LED" << std::endl;
              break;
            case EPS::CHANNEL_STATUSB1_LED:
              std::cout << "  STATUSB1_LED" << std::endl;
              break;
            case EPS::CHANNEL_STATUSC2_LED:
              std::cout << "  STATUSC2_LED" << std::endl;
              break;
            case EPS::CHANNEL_STATUSC1_LED:
              std::cout << "  STATUSC1_LED" << std::endl;
              break;
            case EPS::CHANNEL_VIDEO_LED:
              std::cout << "  VIDEO_LED" << std::endl;
              break;
            case EPS::CHANNEL_AUDIO_LED:
              std::cout << "  AUDIO_LED" << std::endl;
              break;
            case EPS::CHANNEL_LIVE_LED:
              std::cout << "  LIVE_LED" << std::endl;
              break;
            default:
              std::cout << "UNKNOWN[" << i << "]" << std::endl;
              break;
          }
        }
        std::cout << "- Active charge channels: " << std::endl;
        if (!berth.charge_mask) std::cout << "    NONE" << std::endl;
        for (uint32_t i = 0; i < EPS::NUM_CHARGERS; i++) {
          if (!(berth.charge_mask & (1 << i))) continue;
          std::cout << "  ";
          switch (static_cast<EPS::Charger>(i)) {
            default:
              std::cout << "UNKNOWN" << std::endl;
              break;
            case EPS::CHARGER_TOP_RIGHT:
              std::cout << "  CHARGER_TOP_RIGHT" << std::endl;
              break;
            case EPS::CHARGER_BOTTOM_RIGHT:
              std::cout << "  CHARGER_BOTTOM_RIGHT" << std::endl;
              break;
            case EPS::CHARGER_TOP_LEFT:
              std::cout << "  CHARGER_TOP_LEFT" << std::endl;
              break;
            case EPS::CHARGER_BOTTOM_LEFT:
              std::cout << "  CHARGER_BOTTOM_LEFT" << std::endl;
              break;
          }
        }
        // Print out battery information
        for (uint32_t i = 0; i < EPS::NUM_BATTERIES; i++) {
          EPS::BatteryInfo& b = berth.batteries[i];
          switch (static_cast<EPS::Battery>(b.chan)) {
            default:
              std::cout << "- BATTERY UNKNOWN" << std::endl;
              break;
            case EPS::BATTERY_TOP_RIGHT:
              std::cout << "- BATTERY_TOP_RIGHT" << std::endl;
              break;
            case EPS::BATTERY_BOTTOM_RIGHT:
              std::cout << "- BATTERY_BOTTOM_RIGHT" << std::endl;
              break;
            case EPS::BATTERY_TOP_LEFT:
              std::cout << "- BATTERY_TOP_LEFT" << std::endl;
              break;
            case EPS::BATTERY_BOTTOM_LEFT:
              std::cout << "- BATTERY_BOTTOM_LEFT" << std::endl;
              break;
          }
          std::cout << "    Present: " << (b.present ? "TRUE" : "FALSE")
                    << std::endl;
          if (b.present) {
            std::cout << "    Voltage: " << b.voltage << " mV" << std::endl;
            std::cout << "    Current: " << b.current << " mA" << std::endl;
            std::cout << "    Design cap.: " << b.design << " mAh" << std::endl;
            std::cout << "    Full cap.: " << b.full << " mAh" << std::endl;
            std::cout << "    Rem. cap.: " << b.remaining << " mAh"
                      << std::endl;
            std::cout << "    Percentage: " << b.percentage << " %"
                      << std::endl;
            std::cout << "    Cell voltage: ";
            for (int i = 0; i < 4; i++)
              std::cout << "[" << i << "] " << b.cell[i] << " mV ";
            std::cout << std::endl;
            std::cout << "    Status bits: 0x" << std::hex << std::uppercase
                      << b.status << std::dec << std::nouppercase << std::endl;
            std::cout << "    Temperature: "
                      << static_cast<float>(b.temperature) / 10.0f - 273.15
                      << " deg C" << std::endl;
            std::cout << "    Serial number: "
                      << static_cast<unsigned int>(b.serial) << std::endl;
          }
        }
      }
      return 0;
    }
    if (!FLAGS_set.empty())
      return Error("The -battery argument does not support -set");
    return HelpView("battery", "View battery information");
  }

  // VIEW HOUESEKEEPING INFORMATION
  if (FLAGS_hk) {
    ValueMap idxs;
    idxs[SD::HK_FAN_MAG_I] = Value("HK_FAN_MAG_I", {"mag_i"});
    idxs[SD::HK_CHR_V_V] = Value("HK_CHR_V_V", {"chr_v"});
    idxs[SD::HK_CHR_T_PROTECT] = Value("HK_CHR_T_PROTECT", {"chr_t"});
    idxs[SD::HK_VLIVE_I] = Value("HK_VLIVE_I", {"vlive_i"});
    idxs[SD::HK_MAIN5_PWR_I] = Value("HK_MAIN5_PWR_I", {"main5_i"});
    idxs[SD::HK_DEV_I] = Value("HK_DEV_I", {"dev_i"});
    idxs[SD::HK_EC_PWR_I] = Value("HK_EC_PWR_I", {"ec_pwr_i"});
    idxs[SD::HK_A_GND_V1] = Value("HK_A_GND_V1", {"a_gnd_1"});
    idxs[SD::HK_AS2_I] = Value("HK_AS2_I", {"as2_i"});
    idxs[SD::HK_AS1_I] = Value("HK_AS1_I", {"as1_i"});
    idxs[SD::HK_A_GND_V2] = Value("HK_A_GND_V2", {"a_gnd_2"});
    idxs[SD::HK_DEV1_T_PROTECT] = Value("HK_DEV1_T_PROTECT", {"dev1_t"});
    idxs[SD::HK_DEV2_T_PROTECT] = Value("HK_DEV2_T_PROTECT", {"dev2_t"});
    idxs[SD::HK_A_GND_V3] = Value("HK_A_GND_V3", {"a_gnd_3"});
    idxs[SD::HK_DEV2_I] = Value("HK_DEV2_I", {"dev2_i"});
    idxs[SD::HK_DEV1_I] = Value("HK_DEV1_I", {"dev1_i"});
    if (FLAGS_list) return Print("Indexes", idxs, true);
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes. Aborted.");
      std::map<SD::Housekeeping, double> data;
      if (!sd.GetHousekeeping(mask, data))
        return Error("There was a problem querying the indexes. Aborted.");
      std::map<SD::Housekeeping, double>::iterator it;
      std::cout << "Current housekeeping values: " << std::endl;
      for (it = data.begin(); it != data.end(); it++)
        std::cout << "- " << idxs[it->first].first << ": " << std::fixed
                  << std::setprecision(4) << it->second << std::endl;
      return 0;
    }
    if (!FLAGS_set.empty())
      return Error("The -housekeeping argument does not support -set");
    return HelpView("housekeeping", "View housekeeping information");
  }

  // Power channels
  if (FLAGS_power) {
    ValueMap idxs;
    idxs[SD::CHANNEL_DEV_EN] = Value("DEV_EN", {"dev"});
    idxs[SD::CHANNEL_EC_PWR_EN] = Value("EC_PWR_EN", {"ec"});
    idxs[SD::CHANNEL_DEV2_PWR_EN] = Value("DEV2_PWR_EN", {"dev2"});
    idxs[SD::CHANNEL_DEV1_PWR_EN] = Value("DEV1_PWR_EN", {"dev1"});
    idxs[SD::CHANNEL_ACT_SIGOUT2] = Value("ACT_SIGOUT2", {"act2"});
    idxs[SD::CHANNEL_ACT_SIGOUT1] = Value("ACT_SIGOUT1", {"act1"});
    idxs[SD::CHANNEL_AS2_PWR_EN] = Value("AS2_PWR_EN", {"as2"});
    idxs[SD::CHANNEL_AS1_PWR_EN] = Value("AS1_PWR_EN", {"as1"});
    idxs[SD::CHANNEL_FAN_EN] = Value("FAN_EN", {"fan"});
    idxs[SD::CHANNEL_LED_1] = Value("LED_1", {"led1"});
    idxs[SD::CHANNEL_LED_2] = Value("LED_2", {"led2"});
    idxs[SD::CHANNEL_LED_3] = Value("LED_3", {"led3"});
    idxs[SD::CHANNEL_LED_4] = Value("LED_4", {"led4"});
    idxs[SD::CHANNEL_LED_5] = Value("LED_5", {"led5"});
    idxs[SD::CHANNEL_LED_6] = Value("LED_6", {"led6"});
    ValueMap vals;
    vals[SD::OFF] = Value("Turn off", {"disable", "off", "false", "0"});
    vals[SD::ON] = Value("Turn on", {"enable", "on", "true", "1"});
    if (FLAGS_list) {
      Print("Indexes", idxs);
      Print("Values", vals);
      return 0;
    }
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes. Aborted.");
      std::map<SD::Channel, bool> data;
      if (!sd.GetChannels(mask, data))
        return Error("There was a problem querying the indexes. Aborted.");
      std::map<SD::Channel, bool>::iterator it;
      std::cout << "Current power states:" << std::endl;
      for (it = data.begin(); it != data.end(); it++)
        std::cout << "- " << idxs[it->first].first << ": "
                  << (it->second ? "TRUE" : "FALSE") << std::endl;
      return 0;
    }
    if (!FLAGS_set.empty()) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, false))
        return Error("There was a problem parsing the indexes you supplied.");
      uint32_t value;
      if (!Valid(vals, FLAGS_set, value))
        return Error("The supplied value for -set is not supported");
      if (!sd.SetChannels(mask, value))
        return Error(
            "There was a problem setting the channel values. Aborted.");
      return Print("Power channels set successfully");
    }
    return HelpConf("power", "Configure power channels");
  }

  // SET COMMAND
  if (FLAGS_command) {
    ValueMap idxs;
    idxs[SD::BERTH_1] = Value("Berth 1", {"1"});
    idxs[SD::BERTH_2] = Value("Berth 2", {"2"});
    ValueMap vals;
    vals[SD::COMMAND_SET_POWER_MODE_HIBERNATE] =
        Value("Set power mode to hibernate", {"hibernate"});
    vals[SD::COMMAND_SET_POWER_MODE_AWAKE_NOMINAL] =
        Value("Set power mode to wake (nominal)", {"wake"});
    vals[SD::COMMAND_SET_POWER_MODE_AWAKE_SAFE] =
        Value("Set power mode to  safe mode", {"safe"});
    vals[SD::COMMAND_SET_POWER_MODE_CRITICAL_FAULT] =
        Value("Set power mode to critical fault", {"critical"});
    vals[SD::COMMAND_CLEAR_TERMINATE] =
        Value("Clear terminate", {"unterminate"});
    vals[SD::COMMAND_CLEAR_FAULTS] = Value("Clear faults", {"clear"});
    vals[SD::COMMAND_ENABLE_ALL_PAYLOADS] =
        Value("Enable power to all payloads", {"pay_on"});
    vals[SD::COMMAND_DISABLE_ALL_PAYLOADS] =
        Value("Disable power to all payloads", {"pay_off"});
    vals[SD::COMMAND_ENABLE_ALL_PMCS] =
        Value("Enable power to all PMCs", {"pmc_on"});
    vals[SD::COMMAND_DISABLE_ALL_PMCS] =
        Value("Disable power to all PMCs", {"pmc_of"});
    vals[SD::COMMAND_REBOOT] = Value("Reboot", {"reboot"});
    if (FLAGS_list) {
      Print("Indexes", idxs);
      Print("Values", vals);
      return 0;
    }
    if (FLAGS_get) return Error("The -command argument does not support -get");
    if (!FLAGS_set.empty()) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes you supplied.");
      uint32_t value;
      if (!Valid(vals, FLAGS_set, value))
        return Error("The supplied value for -set is not supported");
      if (!sd.SendBerthCommand(mask, static_cast<SD::BerthCommand>(value)))
        return Error("There was a problem sending the berth command. Aborted.");
      return Print("Berth comand set successfully");
    }
    return HelpConf("command", "Command berths");
  }

  // MANAGE FAULT INFORMATION
  if (FLAGS_fault) {
    ValueMap idxs;
    idxs[SD::FAULT_OC_BERTH_1] = Value("Berth1 o/current", {"c_berth1"});
    idxs[SD::FAULT_OC_BERTH_2] = Value("Berth2 o/current", {"c_berth2"});
    idxs[SD::FAULT_OC_SYSTEM] = Value("System c/current", {"c_sys"});
    idxs[SD::FAULT_OC_DOCK_PROCESSOR] = Value("CPU o/current", {"c_cpu"});
    idxs[SD::FAULT_OT_CHARGER] = Value("Charger o/temp", {"t_chg"});
    idxs[SD::FAULT_OT_ACTUATOR_1] = Value("Actuator 1 o/temp", {"t_act1"});
    idxs[SD::FAULT_OT_ACTUATOR_2] = Value("Actuator 2 o/temp", {"t_act2"});
    if (FLAGS_list) return Print("Indexes", idxs, true);
    if (FLAGS_clear) {
      if (!sd.ClearFaults()) return Error("Could not clear the faults.");
      return Print("Faults cleared successfully");
    }
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes. Aborted.");
      std::map<SD::Fault, bool> data;
      if (!sd.GetFaults(mask, data))
        return Error("There was a problem querying the indexes. Aborted.");
      std::map<SD::Fault, bool>::iterator it;
      std::cout << "Current fault values: " << std::endl;
      for (it = data.begin(); it != data.end(); it++)
        std::cout << "- " << idxs[it->first].first << ": "
                  << (it->second ? "FAULT" : "NOMINAL") << std::endl;
      return 0;
    }
    if (!FLAGS_set.empty())
      return Error("The -fault argument does not support -set");
    return HelpView("fault", "View fault information");
  }

  // If we get here there was no valid command, so print useage info and exit
  return Help();
}  // NOLINT
