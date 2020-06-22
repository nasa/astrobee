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
#include <eps_driver/eps_driver.h>

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
DEFINE_string(device, "/dev/i2c-1", "i2c bus of EPS");
DEFINE_int32(address, 0x40, "i2c address of EPS");
DEFINE_int32(retries, 3, "i2c retries");
DEFINE_string(w, "", "Write power state to given file");

// Single-switch options
DEFINE_bool(reboot, false, "Reboot");
DEFINE_bool(bootloader, false, "Jump to bootloader");
DEFINE_bool(undock, false, "Send an undock command");
DEFINE_bool(unterminate, false, "Clear a terminate event");
DEFINE_bool(buzz, false, "Ring the buzzer");
DEFINE_bool(hk, false, "View housekeeping information");
DEFINE_bool(string, false, "View string information");
DEFINE_bool(battery, false, "View battery information");
DEFINE_bool(temp, false, "View temperatures");
DEFINE_bool(fault, false, "View and clear faults");
DEFINE_bool(power, false, "View and toggle power channels");
DEFINE_bool(charge, false, "View and toggle charge state");
DEFINE_bool(led, false, "View and configure LEDs");
DEFINE_bool(state, false, "View state information");

// Second-level options for VIEW [read-only multiple options]
DEFINE_int32(freq, 1000, "Buzzer frequency in Hz (1000 - 2000)");
DEFINE_string(set, "", "Set a new value for all specified indexes");
DEFINE_bool(get, false, "Get the current value for all specified indexes");
DEFINE_bool(list, false, "List all indexes and keys for the given command");
DEFINE_bool(clear, false, "Send a clear command (only for -fault)");

// What we'll use to index various channels, payloads, batteries, etc.
typedef std::vector<std::string> Keywords;
typedef std::pair<std::string, Keywords> Value;
typedef std::map<uint32_t, Value> ValueMap;

// Avoid long constants
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
            << " eps_driver_tool -" << flag << " [ -list | -get ] " << extra
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
            << " eps_driver_tool -" << flag << " [ -list | -get | -set <val> ]"
            << extra << std::endl;
  return 0;
}

// Help buzzer
int HelpBuzz(std::string const& flag, std::string const& desc) {
  std::cout << "DESCRIPTION" << std::endl
            << " " << desc << std::endl
            << std::endl
            << "USAGE OVERVIEW" << std::endl
            << " The -" << flag << " flag can also optionally be paired with"
            << std::endl
            << " the -freq flag to set the desired frequency in the range"
            << std::endl
            << " 1000 - 2000 Hertz." << std::endl
            << std::endl
            << "USAGE PATTERN" << std::endl
            << " eps_driver_tool -" << flag << " [ -freq <freq> ] <duration>"
            << std::endl;
  return 0;
}

// Help buzzer
int Help() {
  std::cout << "DESCRIPTION" << std::endl
            << " Command-line tool for interacting with the EPS subsystem"
            << std::endl
            << std::endl
            << "SUBSYSTEM CONTROL" << std::endl
            << " The tool supports the following subsystem flags" << std::endl
            << "  -power    : Turn on an off power to channels" << std::endl
            << "  -led      : Configure LEDs to be on, off or flash"
            << std::endl
            << "  -battery  : View battery information" << std::endl
            << "  -charge   : View charger information" << std::endl
            << "  -hk       : View housekeeping information" << std::endl
            << "  -fault    : View and clear fault information" << std::endl
            << "  -state    : View docking and power states" << std::endl
            << "  -string   : View build info, software version and serial"
            << std::endl
            << "  -temp     : View temperature information" << std::endl
            << "  -buzz     : Ring the buzzer" << std::endl
            << " For more help about a specific subsystem, add the flag"
            << std::endl
            << "  eg. eps_driver_tool -power" << std::endl
            << std::endl
            << "ONE-SHOT COMMANDS" << std::endl
            << " The tool supports the following oneshot commands" << std::endl
            << "  -undock       : Force an undock" << std::endl
            << "  -unterminate  : Clear a TERMINATE state" << std::endl
            << "  -reboot       : Reboot the EPS" << std::endl
            << "  -bootloader   : Enter the EPS bootloader" << std::endl
            << " To use the command, just add the flag" << std::endl
            << "  eg. eps_driver_tool -reboot";
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

// Sleep function required by EPS driver
void Sleep(uint32_t microseconds) {
  struct timespec req, rem;
  req.tv_sec = microseconds / 1000000;
  req.tv_nsec = (microseconds % 1000000) * 100;
  while ((req.tv_sec != 0) || (req.tv_nsec != 0)) {
    if (nanosleep(&req, &rem) == 0) break;
    if (errno == EINTR) {
      req.tv_sec = rem.tv_sec;
      req.tv_nsec = rem.tv_nsec;
      continue;
    }
    std::cout << "Warning: nanosleep terminated prematurely" << std::endl;
    break;
  }
}

// Main entry point for application
int main(int argc, char** argv) {
  // Set up gflags
  google::SetUsageMessage("Usage: eps_driver_tool <options> [value]\n");
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

  // Attach an EPS driver to the device on the bus
  EPS eps(device, std::bind(&Sleep, std::placeholders::_1));

  // WRITE CURRENT STATE TO FILE
  if (!FLAGS_w.empty()) {
    // Open a file to write content
    std::ofstream ofs(FLAGS_w, std::ofstream::trunc);
    if (!ofs.is_open()) {
      std::cerr << "Unable to open power mode file" << std::endl;
      return 3;
    }
    // Get the power state
    std::map<EPS::State, uint8_t> states;
    if (eps.GetStates((1 << EPS::STATE_POWER), states)) {
      switch (states[EPS::STATE_POWER]) {
        default:
        case EPS::POWER_STATE_UNKNOWN:
          ofs << "unknown";
          break;
        case EPS::POWER_STATE_HIBERNATE:
          ofs << "hibernate";
          break;
        case EPS::POWER_STATE_AWAKE_SAFE:
          ofs << "safe";
          break;
        case EPS::POWER_STATE_CRITICAL_FAULT:
          ofs << "fault";
          break;
        case EPS::POWER_STATE_AWAKE_NOMINAL:
          break;
      }
    } else {
      ofs << "cannot_query";
    }
    // Close the file and exit gracefully
    ofs.close();
    return 0;
  }

  // REBOOT
  if (FLAGS_reboot) {
    if (!eps.Reboot()) return Error("Failed to send a REBOOT command");
    return Print("Sent a REBOOT command successfully");
  }

  // JUMP TO BOOTLOADER
  if (FLAGS_bootloader) {
    if (!eps.EnterBootloader())
      return Error("Failed to send an ENTER_BOOTLOADER command");
    return Print("Sent an ENTER_BOOTLOADER command successfully");
  }

  // SEND UNDOCK COMMAND
  if (FLAGS_undock) {
    if (!eps.Undock()) return Error("Failed to send an UNDOCK command.");
    return Print("Sent an UNDOCK command successfully");
  }

  // REVERT A TERMINATE STATE
  if (FLAGS_unterminate) {
    if (!eps.Unterminate()) return Error("Failed to clear the TERMINATE event");
    return Print("Cleared the TERMINATE event successfully");
  }

  // RING THE BUZZER
  if (FLAGS_buzz) {
    if (input.empty())
      return Error("Buzzer expects a single argument <duration>");
    if (input.size() == 1) {
      uint8_t duration = static_cast<uint8_t>(std::stoi(input[0]));
      if (!eps.RingBuzzer(FLAGS_freq, duration))
        return Error("Could not ring the buzzer");
      return Print("Buzzer rung");
    }
    return HelpBuzz("buzzer", "Ring the buzzer for a given duration");
  }

  // VIEW STRING INFORMATION
  if (FLAGS_string) {
    ValueMap idxs;
    idxs[EPS::STRING_SW_VERSION] = Value("Software version", {"sw"});
    idxs[EPS::STRING_BUILD] = Value("Build", {"build"});
    idxs[EPS::STRING_SERIAL] = Value("Serial", {"serial"});
    if (FLAGS_list) return Print("Indexes", idxs, true);
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the request. Aborting.");
      std::map<EPS::String, std::string> data;
      if (!eps.GetStrings(mask, data))
        return Error("There was a problem querying the strings. Aborting.");
      std::map<EPS::String, std::string>::iterator it;
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

  // VIEW BATTERY INFORMATION
  if (FLAGS_battery) {
    ValueMap idxs;
    idxs[EPS::BATTERY_TOP_LEFT] = Value("Battery top left/port", {"tl"});
    idxs[EPS::BATTERY_TOP_RIGHT] = Value("Battery top right/stbd", {"tr"});
    idxs[EPS::BATTERY_BOTTOM_LEFT] = Value("Battery bot left/port", {"bl"});
    idxs[EPS::BATTERY_BOTTOM_RIGHT] = Value("Battery bot right/stbd", {"br"});
    if (FLAGS_list) return Print("Indexes", idxs, true);
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes. Aborted.");
      std::map<EPS::Battery, EPS::BatteryInfo> data;
      if (!eps.GetBatteries(mask, data))
        return Error("There was a problem querying the indexes. Aborted.");
      std::map<EPS::Battery, EPS::BatteryInfo>::iterator it;
      for (it = data.begin(); it != data.end(); it++) {
        EPS::BatteryInfo& b = it->second;
        std::cout << idxs[it->first].first << std::endl;
        std::cout << "- Channel: " << static_cast<int>(b.chan) << std::endl;
        std::cout << "- Present: " << (b.present ? "TRUE" : "FALSE")
                  << std::endl;
        if (b.present) {
          std::cout << "- Voltage: " << b.voltage << " mV" << std::endl;
          std::cout << "- Current: " << b.current << " mA" << std::endl;
          std::cout << "- Design cap.: " << b.design << " mAh" << std::endl;
          std::cout << "- Full cap.: " << b.full << " mAh" << std::endl;
          std::cout << "- Rem. cap.: " << b.remaining << " mAh" << std::endl;
          std::cout << "- Percentage: " << b.percentage << " %" << std::endl;
          std::cout << "- Cell voltage: " << std::endl;
          for (int i = 0; i < 4; i++)
            std::cout << "- [" << i << "] " << b.cell[i] << " mV" << std::endl;
          std::cout << "- Status bits: 0x" << std::hex << std::uppercase
                    << b.status << std::dec << std::nouppercase << std::endl;
          std::cout << "- Temperature: "
                    << static_cast<float>(b.temperature) / 10.0f - 273.15
                    << " deg C" << std::endl;
          std::cout << "- Serial number: "
                    << static_cast<unsigned int>(b.serial) << std::endl;
        }
      }
      return 0;
    }
    if (!FLAGS_set.empty())
      return Error("The -battery argument does not support -set");
    return HelpView("battery", "View battery information");
  }

  // VIEW TEMPERATURE INFORMATION
  if (FLAGS_temp) {
    ValueMap idxs;
    idxs[EPS::TEMP_BOTTOM] = Value("Bottom side of board", {"bot"});
    idxs[EPS::TEMP_TOP] = Value("Top side of board", {"top"});
    idxs[EPS::TEMP_CONNECTOR] = Value("Connector", {"con"});
    if (FLAGS_list) return Print("Indexes", idxs, true);
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes. Aborted.");
      std::map<EPS::Temp, EPS::TempInfo> data;
      if (!eps.GetTemps(mask, data))
        return Error("There was a problem querying the indexes. Aborted.");
      std::map<EPS::Temp, EPS::TempInfo>::iterator it;
      std::cout << "Current temperatures:" << std::endl;
      for (it = data.begin(); it != data.end(); it++)
        std::cout << "- " << idxs[it->first].first << ": " << it->second.temp
                  << " celcius" << std::endl;
      return 0;
    }
    if (!FLAGS_set.empty())
      return Error("The -temp argument does not support -set");
    return HelpView("temp", "View temperature information");
  }

  // VIEW CONNECTION INFORMATION
  if (FLAGS_state) {
    ValueMap idxs;
    idxs[EPS::STATE_POWER] = Value("Power state", {"power"});
    idxs[EPS::STATE_DOCK] = Value("Dock State", {"dock"});
    ValueMap pvals;
    pvals[EPS::POWER_STATE_UNKNOWN] = Value("Unknown", {"unknown"});
    pvals[EPS::POWER_STATE_HIBERNATE] = Value("Hibernating", {"hibernate"});
    pvals[EPS::POWER_STATE_AWAKE_NOMINAL] = Value("Nominal", {"nominal"});
    pvals[EPS::POWER_STATE_AWAKE_SAFE] = Value("Safe", {"safe"});
    pvals[EPS::POWER_STATE_CRITICAL_FAULT] = Value("Critical fault", {"fault"});
    ValueMap dvals;
    dvals[EPS::DOCK_DISCONNECTED] = Value("Disconnected", {"disconnected"});
    dvals[EPS::DOCK_CONNECTING] = Value("Connecting", {"connecting"});
    dvals[EPS::DOCK_CONNECTED] = Value("Connected", {"connected"});
    if (FLAGS_list) {
      Print("Indexes", idxs);
      Print("Power state values", pvals, true);
      Print("Dock state values", dvals, true);
      return 0;
    }
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes. Aborted.");
      std::map<EPS::State, uint8_t> data;
      if (!eps.GetStates(mask, data))
        return Error("There was a problem querying the indexes. Aborted.");
      std::map<EPS::State, uint8_t>::iterator it;
      std::cout << "Current states:" << std::endl;
      for (it = data.begin(); it != data.end(); it++) {
        std::cout << "- " << idxs[it->first].first << ": ";
        switch (it->first) {
          case EPS::STATE_POWER:
            std::cout << pvals[it->second].first;
            break;
          case EPS::STATE_DOCK:
            std::cout << dvals[it->second].first;
            break;
          default:
            std::cout << "unknown";
            break;
        }
        std::cout << std::endl;
      }
      return 0;
    }
    // Only allow the power state to be set. The dock state is immutable.
    if (!FLAGS_set.empty()) {
      ValueMap tmp;
      tmp[EPS::STATE_POWER] = Value("Power state", {"power"});
      uint32_t mask;
      if (!Mask(tmp, input, mask, false))
        return Error("There was a problem parsing the indexes you supplied.");
      // Now get the value to set
      uint32_t value;
      if (!Valid(pvals, FLAGS_set, value))
        return Error("The supplied value for -set is not supported");
      // Set the power state
      if (!eps.SetPowerState(static_cast<EPS::PowerStateValue>(value)))
        return Error("There was a problem setting the power state. Aborted.");
      return Print("Power state set successfully");
    }
    return HelpView("state", "View and set certain state information");
  }

  // VIEW HOUESEKEEPING INFORMATION
  if (FLAGS_hk) {
    ValueMap idxs;
    idxs[EPS::HK_AGND1_V] = Value("Analog GND 1 voltage", {"v_agnd1"});
    idxs[EPS::HK_SUPPLY_IN_V] = Value("Supply voltage", {"v_supin"});
    idxs[EPS::HK_PAYLOAD_PWR3_I] = Value("Payload 3 current", {"c_pay3"});
    idxs[EPS::HK_SUBSYS1_1_PWR_V] = Value("Subsystem 1,1 voltage", {"v_ss12"});
    idxs[EPS::HK_SUBSYS1_2_PWR_V] = Value("Subsystem 1,2 voltage", {"v_ss11"});
    idxs[EPS::HK_UNREG_V] = Value("Unregulated voltage", {"v_unreg"});
    idxs[EPS::HK_SYSTEM_I] = Value("System current", {"c_sys"});
    idxs[EPS::HK_BAT4V_V] = Value("Battery 4 voltage", {"v_bat4"});
    idxs[EPS::HK_BAT3V_V] = Value("Battery 3 voltage", {"v_bat3"});
    idxs[EPS::HK_BAT2V_V] = Value("Battery 2 voltage", {"v_bat2"});
    idxs[EPS::HK_BAT1V_V] = Value("Battery 1 voltage", {"v_bat1"});
    idxs[EPS::HK_SUPPLY_I] = Value("Supply current", {"v_supp"});
    idxs[EPS::HK_5VLIVE_V] = Value("5V live voltage", {"v_live"});
    idxs[EPS::HK_AGND2_V] = Value("Analog GND 2 voltage", {"v_agnd2"});
    idxs[EPS::HK_FAN_PWR_I] = Value("Fan current", {"c_fan"});
    idxs[EPS::HK_AUX_PWR_I] = Value("Auxiliary current", {"c_aux"});
    idxs[EPS::HK_PAYLOAD_PWR4_I] = Value("Payload 4 current", {"c_pay4"});
    idxs[EPS::HK_PAYLOAD_PWR2_I] = Value("Payload 2 current", {"c_pay2"});
    idxs[EPS::HK_PAYLOAD_PWR1_I] = Value("Payload 1 current", {"c_pay1"});
    idxs[EPS::HK_5A_REG1_PWR_I] = Value("5A regulator 1 current", {"c_reg1"});
    idxs[EPS::HK_MOTOR1_I] = Value("PMC 1 (right) current", {"c_pmc1"});
    idxs[EPS::HK_SUBSYS2_PWR_V] = Value("Subsystem 2 voltage", {"v_ss2"});
    idxs[EPS::HK_MOTOR2_I] = Value("PMC 2 (left) current", {"c_pmc2"});
    idxs[EPS::HK_5A_REG2_PWR_I] = Value("5A regulator 2 current", {"c_reg2"});
    idxs[EPS::HK_5A_REG3_PWR_I] = Value("5A regulator 3 current", {"c_reg3"});
    idxs[EPS::HK_MAIN5_PWR_I] = Value("Main 5 current", {"c_main"});
    idxs[EPS::HK_AUO_PWR_I] = Value("AUO current", {"c_auo"});
    idxs[EPS::HK_HLP_I] = Value("HLP current", {"c_hlp"});
    idxs[EPS::HK_USB_PWR_I] = Value("Universal serial bus current", {"c_usb"});
    idxs[EPS::HK_LLP_I] = Value("LLP current", {"c_llp"});
    idxs[EPS::HK_MLP_I] = Value("MLP current", {"c_mlp"});
    idxs[EPS::HK_ENET_PWR_I] = Value("Ethernet current", {"c_eth"});
    if (FLAGS_list) return Print("Indexes", idxs, true);
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes. Aborted.");
      std::map<EPS::Housekeeping, double> data;
      if (!eps.GetHousekeeping(mask, data))
        return Error("There was a problem querying the indexes. Aborted.");
      std::map<EPS::Housekeeping, double>::iterator it;
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

  // MANAGE FAULT INFORMATION
  if (FLAGS_fault) {
    ValueMap idxs;
    idxs[EPS::FAULT_OC_ENET] = Value("Ethernet o/current", {"c_eth"});
    idxs[EPS::FAULT_OT_FLASHLIGHT_1] = Value("Light 1 o/temp", {"t_light1"});
    idxs[EPS::FAULT_OT_FLASHLIGHT_2] = Value("Light 2 o/temp", {"t_light2"});
    idxs[EPS::FAULT_OC_FAN] = Value("Fan o/current", {"t_fan"});
    idxs[EPS::FAULT_OT_MLP] = Value("MLP o/temp", {"t_mlp"});
    idxs[EPS::FAULT_OT_LLP] = Value("LLP o/temp", {"t_llp"});
    idxs[EPS::FAULT_OT_HLP] = Value("HLP o/temp", {"t_hlp"});
    idxs[EPS::FAULT_OC_USB] = Value("USB o/current", {"c_usb"});
    idxs[EPS::FAULT_OC_LLP] = Value("LLP o/current", {"c_llp"});
    idxs[EPS::FAULT_OC_MLP] = Value("MLP o/current", {"c_mlp"});
    idxs[EPS::FAULT_OC_HLP] = Value("HLP o/current", {"c_hlp"});
    idxs[EPS::FAULT_OC_AUX] = Value("Auxiliary o/current", {"c_aux"});
    idxs[EPS::FAULT_ST_5A_REG_3] = Value("5A reg 3 state", {"s_reg3"});
    idxs[EPS::FAULT_OC_5A_REG_3] = Value("5A reg 3 o/current", {"c_reg3"});
    idxs[EPS::FAULT_OC_5A_REG_2] = Value("5A reg 2 o/current", {"c_reg2"});
    idxs[EPS::FAULT_ST_5A_REG_2] = Value("5A reg 2 state", {"s_reg2"});
    idxs[EPS::FAULT_OC_5A_REG_1] = Value("5A reg 1 o/current", {"c_reg1"});
    idxs[EPS::FAULT_ST_5A_REG_1] = Value("5A reg 1 state", {"s_reg1"});
    idxs[EPS::FAULT_OC_PAYLOAD_1] = Value("Payload 1 o/current", {"c_pay1"});
    idxs[EPS::FAULT_OC_PAYLOAD_2] = Value("Payload 2 o/current", {"c_pay2"});
    idxs[EPS::FAULT_OC_PAYLOAD_3] = Value("Payload 3 o/current", {"c_pay3"});
    idxs[EPS::FAULT_OC_PAYLOAD_4] = Value("Payload 4 o/current", {"c_pay4"});
    if (FLAGS_list) return Print("Indexes", idxs, true);
    if (FLAGS_clear) {
      if (!eps.ClearFaults()) return Error("Could not clear the faults.");
      return Print("Faults cleared successfully");
    }
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes. Aborted.");
      std::map<EPS::Fault, bool> data;
      if (!eps.GetFaults(mask, data))
        return Error("There was a problem querying the indexes. Aborted.");
      std::map<EPS::Fault, bool>::iterator it;
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

  // TOGGLE POWER
  if (FLAGS_power) {
    ValueMap idxs;
    idxs[EPS::CHANNEL_LLP_EN] = Value("Low-level processor", {"llp"});
    idxs[EPS::CHANNEL_MLP_EN] = Value("Mid-level processor", {"mlp"});
    idxs[EPS::CHANNEL_HLP_EN] = Value("High-level processor", {"hlp"});
    idxs[EPS::CHANNEL_USB_PWR_EN] = Value("Universal serial bus", {"usb"});
    idxs[EPS::CHANNEL_AUX_PWR_EN] = Value("Auxiliary", {"aux"});
    idxs[EPS::CHANNEL_ENET_PWR_EN] = Value("Ethernet", {"eth"});
    idxs[EPS::CHANNEL_FAN_EN] = Value("Fan", {"fan"});
    idxs[EPS::CHANNEL_SPEAKER_EN] = Value("Speaker", {"spk"});
    idxs[EPS::CHANNEL_PAYLOAD_EN_TOP_AFT] =
        Value("Payload top-aft", {"pay_ta"});
    idxs[EPS::CHANNEL_PAYLOAD_EN_BOT_AFT] =
        Value("Payload bot-aft", {"pay_ba"});
    idxs[EPS::CHANNEL_PAYLOAD_EN_BOT_FRONT] =
        Value("Payload bot-front", {"pay_bf"});
    idxs[EPS::CHANNEL_PAYLOAD_EN_TOP_FRONT] =
        Value("Payload top-front", {"pay_tf"});
    idxs[EPS::CHANNEL_MOTOR_EN1] = Value("PMC 1 right/stbd", {"pmc1"});
    idxs[EPS::CHANNEL_MOTOR_EN2] = Value("PMC 2 left/port", {"pmc2"});
    idxs[EPS::CHANNEL_VIDEO_LED] = Value("LED Video", {"video"});
    idxs[EPS::CHANNEL_AUDIO_LED] = Value("LED Audio", {"audio"});
    idxs[EPS::CHANNEL_LIVE_LED] = Value("LED Live", {"live"});
    idxs[EPS::CHANNEL_STATUSA2_LED] = Value("LED Status A2", {"a2"});
    idxs[EPS::CHANNEL_STATUSA1_LED] = Value("LED Status A1", {"a1"});
    idxs[EPS::CHANNEL_STATUSB2_LED] = Value("LED Status B2", {"b2"});
    idxs[EPS::CHANNEL_STATUSB1_LED] = Value("LED Status B1", {"b1"});
    idxs[EPS::CHANNEL_STATUSC2_LED] = Value("LED Status C2", {"c2"});
    idxs[EPS::CHANNEL_STATUSC1_LED] = Value("LED Status C1", {"c1"});
    ValueMap vals;
    vals[EPS::OFF] = Value("Turn off", {"disable", "off", "false", "0"});
    vals[EPS::ON] = Value("Turn on", {"enable", "on", "true", "1"});
    if (FLAGS_list) {
      Print("Indexes", idxs);
      Print("Values", vals);
      return 0;
    }
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes. Aborted.");
      std::map<EPS::Channel, bool> data;
      if (!eps.GetChannels(mask, data))
        return Error("There was a problem querying the indexes. Aborted.");
      std::map<EPS::Channel, bool>::iterator it;
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
      if (!eps.SetChannels(mask, value))
        return Error(
            "There was a problem setting the channel values. Aborted.");
      return Print("Power channels set successfully");
    }
    return HelpConf("power", "Configure power channels");
  }

  // TOGGLE CHARGE STATE
  if (FLAGS_charge) {
    ValueMap idxs;
    idxs[EPS::CHARGER_TOP_LEFT] = Value("Charger top left/port", {"tl"});
    idxs[EPS::CHARGER_TOP_RIGHT] = Value("Charger top right/stbd", {"tr"});
    idxs[EPS::CHARGER_BOTTOM_LEFT] = Value("Charger bot left/port", {"bl"});
    idxs[EPS::CHARGER_BOTTOM_RIGHT] = Value("Charger bot right/stbd", {"br"});
    ValueMap vals;
    vals[EPS::OFF] = Value("Turn off", {"disable", "off", "false", "0"});
    vals[EPS::ON] = Value("Turn on", {"enable", "on", "true", "1"});
    if (FLAGS_list) {
      Print("Indexes", idxs);
      Print("Values", vals);
      return 0;
    }
    if (FLAGS_get) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes. Aborted.");
      std::map<EPS::Charger, bool> data;
      if (!eps.GetChargers(mask, data))
        return Error("There was a problem querying the indexes. Aborted.");
      std::map<EPS::Charger, bool>::iterator it;
      std::cout << "Current charge states:" << std::endl;
      for (it = data.begin(); it != data.end(); it++)
        std::cout << "- " << idxs[it->first].first << ": "
                  << (it->second ? "ENABLED" : "DISABLED") << std::endl;
      return 0;
    }
    if (!FLAGS_set.empty()) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes you supplied.");
      uint32_t value;
      if (!Valid(vals, FLAGS_set, value))
        return Error("The supplied value for -set is not supported");
      if (!eps.SetChargers(mask, value))
        return Error("There was a problem setting the charge values. Aborted.");
      return Print("Charge state set successfully");
    }
    return HelpConf("charge", "Configure charging systems");
  }

  // CONFIGURE LED
  if (FLAGS_led) {
    ValueMap idxs;
    idxs[EPS::LED_VIDEO] = Value("Video", {"video"});
    idxs[EPS::LED_AUDIO] = Value("Audio", {"audio"});
    idxs[EPS::LED_LIVE] = Value("Live", {"live"});
    idxs[EPS::LED_SA2] = Value("Status A2", {"a2", "status_a2"});
    idxs[EPS::LED_SA1] = Value("Status A1", {"a1", "status_a1"});
    idxs[EPS::LED_SB2] = Value("Status B2", {"b2", "status_b2"});
    idxs[EPS::LED_SB1] = Value("Status B1", {"b1", "status_b1"});
    idxs[EPS::LED_SC2] = Value("Status C2", {"c2", "status_c2"});
    idxs[EPS::LED_SC1] = Value("Status C1", {"c1", "status_c1"});
    ValueMap vals;
    vals[EPS::LED_MODE_OFF] =
        Value("Off", {"off", "0", "2", "disabled", "false"});
    vals[EPS::LED_MODE_ON] = Value("On", {"on", "1", "enabled", "true"});
    vals[EPS::LED_MODE_BLINK_2HZ] = Value("Fast", {"fast", "5", "error"});
    vals[EPS::LED_MODE_BLINK_1HZ] = Value("Medium", {"medium", "4", "waiting"});
    vals[EPS::LED_MODE_BLINK_0_5HZ] = Value("Slow", {"slow", "3", "sleeping"});
    if (FLAGS_list) {
      Print("Indexes", idxs);
      Print("Values", vals);
      return 0;
    }
    if (FLAGS_get) return Error("The -led argument does not support -get");
    if (!FLAGS_set.empty()) {
      uint32_t mask;
      if (!Mask(idxs, input, mask, true))
        return Error("There was a problem parsing the indexes you supplied.");
      uint32_t value;
      if (!Valid(vals, FLAGS_set, value))
        return Error("The supplied value for -set is not supported");
      if (!eps.SetLeds(mask, static_cast<EPS::LedMode>(value)))
        return Error("There was a problem setting the charge values. Aborted.");
      return Print("LEDs channels set successfully");
    }
    return HelpConf("led", "Configure LEDs");
  }

  // If we get here there was no valid command, so print useage info and exit
  return Help();
}
