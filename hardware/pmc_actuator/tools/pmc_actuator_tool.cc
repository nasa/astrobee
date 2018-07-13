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

// PMC helper libraries
#include <pmc_actuator/pmc_actuator.h>

// C includes
#include <unistd.h>
#include <cerrno>
#include <cstring>

// C++11 STL includes
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// Default parameter values
#define DEFAULT_CSV_FILE "nozzle_identification.csv"
#define DEFAULT_I2C_DEV "/dev/i2c-2"
#define DEFAULT_I2C_ADDRESSES \
  { 0x20, 0x21 }
#define DEFAULT_I2C_RETRIES 3
#define DEFAULT_CONTROL_RATE_HZ 62.5
#define MIN_BLOWER_SPEED 0
#define MAX_BLOWER_SPEED 232
// #define MIN_NOZZLE_POSITION 25
// #define MAX_NOZZLE_POSITION 90
#define MIN_NOZZLE_POSITION 0
#define MAX_NOZZLE_POSITION 255

#define STUB_CMDS_FILENAME "/tmp/hw/pmc/commands"
#define STUB_TELEM_FILENAME "/tmp/hw/pmc/telemetry"

// Timestamped list of commands
typedef std::map<double, pmc_actuator::Command> PmcCommandList;

// packed data for the PMC
struct PmcActuatorData {
  pmc_actuator::Telemetry telemetry;  // Telemetry returned from the PMC
  PmcCommandList plan;                // Commands for this PMC
  PmcCommandList::iterator index;     // Curent index in the plan
  pmc_actuator::Command command;      // The current command being processed
};

// Prints out telemetry or not
bool kLogging = false;

// Print extra debug information (for now spits out the plan read)
bool kDebug = false;

// Repeat the plan indefinitely or not
bool kLooping = false;

// Mutex for changing data
std::mutex data_mutex_;

// A null command, which is useful for initialization / E-Stop
pmc_actuator::Command null_command_;

// A time point corresponding to the start
std::chrono::time_point<std::chrono::steady_clock> plan_start_time_ =
    std::chrono::steady_clock::now();

// Convenience declarations
typedef std::shared_ptr<pmc_actuator::PmcActuatorBase> PmcActuatorPtr;
typedef std::pair<PmcActuatorPtr, PmcActuatorData> PmcActuatorPair;
typedef std::vector<PmcActuatorPair> PmcActuatorVec;

// Grab an unsigned integer from user input with type and value checking
uint32_t InputUnsignedInteger(uint32_t min, uint32_t max) {
  uint32_t choice;
  while (true) {
    std::cout << "Input choice in range [" << min << ":" << max << "] > ";
    std::string input;
    getline(std::cin, input);
    std::stringstream ss(input);
    if (ss >> choice && choice >= min && choice <= max) return choice;
    std::cerr << "Number not in range [" << min << ":" << max
              << "], please try again" << std::endl;
  }
  std::cerr << "Got to an unreachable section of code" << std::endl;
  return 0;
}

// Grab a file name and check that the file can be read
std::string InputFile() {
  std::string choice;
  while (true) {
    std::cout << std::endl << "Input choice > ";
    getline(std::cin, choice);
    std::ifstream f(choice.c_str());
    if (f.good()) return choice;
    std::cerr << "Could not open the specified file " << choice << std::endl;
  }
  std::cerr << "Got to an unreachable section of code" << std::endl;
  return "";
}

// Pmc menu
void PmcMenu(PmcActuatorPair &pmc_pair) {
  std::cout << "*******************************************************"
            << std::endl;
  std::cout << "****************** SUB MENU FOR PMC *******************"
            << std::endl;
  std::cout << "*******************************************************"
            << std::endl;
  std::cout << "********************* TELEMETRY ***********************"
            << std::endl;
  if (pmc_pair.first->GetTelemetry(&pmc_pair.second.telemetry)) {
    std::cout << "Motor speed: "
              << static_cast<int>(pmc_pair.second.telemetry.motor_speed)
              << std::endl;
    std::cout << "6V current: "
              << static_cast<int>(pmc_pair.second.telemetry.v6_current)
              << std::endl;
    std::cout << "Pressure: "
              << static_cast<int>(pmc_pair.second.telemetry.pressure)
              << std::endl;
    std::cout << "Status 1: "
              << static_cast<int>(pmc_pair.second.telemetry.status_1.asUint8)
              << std::endl;
    std::cout << "Status 2: "
              << static_cast<int>(pmc_pair.second.telemetry.status_2.asUint8)
              << std::endl;
    std::cout << "Command ID: "
              << static_cast<int>(pmc_pair.second.telemetry.command_id)
              << std::endl;
    for (size_t i = 0; i < pmc_actuator::kTemperatureSensorsCount; i++)
      std::cout << "Temperature " << (i + 1) << ": "
                << static_cast<int>(pmc_pair.second.telemetry.temperatures[i])
                << std::endl;
  } else {
    std::cout << "Error: telemetry could not be obtained for this PMC"
              << std::endl;
  }
  std::cout << "********************** COMMAND *#**********************"
            << std::endl;
  std::cout << "0. Exit without changing anything" << std::endl;
  data_mutex_.lock();
  pmc_actuator::Command cmd = pmc_pair.second.command;
  data_mutex_.unlock();
  for (size_t i = 0; i < pmc_actuator::kNumNozzles; i++)
    std::cout << (i + 1) << ". Set nozzle " << (i + 1) << " value (current: "
              << static_cast<int>(cmd.nozzle_positions[i]) << ")" << std::endl;
  std::cout << (pmc_actuator::kNumNozzles + 1) << ". Set value for all nozzles"
            << std::endl;
  std::cout << (pmc_actuator::kNumNozzles + 2)
            << ". Set motor speed (current: "
            << static_cast<int>(cmd.motor_speed) << ")" << std::endl;
  std::cout << (pmc_actuator::kNumNozzles + 3)
            << ". Get firmware version" << std::endl;
  std::cout << "*******************************************************"
            << std::endl;
  // Keep looping until we have valid input
  uint8_t choice = InputUnsignedInteger(0, pmc_actuator::kNumNozzles + 3);
  // Do something based on the choice
  switch (choice) {
    case 0:
      break;
    default: {
      std::cerr << "Enter nozzle value:" << std::endl;
      uint32_t val = InputUnsignedInteger(0, 255);
      data_mutex_.lock();
      pmc_pair.second.command.nozzle_positions[choice - 1] = val;
      data_mutex_.unlock();
      std::cout << "Success" << std::endl;
      break;
    }
    case pmc_actuator::kNumNozzles + 1: {
      std::cerr << "Enter value for all nozzles:" << std::endl;
      uint32_t val = InputUnsignedInteger(0, 255);
      data_mutex_.lock();
      for (size_t i = 0; i < pmc_actuator::kNumNozzles; i++)
        pmc_pair.second.command.nozzle_positions[i] = val;
      data_mutex_.unlock();
      std::cout << "Success" << std::endl;
      break;
    }
    case pmc_actuator::kNumNozzles + 2: {
      std::cerr << "Enter motor speed:" << std::endl;
      uint32_t val = InputUnsignedInteger(0, 249);
      data_mutex_.lock();
      pmc_pair.second.command.motor_speed = val;
      data_mutex_.unlock();
      std::cout << "Success" << std::endl;
      break;
    }
    case pmc_actuator::kNumNozzles + 3: {
      std::cout << "Firmware info:" << std::endl;
      std::string hash;
      if (pmc_pair.first->GetFirmwareHash(hash))
        std::cout << "Compile hash:" << hash << std::endl;
      else
        std::cout << "Compile hash: <not yet received>" << std::endl;
      std::string time;
      if (pmc_pair.first->GetFirmwareTime(time))
        std::cout << "Compile time:" << time << std::endl;
      else
        std::cout << "Compile time: <not yet received>" << std::endl;
      break;
    }
  }
}

// Print a main menu
bool RunFile(PmcActuatorVec &pmcs, std::string &csv) {
  // Clear all of the current plan information and reset the command to null
  data_mutex_.lock();
  for (size_t p = 0; p < pmcs.size(); p++) {
    pmcs[p].second.plan.empty();
  }
  data_mutex_.unlock();
  // Now load the data from the file
  std::ifstream input(csv.c_str());
  std::string line;
  // std::cout << "filename:" << csv << std::endl;
  // std::cout << input.good() << std::endl;
  int counter = 0;
  while (std::getline(input, line)) {
    counter++;
    // Skip comments and empty lines
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\0') continue;
    // Grab the list of numbers
    std::stringstream str(line);
    std::vector<double> numbers;
    double value;
    while (str >> value) {
      numbers.push_back(value);
    }
    // Check that we have enough values for the PMCs specified
    if (numbers.size() != (1 + pmcs.size() * (pmc_actuator::kNumNozzles + 1))) {
      std::cerr << "Line #" << counter << " is malformed..." << std::endl;
    } else {
      // Add the command to the plan - the map data type keeps the commands
      // ordered in time
      data_mutex_.lock();
      for (size_t p = 0; p < pmcs.size(); p++) {
        pmcs[p].second.plan[numbers[0]].mode = pmc_actuator::CmdMode::NORMAL;
        pmcs[p].second.plan[numbers[0]].motor_speed =
            numbers[1 + p * (pmc_actuator::kNumNozzles + 1)];
        if (pmcs[p].second.plan[numbers[0]].motor_speed > MAX_BLOWER_SPEED)
          pmcs[p].second.plan[numbers[0]].motor_speed = MAX_BLOWER_SPEED;
        for (size_t n = 0; n < pmc_actuator::kNumNozzles; n++) {
          pmcs[p].second.plan[numbers[0]].nozzle_positions[n] =
              numbers[2 + n + p * (pmc_actuator::kNumNozzles + 1)];
          if (pmcs[p].second.plan[numbers[0]].nozzle_positions[n] <
              MIN_NOZZLE_POSITION)
            pmcs[p].second.plan[numbers[0]].nozzle_positions[n] =
                MIN_NOZZLE_POSITION;
          if (pmcs[p].second.plan[numbers[0]].nozzle_positions[n] >
              MAX_NOZZLE_POSITION)
            pmcs[p].second.plan[numbers[0]].nozzle_positions[n] =
                MAX_NOZZLE_POSITION;
        }
      }
    }
    data_mutex_.unlock();
  }
  if (kDebug) {
    // printout plan
    for (PmcActuatorVec::iterator it = pmcs.begin(); it != pmcs.end(); it++) {
      std::cout << "PMC @ 0x" << std::hex << it->first->GetAddress() << ":"
                << std::dec << std::endl;
      for (PmcCommandList::iterator jt = it->second.plan.begin();
           jt != it->second.plan.end(); jt++) {
        std::cout << "ts=" << jt->first << " ";
        std::cout << "speed="
                  << static_cast<unsigned int>(jt->second.motor_speed);
        std::cout << " nozzles=";
        for (int i = 0; i < pmc_actuator::kNumNozzles; i++) {
          std::cout << static_cast<unsigned int>(jt->second.nozzle_positions[i])
                    << " ";
        }
        std::cout << std::endl;
      }
    }
  }
  data_mutex_.lock();
  // reset the indexes
  for (PmcActuatorVec::iterator it = pmcs.begin(); it != pmcs.end(); it++) {
    it->second.index = it->second.plan.begin();
  }
  data_mutex_.unlock();
  // The last thing to do is to set the plan run time to now
  plan_start_time_ = std::chrono::steady_clock::now();
  // Success!
  return true;
}

// Print a main menu
bool MainMenu(PmcActuatorVec &pmcs, std::string &csv) {
  // Print title
  std::cout << std::endl;
  std::cout << "*******************************************************"
            << std::endl;
  std::cout << "*********** ASTROBEE PMC ACTUATOR HOST TEST ***********"
            << std::endl;
  std::cout << "*******************************************************"
            << std::endl;
  std::cout << "0. Quit" << std::endl;
  std::cout << "*******************************************************"
            << std::endl;
  for (size_t p = 0; p < pmcs.size(); p++) {
    std::string name = (p == 0 ? "right" : "left");
    std::cout << (p + 1) << ". Command or view PMC"
              << (p + 1) << " (" << name << ")" << std::endl;
  }
  std::cout << (pmcs.size() + 1) << ". E-Stop       " << std::endl;
  std::cout << (pmcs.size() + 2) << ". Log ON/OFF   " << std::endl;
  std::cout << (pmcs.size() + 3) << ". Enter CSV file" << std::endl;
  if (!csv.empty())
    std::cout << (pmcs.size() + 4) << ". Run " << csv << std::endl;

  std::cout << "*******************************************************"
            << std::endl;
  // Keep looping until we have valid input
  uint8_t choice = InputUnsignedInteger(0, pmcs.size() + 4);
  // Do something based on the choice
  if (choice == 0) {
    std::cout << "Goodbye." << std::endl;
    return false;
  } else if (choice == pmcs.size() + 1) {
    data_mutex_.lock();  // Lock PMC resource
    for (size_t p = 0; p < pmcs.size(); p++) {
      pmcs[p].second.plan.empty();  // Remove all plan data
      null_command_.command_id =
          pmcs[p].second.command.command_id;   // Preserve the command ID
      pmcs[p].second.command = null_command_;  // Copy over the NULL command
    }
    data_mutex_.unlock();  // Unlock PMC resource
  } else if (choice == pmcs.size() + 2) {
    kLogging = !kLogging;
  } else if (choice == pmcs.size() + 3) {
    std::cout << "Enter file name:" << std::endl;
    csv = InputFile();
  } else if (choice == pmcs.size() + 4) {
    if (!RunFile(pmcs, csv)) {
      std::cerr << "Error: problem reading file" << std::endl;
      return false;
    }
  } else {
    PmcMenu(pmcs[choice - 1]);
  }
  return true;
}

// A background thread to handle sending control over i2c - the naive way to
// create a rate loop is to simply call std::this_thread::sleep_for(duration)
// at the end of every iteration. However, the time taken to process all
// commands within this loop will be added to the sleep time, which means a
// delta error will creep in.
void ControlThread(PmcActuatorVec &pmcs, double rate, bool &finished) {
  std::chrono::time_point<std::chrono::steady_clock> next_loop =
      std::chrono::steady_clock::now();
  std::chrono::time_point<std::chrono::steady_clock> temp_time = next_loop;
  bool plan_restart = false;
  // Keep iterating until the main thread says stop
  while (!finished) {
    data_mutex_.lock();
    // Send the commands
    for (PmcActuatorVec::iterator it = pmcs.begin(); it != pmcs.end(); it++) {
      it->first->SendCommand(it->second.command);
      it->second.command.command_id++;
    }
    // Read telemetry
    for (PmcActuatorVec::iterator it = pmcs.begin(); it != pmcs.end(); it++) {
      it->first->GetTelemetry(&(it->second.telemetry));
      if (kLogging) {
        it->first->PrintTelemetry(std::cout, it->second.telemetry);
      }
    }
    // What will the ideal next loop time epoch be
    next_loop += std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::duration<double>(1.0 / rate));
    // Advance the plans by as much as required
    for (PmcActuatorVec::iterator it = pmcs.begin(); it != pmcs.end(); it++) {
      // std::cout << "current index = " << (it->second.index)->first <<
      // std::endl;
      for (PmcCommandList::iterator jt = it->second.index;
           jt != it->second.plan.end(); jt++) {
        temp_time = plan_start_time_ +
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::duration<double>(jt->first));
        if (next_loop >= temp_time) {
          jt->second.command_id =
              it->second.command.command_id;  // Preserve the command ID
          it->second.command = jt->second;    // Copy over the new command
          it->second.index++;                 // Increment the index
        } else {
          break;  // Time-ordering allows for breake here
        }
        if (kLooping && it->second.index == it->second.plan.end()) {
          it->second.index = it->second.plan.begin();
          plan_restart = true;
        }
      }
    }
    if (plan_restart) {
      plan_start_time_ = std::chrono::steady_clock::now();
      plan_restart = false;
    }
    data_mutex_.unlock();
    // Sleep until the next control tick, taking into account the loop time
    auto sleep_duration = next_loop - std::chrono::steady_clock::now();
    // auto sleep_ms =
    //  std::chrono::duration_cast<std::chrono::milliseconds>(sleep_duration);
    // std::cout << "sleep_ms=" << sleep_ms.count() << std::endl;
    std::this_thread::sleep_for(sleep_duration);
  }
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Default values
  std::string i2c_dev = DEFAULT_I2C_DEV;
  std::vector<int> i2c_addrs = DEFAULT_I2C_ADDRESSES;
  int i2c_retries = DEFAULT_I2C_RETRIES;
  std::string csv = DEFAULT_CSV_FILE;
  double rate = DEFAULT_CONTROL_RATE_HZ;
  bool fake_pmcs = false;
  std::string statefile("");
  std::fstream output_cmds;
  std::fstream output_telem;

  // Manipulate from stdin
  int c;
  while ((c = getopt(argc, argv, "hd:r:sgla:c:f:w:")) != -1) {
    switch (c) {
      case 'd':
        i2c_dev = std::string(optarg);
        break;
      case 'r': {
        std::stringstream stream(optarg);
        stream >> i2c_retries;
        if (stream.fail()) {
          std::cerr << "Error: Retry count " << optarg << " cannot be read."
                    << std::endl;
          return -3;
        }
        break;
      }
      case 's':
        fake_pmcs = true;
        break;
      case 'g':
        kDebug = true;
        break;
      case 'l':
        kLooping = true;
        break;
      case 'a': {
        i2c_addrs.clear();
        std::istringstream iss(optarg);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                  std::istream_iterator<std::string>(),
                  std::back_inserter(tokens));
        for (size_t i = 0; i < tokens.size(); i++) {
          int hex;
          if (sscanf(tokens[i].c_str(), "0x%X", &hex)) {
            i2c_addrs.push_back(hex);
          } else {
            std::cerr << "Error: PMC address " << tokens[i]
                      << " cannot be read. Expecting 0x prefix" << std::endl;
            return -3;
          }
        }
        if (i2c_addrs.empty()) {
          std::cerr << "Error: No valid PMC hex addresses supplied."
                    << std::endl;
          return -3;
        }
        break;
      }
      case 'c': {
        std::stringstream stream(optarg);
        stream >> rate;
        if (stream.fail()) {
          std::cerr << "Error: Rate " << optarg << " cannot be read."
                    << std::endl;
          return -3;
        }
        break;
      }
      case 'f': {
        std::ifstream f(optarg);
        csv = std::string(optarg);
        if (!f.good()) {
          std::cerr << "Error: File " << optarg << " cannot be read."
                    << std::endl;
          return -2;
        }
        break;
      }
      case 'w': {
        statefile = std::string(optarg);
        break;
      }
      default: {
        std::cout << std::endl;
        std::cout << "Usage: pmc_actuator_tool [OPTIONS]..." << std::endl;
        std::cout << "Generate PMC command" << std::endl;
        std::cout << "  -h                    Show usage and help" << std::endl;
        std::cout << "  -d bus                i2c bus character device"
                  << std::endl;
        std::cout << "  -r retries            i2c retries" << std::endl;
        std::cout << "  -a addr1 ... addrN    PMC i2c address (default: "
                  << std::endl;
        std::cout << "  -c rate               control rate" << std::endl;
        std::cout << "  -f file               CSV file with data" << std::endl;
        std::cout << "  -w file               Write state to file" << std::endl;
        std::cout << "  -l                    Loop over the plan" << std::endl;
        std::cout << "  -s                    Use PMC stubs" << std::endl;
        std::cout
            << "  -g                    Print more debug info (like plan read)"
            << std::endl;
        std::cout << "                        make sure /tmp/hw/pmc exist first"
                  << std::endl;
        std::cout << std::endl;
        return -1;
      }
    }
  }
  // Initialize the null command
  null_command_.motor_speed = MIN_BLOWER_SPEED;
  for (size_t n = 0; n < pmc_actuator::kNumNozzles; n++)
    null_command_.nozzle_positions[n] = MIN_NOZZLE_POSITION;
  null_command_.mode = pmc_actuator::CmdMode::NORMAL;
  // Try open the bus
  i2c::Error err;
  i2c::Bus::Ptr bus;
  if (!fake_pmcs) {
    bus = i2c::Open(i2c_dev, &err);
    if (!bus) {
      std::cerr << "Unable to open i2c bus ('" << i2c_dev
                << "'): " << std::strerror(err) << std::endl;
      return 1;
    }
    bus->SetRetries(i2c_retries);
  } else {
    output_cmds.open(STUB_CMDS_FILENAME,
                     std::fstream::out | std::fstream::trunc);
    output_telem.open(STUB_TELEM_FILENAME,
                      std::fstream::out | std::fstream::trunc);
  }
  // Try and open the devices
  PmcActuatorVec pmcs(i2c_addrs.size());
  data_mutex_.lock();
  for (size_t i = 0; i < i2c_addrs.size(); i++) {
    if (!fake_pmcs) {
      pmcs[i].first = PmcActuatorPtr(
          new pmc_actuator::PmcActuator(bus->DeviceAt(i2c_addrs[i])));
    } else {
      pmcs[i].first = PmcActuatorPtr(new pmc_actuator::PmcActuatorStub(
          i2c_addrs[i], output_cmds, output_telem));
    }
    pmcs[i].second.command = null_command_;  // Default is null command
    pmcs[i].second.plan.clear();             // Clear all existing plans
    pmcs[i].second.index = pmcs[i].second.plan.begin();  // set indexes
  }
  data_mutex_.unlock();

  // Special case: "state mode" (flag -w /some/file) is used to write a zero
  // length file if the PMCs are found, and a non-zero length file with an
  // error if the PMCs are not found
  if (!statefile.empty()) {
    std::ofstream ofs(statefile, std::ofstream::trunc);
    for (size_t i = 0; i < i2c_addrs.size(); i++) {
      pmc_actuator::Telemetry telemetry;
      if (!pmcs[i].first || !pmcs[i].first->GetTelemetry(&telemetry))
        ofs << "PMC " << i << " not detected" << std::endl;
    }
    ofs.close();
    return 0;
  }

  // This flag is passed by reference to the thread, so the thread knows when
  // to end
  bool finished = false;
  // Start timer in a background thread for sending commands
  std::thread thread(ControlThread, std::ref(pmcs), rate, std::ref(finished));
  // Load the main menu as a foreground task
  while (MainMenu(pmcs, csv)) {
  }
  // Tell the thread to finish (there will be a small delay while the control
  // loop times out)
  finished = true;
  // Join thee background thread
  thread.join();

  // Close stub stream
  if (fake_pmcs) {
    output_cmds.close();
    output_telem.close();
  }

  // Make for great success
  return 0;
}
