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

#include <cpu_mem_monitor/cpu.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

namespace cpu_mem_monitor {

Core::Core(const std::string sys_cpu_path, int id)
  : sys_cpu_path_(sys_cpu_path)
  , id_(id), always_on_(false) {
}

Core::~Core(void) {
}

bool Core::Init(void) {
  // Check if sys files are there.
  std::vector<std::string> files;

  files.push_back(sys_cpu_path_ + "/online");
  files.push_back(sys_cpu_path_ + "/cpufreq/scaling_cur_freq");
  files.push_back(sys_cpu_path_ + "/cpufreq/scaling_max_freq");
  files.push_back(sys_cpu_path_ + "/cpufreq/scaling_min_freq");

  for (int i = 0; i < static_cast<int>(files.size()); i++) {
    std::ifstream is(files[i]);

    if (!is.is_open()) {
      if (i == 0) {
        std::cerr << "CPU0 is forced to be always on" << std::endl;
        always_on_ = true;
        continue;
      }

      std::cerr << "Error opening file '" << files[i] << "': " <<
        std::strerror(errno) << std::endl;

      return false;
    }

    is.close();
  }

  return true;
}

int Core::GetMinFreq(void) {
  return GetIntValue(sys_cpu_path_ + "/cpufreq/scaling_min_freq");
}

int Core::GetCurFreq(void) {
  return GetIntValue(sys_cpu_path_ + "/cpufreq/scaling_cur_freq");
}

int Core::GetMaxFreq(void) {
  return GetIntValue(sys_cpu_path_ + "/cpufreq/scaling_max_freq");
}

bool Core::IsOn(void) {
  if (always_on_)
    return true;
  return GetIntValue(sys_cpu_path_ + "/online") == 1 ? true : false;
}

int Core::GetId(void) {
  return id_;
}

int Core::GetIntValue(const std::string &file) {
  std::string line;
  std::ifstream is(file);

  if (!is.is_open()) {
    std::cerr << "error opening file '" << file << "': " <<
      std::strerror(errno) << std::endl;

    return -1;
  }

  std::getline(is, line);

  int value = std::stoi(line);

  is.close();

  return value;
}

ThermalZone::ThermalZone(const std::string sys_thermal_path, int id)
  : sys_thermal_path_(sys_thermal_path)
  , id_(id) {
}

ThermalZone::~ThermalZone(void) {
}

bool ThermalZone::Init(void) {
  // Check if 'temp' file exists.
  std::ifstream temp(sys_thermal_path_ + "/temp");

  if (!temp.is_open()) {
    std::cerr << "Error opening file: "
      << std::strerror(errno) << std::endl;

    return false;
  }

  temp.close();

  return true;
}

int ThermalZone::GetId(void) {
  return id_;
}

double ThermalZone::GetTemperature(double scale) {
  std::string line;
  std::ifstream is(sys_thermal_path_ + "/temp");

  if (!is.is_open()) {
    std::cerr << "Error opening file: " <<
      std::strerror(errno) << std::endl;

    return -1.0;
  }

  std::getline(is, line);

  int value = std::stoi(line);

  is.close();

  return value * scale;
}

Cpu::Cpu(const std::string sys_cpu_path,
  const std::string sys_thermal_path)
  : sys_cpu_path_(sys_cpu_path)
  , sys_thermal_path_(sys_thermal_path) {
  cores_.clear();
  thermal_zones_.clear();
}

Cpu::~Cpu(void) {
}

bool Cpu::Init(void) {
  std::string line;
  std::ifstream present(sys_cpu_path_ + "/present");

  if (!present.is_open()) {
    std::cerr << "Error opening file '" << sys_cpu_path_ << "/present" << "': " <<
      std::strerror(errno) << std::endl;

    return false;
  }

  std::getline(present, line, '-');
  int min = std::stoi(line);
  std::getline(present, line, '-');
  int max = std::stoi(line);

  present.close();

  std::cout << "CPU cores available: cpu"
    << min << "-" << max << std::endl;

  for (int id = min; id <= max; id++) {
    std::string s = sys_cpu_path_ + "/cpu" + std::to_string(id);
    cores_.push_back(new Core(s, id));
  }

  for (int i = 0; i < static_cast<int>(cores_.size()); i++) {
    if (!cores_[i]->Init()) {
      std::cerr << "Failed to initialize cpu" << cores_[i]->GetId() <<
        ": " << std::strerror(errno) << std::endl;

      return false;
    }
  }

  // Detecting thermal zones.
  // FIXME: There's should be a better way for this...
  // This assume that the maximum number of thermal zones is 20.
  for (int id = 0; id < 20; id++) {
    std::string file = sys_thermal_path_ + "/thermal_zone"
      + std::to_string(id);
    std::ifstream is(file);

    if (!is.is_open())
      break;

    thermal_zones_.push_back(new ThermalZone(file, id));
  }

  std::cout << "# thermal zones available: " <<
    static_cast<int>(thermal_zones_.size()) << std::endl;

  for (int i = 0; i < static_cast<int>(thermal_zones_.size()); i++) {
    if (!thermal_zones_[i]->Init()) {
      std::cerr << "Failed to initialize thermal_zone"
        << thermal_zones_[i]->GetId() << ": "
        << std::strerror(errno) << std::endl;

      return false;
    }
  }

  return true;
}

size_t Cpu::GetNumCores(void) {
  return cores_.size();
}

const std::vector<Core *> Cpu::GetCores(void) {
  return cores_;
}

const std::vector<ThermalZone *> Cpu::GetThermalZones(void) {
  return thermal_zones_;
}

double Cpu::GetTemperature(double scale) {
  double temp = 0;

  for (size_t i = 0; i < thermal_zones_.size(); i++) {
    // FIXME: in mlp, thermal_zone11 uses mC but C for others..
    // Ignore thermal_zone11
    if (i == 11)
      continue;

    double t = thermal_zones_[i]->GetTemperature();

    if (t > temp)
      temp = t;
  }

  return temp * scale;
}

}  // namespace cpu_mem_monitor
