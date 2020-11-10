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

#ifndef CPU_MEM_MONITOR_CPU_H_
#define CPU_MEM_MONITOR_CPU_H_

#include <string>
#include <vector>

namespace cpu_mem_monitor {

class Core {
 private:
  std::string sys_cpu_path_;
  int id_;
  bool always_on_;

 public:
  explicit Core(const std::string sys_cpu_path, int id);
  ~Core(void);

  bool Init(void);
  int GetId(void);
  bool IsOn(void);
  int GetMinFreq(void);
  int GetCurFreq(void);
  int GetMaxFreq(void);

 private:
  int GetIntValue(const std::string &file);
};

class ThermalZone {
 private:
  std::string sys_thermal_path_;
  int id_;

 public:
  explicit ThermalZone(const std::string sys_thermal_path, int id);
  ~ThermalZone(void);

  bool Init(void);
  int GetId(void);
  double GetTemperature(double scale = 1.0);
};

// This class represents a cpu in the system.
// This assumes sys file system exists.
class Cpu {
 private:
  std::string sys_cpu_path_;
  std::string sys_thermal_path_;
  std::vector<Core *> cores_;
  std::vector<ThermalZone *> thermal_zones_;

 public:
  explicit Cpu(const std::string sys_cpu_path,
    const std::string sys_thermal_path);
  ~Cpu(void);

  bool Init(void);
  size_t GetNumCores(void);
  const std::vector<Core *> GetCores(void);
  const std::vector<ThermalZone *> GetThermalZones(void);
  double GetTemperature(double scale = 1.0);
};

}  // namespace cpu_mem_monitor

#endif  // CPU_MEM_MONITOR_CPU_H_
