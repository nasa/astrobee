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

#ifndef TEMP_MONITOR_TEMP_MONITOR_H_
#define TEMP_MONITOR_TEMP_MONITOR_H_

#include <i2c/i2c_new.h>

#include <string>
#include <memory>
#include <map>
#include <vector>

namespace temp_monitor {

// The abstract interface through which temperature data is made available

class TempMonitor {
 public:
  explicit TempMonitor(i2c::Device const& device) : device_(device) {}
  virtual bool GetTemperature(double *temp) = 0;
 protected:
  i2c::Device device_;
};

typedef std::shared_ptr < TempMonitor > TempMonitorPtr;

// The internal interface used by developers to register derived classes

class TempMonitorFactory {
 public:
  static std::shared_ptr < TempMonitor > Create(std::string const& type, i2c::Device const& device);
  static std::vector < std::string > Enumeration();
};

}  // namespace temp_monitor

#endif  // TEMP_MONITOR_TEMP_MONITOR_H_
