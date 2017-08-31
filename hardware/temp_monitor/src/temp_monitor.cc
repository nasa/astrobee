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

#include "ADT7410.h"
#include "TCN75A.h"

namespace temp_monitor {

std::shared_ptr < TempMonitor > TempMonitorFactory::Create(
  std::string const& type, i2c::Device const& device) {
  if (type == "ADT7410") return std::shared_ptr < TempMonitor > (new ADT7410(device));
  if (type == "TCN75A") return std::shared_ptr < TempMonitor > (new TCN75A(device));
  return std::shared_ptr < TempMonitor > ();
}

std::vector < std::string > TempMonitorFactory::Enumeration() {
  std::vector < std::string > enumeration;
  enumeration.push_back("ADT7410");
  enumeration.push_back("TCN75A");
  return enumeration;
}

}  // namespace temp_monitor
