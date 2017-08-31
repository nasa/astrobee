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

#include "TCN75A.h"

namespace temp_monitor {

TCN75A::TCN75A(i2c::Device const& device) : TempMonitor(device), init_(false) {}

bool TCN75A::Init(void) {
  if (init_) return true;
  // One-Shot mode disabled.
  // 12-bit resolution.
  // Other settings - default.
  uint8_t config = 0x60;
  uint8_t reg = static_cast < uint8_t > (TCN75A::Register::CONFIGURATION);
  i2c::Error err;
  init_ = device_.WriteRegister(reg, config, &err) > 0;
  return init_;
}

bool TCN75A::GetTemperature(double *temp) {
  if (!Init()) return false;
  uint8_t reg = static_cast < uint8_t > (TCN75A::Register::TEMPERATURE);
  uint8_t data[2];
  i2c::Error err;
  if (device_.ReadRegister(reg, data, 2, &err) < 0)
    return false;
  *temp = static_cast < int > (data[0] << 8 | (data[1] & 0x00FF)) / 256.0;
  return true;
}

}  // namespace temp_monitor
