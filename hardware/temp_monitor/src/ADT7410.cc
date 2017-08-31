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

#include "ADT7410.h"

namespace temp_monitor {

ADT7410::ADT7410(i2c::Device const& device) : TempMonitor(device), init_(false) {}

bool ADT7410::Init(void) {
  if (init_) return true;
  // 16-bit resolution.
  // Continuous conversion mode.
  // Other settings - Default.
  uint8_t config = 0x80;
  uint8_t reg = static_cast < uint8_t > (ADT7410::Register::CONFIGURATION);
  i2c::Error err;
  init_ = device_.WriteRegister(reg, config, &err) > 0;
  return init_;
}

bool ADT7410::GetTemperature(double *temp) {
  if (!Init()) return false;
  uint8_t data[2] = {0x00, };
  uint8_t reg = static_cast < uint8_t > (ADT7410::Register::TEMP_MSB);
  i2c::Error err;
  if (device_.ReadRegister(reg, data, 2, &err) < 0)
    return false;
  // 16-bit temperature data conversion.
  *temp = static_cast < int > (data[0] << 8 | (data[1] & 0x00FF)) / 128.0;
  return true;
}

}  // namespace temp_monitor
