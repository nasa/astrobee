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

#ifndef HARDWARE_TEMP_MONITOR_SRC_ADT7410_H_
#define HARDWARE_TEMP_MONITOR_SRC_ADT7410_H_

#include <temp_monitor/temp_monitor.h>

namespace temp_monitor {

class ADT7410 : public TempMonitor {
 public:
  explicit ADT7410(i2c::Device const& device);

  bool GetTemperature(double *temp);

 protected:
  bool Init(void);

 private:
  bool init_;

  enum Register {
    TEMP_MSB          = 0x00,
    TEMP_LSB          = 0x01,
    STATUS            = 0x02,
    CONFIGURATION     = 0x03,
    TEMP_HIGH_SET_MSB = 0x04,
    TEMP_HIGH_SET_LSB = 0x05,
    TEMP_LOW_SET_MSB  = 0x06,
    TEMP_LOW_SET_LSB  = 0x07,
    TEMP_CRIT_SET_MSB = 0x08,
    TEMP_CRIT_SET_LSB = 0x09,
    TEMP_HYST_SET     = 0x0A,
    ID                = 0x0B,
    SOFTWARE_RESET    = 0x2F
  };
};

}  // namespace temp_monitor

#endif  // HARDWARE_TEMP_MONITOR_SRC_ADT7410_H_
