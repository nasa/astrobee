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

#ifndef HARDWARE_TEMP_MONITOR_SRC_TCN75A_H_
#define HARDWARE_TEMP_MONITOR_SRC_TCN75A_H_

#include <temp_monitor/temp_monitor.h>

namespace temp_monitor {

class TCN75A : public TempMonitor {
 public:
  explicit TCN75A(i2c::Device const& device);

  bool GetTemperature(double *temp);

 protected:
  bool Init(void);

 private:
  bool init_;

  enum Register {
    TEMPERATURE   = 0x00,
    CONFIGURATION = 0x01,
    HYSTERESIS    = 0x02,
    LIMIT         = 0x03
  };
};

}  // namespace temp_monitor

#endif  // HARDWARE_TEMP_MONITOR_SRC_TCN75A_H_
