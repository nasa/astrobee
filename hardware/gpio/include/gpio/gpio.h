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

#ifndef GPIO_GPIO_H_
#define GPIO_GPIO_H_

#include <stdint.h>

namespace gpio {

// These functions do nothing if we are not on ARM. Technically I could give
// the operators direct access to variables for the IO line, but I'm using
// functions instead so that in the event this is x86_64, nothing happens
// here.
class IOManager {
  volatile uint32_t* gpio4_;
  volatile uint32_t* iomux_;

 public:
  // Constants that are specific to iMX6 DL, which is what was used on Prototype 3 LLP
  // This is the 32bit address for device mux for output pin GPIO19
  static constexpr uint32_t kIOMUXC_SW_MUX_CTL_PAD_GPIO19 = static_cast<uint32_t>(0x20e0220);
  // GPIO19 PIN corresponds to signal GPIO04_IO05
  static constexpr uint32_t kGPIO4_DR = static_cast<uint32_t>(0x20A8000);
  static constexpr uint32_t kGPIO4_GDIR = static_cast<uint32_t>(0x20A8004);
  static constexpr uint32_t kGPIO4_PSR = static_cast<uint32_t>(0x20A8008);
  static constexpr uint32_t kPageSize = 4096u;
  static constexpr uint32_t kPageLMask = kPageSize - 1;
  static constexpr uint32_t kPageHMask = ~(kPageSize - 1);

  // To find more of these magical constants, please read i.MX 6Solo/6DualLite
  // Applications Processor Reference Manual.
  //
  // Again, this is specific to Prototype 3's LLP

  IOManager();

  void InitializeGPIO19();
  void ToggleGPIO19();
};

}  // end namespace gpio

#endif  // GPIO_GPIO_H_
