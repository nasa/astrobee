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

#include <gpio/gpio.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

#ifdef __arm__

gpio::IOManager::IOManager() {
  // Becoming, root destroyer of worlds, so that we may access /dev/mem without
  // asking to the user run us with sudo.
  if (setegid(15)) {
    std::cerr << "Unable to join kmem group" << std::endl;
  }

  // Since we are inside a virtual memory space, we are now access /dev/mem so
  // we can have straight access to real memory space.
  int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (mem_fd < 0) {
    std::cerr << "Unable to open /dev/mem" << std::endl;
  }

  // Memory map io the section of memory that we want.
  gpio4_ =
    reinterpret_cast<uint32_t*>(mmap(NULL, kPageSize, PROT_READ | PROT_WRITE,
                                     MAP_SHARED, mem_fd, kPageHMask & kGPIO4_DR));
  iomux_ =
    reinterpret_cast<uint32_t*>(mmap(NULL, kPageSize, PROT_READ | PROT_WRITE,
                                     MAP_SHARED, mem_fd, kPageHMask & kIOMUXC_SW_MUX_CTL_PAD_GPIO19));

  // Go back to unprivledged mode.
  close(mem_fd);
  if (setegid(getgid())) {
    std::cerr << "Unable to lose kmem privileges." << std::endl;
  }
}

void gpio::IOManager::InitializeGPIO19() {
  // Turn on SION and set the pin to be attached to the GPIO4 module.
  iomux_[(kPageLMask & kIOMUXC_SW_MUX_CTL_PAD_GPIO19) / 4] = 0x15;
  // Set the pin to be an output. FYI, GPIO19 is signal GPIO4_IO05. So here I'm
  // setting the 5th line (counting from zero) to be output (1).
  gpio4_[(kPageLMask & kGPIO4_GDIR) / 4] = 0x20;
  // Set the pin to be low.
  gpio4_[(kPageLMask & kGPIO4_DR) / 4] = 0x00;
}

void gpio::IOManager::ToggleGPIO19() {
  // I reading what the PIN currently is from the PSR register (which is being
  // set because of SION). Then I'm setting DR to be the opposite.
  gpio4_[(kPageLMask & kGPIO4_DR) / 4] = 0x20 & ~gpio4_[(kPageLMask & kGPIO4_PSR) / 4];
}

#else  // __arm__

gpio::IOManager::IOManager() {}
void gpio::IOManager::InitializeGPIO19() {}
void gpio::IOManager::ToggleGPIO19() {}

#endif  // __arm__
