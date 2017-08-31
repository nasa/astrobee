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


#include <i2c/i2c_new.h>

#include <dock/dock.h>

#include <glog/logging.h>

namespace dock {

namespace {

constexpr std::uint8_t REG_INPUT  = 0x00;
constexpr std::uint8_t REG_OUTPUT = 0x01;   // unused
constexpr std::uint8_t REG_INVERT = 0x02;
constexpr std::uint8_t REG_CONFIG = 0x03;

constexpr std::uint8_t LOOP_PIN  = 0x02;

}

Dock::Dock(i2c::Device const& dock, i2c::Device const& loop)
  : dock_(dock), loop_(loop) {
}

bool Dock::Init(i2c::Error *ec) {
  // read config & inversion bits
  std::uint8_t inversion, config;
  if (loop_.ReadRegister(REG_CONFIG, &config, 1, ec) < 0) {
    LOG(ERROR) << "error reading config register";
    return false;
  }

  if (loop_.ReadRegister(REG_INVERT, &inversion, 1, ec) < 0) {
    LOG(ERROR) << "error reading polarity register";
    return false;
  }

  inversion &= ~LOOP_PIN;  // set pin to be non-inverted
  config |= LOOP_PIN;  // set pin to be output

  if (loop_.WriteRegister(REG_INVERT, inversion, ec) < 0) {
    LOG(ERROR) << "error setting polarity";
    return false;
  }

  if (loop_.WriteRegister(REG_CONFIG, config, ec) < 0) {
    LOG(ERROR) << "error setting pin configuration";
    return false;
  }

  return true;
}

bool Dock::Docked(bool *docked, i2c::Error *ec) {
  std::uint8_t in;

  if (loop_.ReadRegister(REG_INPUT, &in, 1, ec) < 0) {
    LOG(ERROR) << "error reading input";
    return false;
  }

  *docked = !(in & LOOP_PIN);

  return true;
}

namespace {

// shamelessly stolen from the eps fw test program and converted to our style
std::uint8_t ComputeChecksum(std::uint8_t *buf, std::uint16_t size) {
  std::uint8_t checksum = 0xFF;

  for (std::uint32_t i = 0; i < size; i++) {
    checksum ^= buf[i];
  }

  return checksum;
}

}  // end namespace

namespace {

constexpr std::uint8_t kCmdEnableActuators = 0x01;

}  // end namespace

bool Dock::Undock(Bay bay, i2c::Error *ec) {
  std::uint8_t cmd[] = {
    kCmdEnableActuators,  // command
    5,                    // length as 16-bit little-endian integer (LSB)
    0,                    // (MSB)
    0,                    // sub-power state
    0,                    // reset state
    0,                    // LED state
    bay,                  // actuator state
    0  };                 // checksum

  cmd[7] = ComputeChecksum(cmd, 7);

  if (dock_.Write(cmd, 8, ec) < 0) {
    LOG(ERROR) << "error activating actuators";
    return false;
  }

  return true;
}

}  // namespace dock
