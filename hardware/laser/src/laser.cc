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


#include <laser/laser.h>

#include <i2c/i2c.h>

#include <glog/logging.h>

#include <cstdint>

namespace laser {

namespace {

constexpr std::uint8_t REG_INPUT  = 0x00;  // unused
constexpr std::uint8_t REG_OUTPUT = 0x01;
constexpr std::uint8_t REG_INVERT = 0x02;
constexpr std::uint8_t REG_CONFIG = 0x03;

constexpr std::uint8_t LASER_PIN  = 0x20;

}  // namespace

Laser::Laser(i2c::Device const& dev)
  : dev_(dev), enabled_(false) {
}

bool Laser::Init(i2c::Error *ec) {
  // read config & inversion bits
  std::uint8_t inversion, config;
  if (dev_.ReadRegister(REG_CONFIG, &config, 1, ec) < 0) {
    LOG(ERROR) << "error reading config register";
    return false;
  }

  if (dev_.ReadRegister(REG_INVERT, &inversion, 1, ec) < 0) {
    LOG(ERROR) << "error reading polarity register";
    return false;
  }

  inversion &= ~LASER_PIN;  // set pin to be non-inverted
  config &= ~LASER_PIN;  // set pin to be output

  if (dev_.WriteRegister(REG_INVERT, inversion, ec) < 0) {
    LOG(ERROR) << "error setting polarity";
    return false;
  }

  if (dev_.WriteRegister(REG_CONFIG, config, ec) < 0) {
    LOG(ERROR) << "error setting pin configuration";
    return false;
  }

  if (!SetEnabled(false, ec)) {
    return false;
  }

  return true;
}

// XXX: there is currently a race condition with any other node that
// uses this same IO-expander chip. This is unlikely to occur in practice,
// but we should be aware of it.
bool Laser::SetEnabled(bool enabled, i2c::Error* ec) {
  // read output
  std::uint8_t output;
  if (dev_.ReadRegister(REG_OUTPUT, &output, 1, ec) < 0) {
    LOG(ERROR) << "error reading output register";
    return false;
  }

  // modify current settings
  if (enabled) {
    output |= LASER_PIN;
  } else {
    output &= ~LASER_PIN;
  }

  // set output
  if (dev_.WriteRegister(REG_OUTPUT, output, ec) < 0) {
    LOG(ERROR) << "error setting laser output";
    return false;
  }

  enabled_ = enabled;
  return true;
}

}  // namespace laser

